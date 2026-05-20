// SPDX-License-Identifier: Apache-2.0
//
// remote-usb-port.c — QEMU device that bridges a USB host bus to a
// remote simulation server speaking the `rusb_protocol`.
//
// This file is **distributed with chisel-usb** but is intended to be
// dropped into a QEMU fork under `hw/usb/remote-usb-port.c`.
//
// Build steps (in your QEMU fork):
//
//   1. Copy this file to `hw/usb/remote-usb-port.c`.
//   2. Copy `rusb_protocol.h` from chisel-usb/cosim/native/rusb-bridge/
//      to `hw/usb/rusb_protocol.h`.
//   3. Add an entry in `hw/usb/meson.build`:
//        softmmu_ss.add(when: 'CONFIG_USB', if_true:
//          files('remote-usb-port.c'))
//   4. Add Kconfig entry `CONFIG_USB_REMOTE` selectable from a board.
//   5. Rebuild QEMU.
//
// QEMU invocation example:
//   qemu-system-x86_64 ...
//     -device remote-usb-port,socket=/tmp/qemu-usb.sock,phy=utmi,speed=high
//
// The device presents itself to the guest as a USB device on the
// nearest USB host controller. Whenever the guest issues a token /
// data / handshake packet, this device serializes the packet to the
// remote socket using the `rusb_protocol` framing.
//
// v1 scope: bulk + control transfers via USB 2.0 UTMI. Isochronous
// scheduling, USB 3.x, and link power management are TODO.
//
// Copyright 2026 Open-DPU Project

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/sockets.h"
#include "qom/object.h"
#include "hw/core/qdev-properties.h"
#include "hw/usb/usb.h"
#include "qapi/error.h"

#include "rusb_protocol.h"

/* ===================================================================
 *  Type definitions
 * =================================================================== */

#define TYPE_REMOTE_USB_PORT "remote-usb-port"
OBJECT_DECLARE_SIMPLE_TYPE(RemoteUsbPort, REMOTE_USB_PORT)

struct RemoteUsbPort {
    USBDevice parent_obj;

    /* Configuration properties */
    char    *socket_path;
    char    *phy_str;      /* "utmi", "ulpi", "pipe" */
    char    *speed_str;    /* "low", "full", "high", "super", "super-plus" */

    /* Runtime state */
    int      fd;
    uint32_t next_seq;
};

/* ===================================================================
 *  Forward declarations
 * =================================================================== */

static void remote_usb_realize(USBDevice *dev, Error **errp);
static void remote_usb_unrealize(USBDevice *dev);
static void remote_usb_handle_control(USBDevice *dev, USBPacket *p,
                                      int request, int value, int index,
                                      int length, uint8_t *data);
static void remote_usb_handle_data(USBDevice *dev, USBPacket *p);

/* ===================================================================
 *  Property table
 * =================================================================== */

static const Property remote_usb_properties[] = {
    DEFINE_PROP_STRING("socket", RemoteUsbPort, socket_path),
    DEFINE_PROP_STRING("phy",    RemoteUsbPort, phy_str),
    DEFINE_PROP_STRING("speed",  RemoteUsbPort, speed_str),
};

/* ===================================================================
 *  Helper: encode a USBPacket into PACKET_OUT bytes
 * =================================================================== */

static uint8_t parse_phy(const char *s) {
    if (!s) return RUSB_PHY_UTMI;
    if (!strcasecmp(s, "utmi")) return RUSB_PHY_UTMI;
    if (!strcasecmp(s, "ulpi")) return RUSB_PHY_ULPI;
    if (!strcasecmp(s, "pipe")) return RUSB_PHY_PIPE;
    return RUSB_PHY_UTMI;
}

static uint8_t parse_speed(const char *s) {
    if (!s) return RUSB_SPEED_HIGH;
    if (!strcasecmp(s, "low"))         return RUSB_SPEED_LOW;
    if (!strcasecmp(s, "full"))        return RUSB_SPEED_FULL;
    if (!strcasecmp(s, "high"))        return RUSB_SPEED_HIGH;
    if (!strcasecmp(s, "super"))       return RUSB_SPEED_SUPER;
    if (!strcasecmp(s, "super-plus"))  return RUSB_SPEED_SUPER_PLUS;
    return RUSB_SPEED_HIGH;
}

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

static void remote_usb_realize(USBDevice *dev, Error **errp) {
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);

    if (!s->socket_path) {
        error_setg(errp, "remote-usb-port: 'socket' property is required");
        return;
    }

    s->fd = unix_connect(s->socket_path, errp);
    if (s->fd < 0) return;
    s->next_seq = 1;   /* odd = downstream */

    /* TODO: send HELLO + wait for ATTACH from the simulation server.
     *       For now we just assume the simulation server is ready. */
    (void)parse_phy(s->phy_str);
    (void)parse_speed(s->speed_str);

    /* No static descriptor table: every SETUP is forwarded to the
     * simulation server, which generates descriptors on demand. */
    dev->speed     = USB_SPEED_HIGH;
    dev->speedmask = USB_SPEED_MASK_HIGH;
}

static void remote_usb_unrealize(USBDevice *dev) {
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);
    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }
}

/* ===================================================================
 *  USB packet handling
 * =================================================================== */
//
//  QEMU's USB core invokes `handle_control` for SETUP-stage control
//  transfers and `handle_data` for bulk / interrupt / isoc data
//  transactions. We will serialize each packet to the simulation
//  server using the `rusb_protocol` framing.
//
//  v1: synchronous request/reply, single in-flight packet. A future
//      revision should make this asynchronous (via qemu_set_fd_handler).
//
static void remote_usb_handle_control(USBDevice *dev, USBPacket *p,
                                      int request, int value, int index,
                                      int length, uint8_t *data) {
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);

    /* TODO: encode the SETUP token + optional DATA stage as
     *       RUSB_MSG_PACKET_OUT and forward to s->fd; await reply. */
    (void)s; (void)request; (void)value; (void)index;
    (void)length; (void)data;
    p->status = USB_RET_NAK;
}

static void remote_usb_handle_data(USBDevice *dev, USBPacket *p) {
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);

    /* TODO: encode `p` (pid, ep, data) as RUSB_MSG_PACKET_OUT and
     *       transmit on s->fd; await RUSB_MSG_PACKET_IN reply. */
    (void)s;
    p->status = USB_RET_NAK;   /* placeholder */
}

/* ===================================================================
 *  Class initialization
 * =================================================================== */

static void remote_usb_class_init(ObjectClass *klass, const void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->realize        = remote_usb_realize;
    uc->unrealize      = remote_usb_unrealize;
    uc->handle_control = remote_usb_handle_control;
    uc->handle_data    = remote_usb_handle_data;
    uc->product_desc   = "Remote USB Port (chisel-usb cosim bridge)";

    dc->desc          = "Remote USB Port (chisel-usb cosim bridge)";
    device_class_set_props(dc, remote_usb_properties);
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
}

static const TypeInfo remote_usb_info = {
    .name          = TYPE_REMOTE_USB_PORT,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(RemoteUsbPort),
    .class_init    = remote_usb_class_init,
};

static void remote_usb_register_types(void) {
    type_register_static(&remote_usb_info);
}

type_init(remote_usb_register_types);
