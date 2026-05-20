// SPDX-License-Identifier: Apache-2.0
//
// remote-usb-port.c - QEMU device that bridges a USB host bus to a
// remote chisel-usb simulation server speaking rusb_protocol v2.
//
// The device:
//   1. Connects to a Unix socket exposed by the simulator (server).
//   2. Sends HELLO.
//   3. For each USB transaction QEMU's USB core requests, it serializes
//      to a CONTROL_TXN / OUT_TXN / IN_TXN and reads back a TXN_RESULT.
//
// The simulator-side UsbHostBfm executes the wire-level USB 2.0
// packets (token + data + handshake with correct CRCs) against the
// chisel-usb device under simulation.
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

#include <sys/socket.h>
#include <sys/uio.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

static FILE *rusb_log_fp = NULL;
static void rusb_log_init(void) {
    if (!rusb_log_fp) {
        rusb_log_fp = fopen("/tmp/rusb-device.log", "w");
        if (rusb_log_fp) setvbuf(rusb_log_fp, NULL, _IOLBF, 0);
    }
}
#define RLOG(fmt, ...) do { rusb_log_init(); if (rusb_log_fp) { fprintf(rusb_log_fp, fmt "\n", ##__VA_ARGS__); } } while (0)

/* ===================================================================
 *  Type
 * =================================================================== */

#define TYPE_REMOTE_USB_PORT "remote-usb-port"
OBJECT_DECLARE_SIMPLE_TYPE(RemoteUsbPort, REMOTE_USB_PORT)

struct RemoteUsbPort {
    USBDevice parent_obj;

    /* properties */
    char *socket_path;
    char *speed_str;
    char *phy_str;

    /* runtime */
    int       fd;
    uint32_t  next_seq;

    /* per-endpoint OUT data toggle (bulk/intr). 16 dirs * 16 eps. */
    uint8_t   out_toggle[16];

    /* control transfer ep0 toggle handling is internal to wire */
};

/* ===================================================================
 *  Forward declarations
 * =================================================================== */

static void remote_usb_realize(USBDevice *dev, Error **errp);
static void remote_usb_unrealize(USBDevice *dev);
static void remote_usb_handle_reset(USBDevice *dev);
static void remote_usb_handle_control(USBDevice *dev, USBPacket *p,
                                      int request, int value, int index,
                                      int length, uint8_t *data);
static void remote_usb_handle_data(USBDevice *dev, USBPacket *p);

/* ===================================================================
 *  Property table
 * =================================================================== */

static const Property remote_usb_properties[] = {
    DEFINE_PROP_STRING("socket", RemoteUsbPort, socket_path),
    DEFINE_PROP_STRING("speed",  RemoteUsbPort, speed_str),
    DEFINE_PROP_STRING("phy",    RemoteUsbPort, phy_str),
};

/* ===================================================================
 *  Low-level I/O over the Unix socket (blocking)
 * =================================================================== */

static int read_all(int fd, void *buf, size_t n)
{
    uint8_t *p = (uint8_t *)buf;
    while (n > 0) {
        ssize_t r = read(fd, p, n);
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (r == 0) return -1;
        p += (size_t)r; n -= (size_t)r;
    }
    return 0;
}

static int send_frame(RemoteUsbPort *s, const void *payload, uint32_t len)
{
    rusb_frame_hdr_t hdr;
    hdr.magic  = RUSB_MAGIC;
    hdr.length = len;
    hdr.seq    = s->next_seq;
    s->next_seq += 2;

    struct iovec iov[2];
    iov[0].iov_base = &hdr; iov[0].iov_len = sizeof(hdr);
    iov[1].iov_base = (void *)payload; iov[1].iov_len = len;

    ssize_t total = sizeof(hdr) + len;
    ssize_t written = 0;
    while (written < total) {
        ssize_t w;
        if (written < (ssize_t)sizeof(hdr)) {
            iov[0].iov_base = (uint8_t *)&hdr + written;
            iov[0].iov_len  = sizeof(hdr) - written;
            w = writev(s->fd, iov, 2);
        } else {
            size_t off = written - sizeof(hdr);
            w = write(s->fd, (uint8_t *)payload + off, len - off);
        }
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (w == 0) return -1;
        written += w;
    }
    return 0;
}

/* Read one frame, returning the message type via *out_type and the
 * payload (including the msg_type byte) into buf. Returns the payload
 * length on success, -1 on error. */
static int recv_frame(RemoteUsbPort *s, uint8_t *buf, uint32_t buflen)
{
    rusb_frame_hdr_t hdr;
    if (read_all(s->fd, &hdr, sizeof(hdr)) < 0) return -1;
    if (hdr.magic != RUSB_MAGIC) {
        error_report("remote-usb-port: bad magic 0x%08x", hdr.magic);
        return -1;
    }
    if (hdr.length > buflen) {
        error_report("remote-usb-port: frame too large (%u > %u)",
                     hdr.length, buflen);
        return -1;
    }
    if (hdr.length > 0 && read_all(s->fd, buf, hdr.length) < 0) return -1;
    return (int)hdr.length;
}

/* ===================================================================
 *  Lifecycle
 * =================================================================== */

static void remote_usb_realize(USBDevice *dev, Error **errp)
{
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);
    RLOG("realize entered, socket=%s", s->socket_path ? s->socket_path : "(null)");

    if (!s->socket_path) {
        error_setg(errp, "remote-usb-port: 'socket' property is required");
        return;
    }

    s->fd = unix_connect(s->socket_path, errp);
    if (s->fd < 0) return;
    s->next_seq = 1;
    memset(s->out_toggle, 0, sizeof(s->out_toggle));

    /* Send HELLO */
    rusb_hello_t hello = {
        .msg_type = RUSB_MSG_HELLO,
        .version  = RUSB_PROTOCOL_VERSION,
    };
    if (send_frame(s, &hello, sizeof(hello)) < 0) {
        error_setg(errp, "remote-usb-port: failed to send HELLO");
        close(s->fd); s->fd = -1; return;
    }

    /* Speed: only high-speed supported in v1. */
    dev->speed     = USB_SPEED_HIGH;
    dev->speedmask = USB_SPEED_MASK_HIGH;
    (void)s->speed_str;
    (void)s->phy_str;
    RLOG("realize completed, fd=%d", s->fd);
}

static void remote_usb_unrealize(USBDevice *dev)
{
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);
    if (s->fd >= 0) {
        close(s->fd);
        s->fd = -1;
    }
}

static void remote_usb_handle_reset(USBDevice *dev)
{
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);
    RLOG("handle_reset fd=%d", s->fd);
    if (s->fd < 0) return;
    rusb_bus_reset_t br = { .msg_type = RUSB_MSG_BUS_RESET };
    if (send_frame(s, &br, sizeof(br)) < 0) {
        error_report("remote-usb-port: bus reset send failed");
    }
    memset(s->out_toggle, 0, sizeof(s->out_toggle));
    /* No reply expected for BUS_RESET. */
}

/* ===================================================================
 *  Helpers: map RUSB status -> USBPacket status
 * =================================================================== */

static int rusb_status_to_qemu(uint8_t status)
{
    switch (status) {
        case RUSB_STATUS_OK:    return USB_RET_SUCCESS;
        case RUSB_STATUS_NAK:   return USB_RET_NAK;
        case RUSB_STATUS_STALL: return USB_RET_STALL;
        default:                return USB_RET_IOERROR;
    }
}

/* ===================================================================
 *  Control transfer
 * =================================================================== */

static void remote_usb_handle_control(USBDevice *dev, USBPacket *p,
                                      int request, int value, int index,
                                      int length, uint8_t *data)
{
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);
    uint8_t bmRequestType = (request >> 8) & 0xFF;
    uint8_t bRequest      = request & 0xFF;
    bool is_in = (bmRequestType & USB_DIR_IN) != 0;
    RLOG("handle_control bmRT=0x%02x bReq=0x%02x val=0x%04x idx=0x%04x len=%d",
         bmRequestType, bRequest, value & 0xFFFF, index & 0xFFFF, length);

    /* Build CONTROL_TXN payload */
    uint8_t payload[sizeof(rusb_control_txn_t) + RUSB_MAX_PACKET];
    rusb_control_txn_t *ct = (rusb_control_txn_t *)payload;
    ct->msg_type        = RUSB_MSG_CONTROL_TXN;
    ct->addr            = dev->addr;
    ct->max_packet_size = 64;     /* HS default */
    ct->direction       = is_in ? 1 : 0;
    ct->setup[0] = bmRequestType;
    ct->setup[1] = bRequest;
    ct->setup[2] = value & 0xFF;
    ct->setup[3] = (value >> 8) & 0xFF;
    ct->setup[4] = index & 0xFF;
    ct->setup[5] = (index >> 8) & 0xFF;
    ct->setup[6] = length & 0xFF;
    ct->setup[7] = (length >> 8) & 0xFF;
    ct->data_len        = length;
    ct->reserved        = 0;

    uint32_t payload_len = sizeof(*ct);
    if (!is_in && length > 0) {
        if ((uint32_t)length > RUSB_MAX_PACKET) {
            p->status = USB_RET_IOERROR; return;
        }
        memcpy(payload + sizeof(*ct), data, length);
        payload_len += length;
    }

    if (send_frame(s, payload, payload_len) < 0) {
        p->status = USB_RET_IOERROR; return;
    }

    /* Receive TXN_RESULT */
    uint8_t rbuf[sizeof(rusb_txn_result_t) + RUSB_MAX_PACKET];
    int rlen = recv_frame(s, rbuf, sizeof(rbuf));
    if (rlen < (int)sizeof(rusb_txn_result_t)) {
        p->status = USB_RET_IOERROR; return;
    }
    rusb_txn_result_t *tr = (rusb_txn_result_t *)rbuf;
    if (tr->msg_type != RUSB_MSG_TXN_RESULT) {
        p->status = USB_RET_IOERROR; return;
    }

    int qstatus = rusb_status_to_qemu(tr->status);
    if (qstatus == USB_RET_SUCCESS) {
        if (is_in && tr->length > 0) {
            uint32_t copy = tr->length;
            if (copy > (uint32_t)length) copy = length;
            memcpy(data, rbuf + sizeof(*tr), copy);
            p->actual_length = copy;
        } else {
            p->actual_length = is_in ? 0 : length;
        }
        /* SET_ADDRESS bookkeeping: QEMU's USB core uses dev->addr to
         * route subsequent transactions. The wire transaction has
         * already informed the device-side SIE; mirror the change here. */
        if (bRequest == USB_REQ_SET_ADDRESS && !is_in) {
            dev->addr = value & 0x7F;
        }
    }
    p->status = qstatus;
}

/* ===================================================================
 *  Bulk / interrupt data transfer
 * =================================================================== */

static void remote_usb_handle_data(USBDevice *dev, USBPacket *p)
{
    RemoteUsbPort *s = REMOTE_USB_PORT(dev);
    uint8_t ep = p->ep->nr;
    RLOG("handle_data pid=0x%02x ep=%u iov=%u", p->pid, ep, (unsigned)p->iov.size);

    if (p->pid == USB_TOKEN_OUT) {
        /* Pull host data out of the iov */
        uint32_t len = p->iov.size;
        if (len > RUSB_MAX_PACKET) { p->status = USB_RET_IOERROR; return; }

        uint8_t buf[sizeof(rusb_out_txn_t) + RUSB_MAX_PACKET];
        rusb_out_txn_t *ot = (rusb_out_txn_t *)buf;
        ot->msg_type = RUSB_MSG_OUT_TXN;
        ot->addr     = dev->addr;
        ot->ep       = ep;
        ot->data_pid = s->out_toggle[ep & 0xF];
        ot->length   = (uint16_t)len;
        ot->reserved = 0;
        if (len > 0) {
            usb_packet_copy(p, buf + sizeof(*ot), len);
        }

        if (send_frame(s, buf, sizeof(*ot) + len) < 0) {
            p->status = USB_RET_IOERROR; return;
        }

        uint8_t rbuf[sizeof(rusb_txn_result_t) + 64];
        int rlen = recv_frame(s, rbuf, sizeof(rbuf));
        if (rlen < (int)sizeof(rusb_txn_result_t)) {
            p->status = USB_RET_IOERROR; return;
        }
        rusb_txn_result_t *tr = (rusb_txn_result_t *)rbuf;
        int qstatus = rusb_status_to_qemu(tr->status);
        if (qstatus == USB_RET_SUCCESS) {
            p->actual_length = len;
            s->out_toggle[ep & 0xF] ^= 1;
        }
        p->status = qstatus;
        return;
    }

    if (p->pid == USB_TOKEN_IN) {
        uint32_t maxlen = p->iov.size;
        rusb_in_txn_t it = {
            .msg_type   = RUSB_MSG_IN_TXN,
            .addr       = dev->addr,
            .ep         = ep,
            .reserved   = 0,
            .max_length = (uint16_t)(maxlen > 0xFFFF ? 0xFFFF : maxlen),
            .reserved2  = 0,
        };
        if (send_frame(s, &it, sizeof(it)) < 0) {
            p->status = USB_RET_IOERROR; return;
        }

        uint8_t rbuf[sizeof(rusb_txn_result_t) + RUSB_MAX_PACKET];
        int rlen = recv_frame(s, rbuf, sizeof(rbuf));
        if (rlen < (int)sizeof(rusb_txn_result_t)) {
            p->status = USB_RET_IOERROR; return;
        }
        rusb_txn_result_t *tr = (rusb_txn_result_t *)rbuf;
        int qstatus = rusb_status_to_qemu(tr->status);
        if (qstatus == USB_RET_SUCCESS) {
            uint32_t copy = tr->length;
            if (copy > maxlen) copy = maxlen;
            if (copy > 0) {
                usb_packet_copy(p, rbuf + sizeof(*tr), copy);
            }
            p->actual_length = copy;
        }
        p->status = qstatus;
        return;
    }

    p->status = USB_RET_STALL;
}

/* ===================================================================
 *  Class init
 * =================================================================== */

static void remote_usb_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->realize        = remote_usb_realize;
    uc->unrealize      = remote_usb_unrealize;
    uc->handle_reset   = remote_usb_handle_reset;
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

static void remote_usb_register_types(void)
{
    type_register_static(&remote_usb_info);
}

type_init(remote_usb_register_types);
