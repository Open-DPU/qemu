/*
 * QEMU USB HUB emulation
 *
 * Copyright (c) 2005 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "trace.h"
#include "hw/core/qdev-properties.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "desc.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qom/object.h"

#define MAX_PORTS 8

typedef struct RemoteUsbHubPort {
    USBPort port;
    uint16_t wPortStatus;
    uint16_t wPortChange;
} RemoteUsbHubPort;

struct RemoteUsbHubState {
    USBDevice dev;
    USBEndpoint *intr;
    uint32_t num_ports;
    bool port_power;
    QEMUTimer *port_timer;
    RemoteUsbHubPort ports[MAX_PORTS];
};

#define TYPE_REMOTE_USB_HUB "remote-usb-hub"
OBJECT_DECLARE_SIMPLE_TYPE(RemoteUsbHubState, REMOTE_USB_HUB)

#define ClearHubFeature         (0x2000 | USB_REQ_CLEAR_FEATURE)
#define ClearPortFeature        (0x2300 | USB_REQ_CLEAR_FEATURE)
#define GetHubDescriptor        (0xa000 | USB_REQ_GET_DESCRIPTOR)
#define GetHubStatus            (0xa000 | USB_REQ_GET_STATUS)
#define GetPortStatus           (0xa300 | USB_REQ_GET_STATUS)
#define SetHubFeature           (0x2000 | USB_REQ_SET_FEATURE)
#define SetPortFeature          (0x2300 | USB_REQ_SET_FEATURE)

#define PORT_STAT_CONNECTION    0x0001
#define PORT_STAT_ENABLE        0x0002
#define PORT_STAT_SUSPEND       0x0004
#define PORT_STAT_OVERCURRENT   0x0008
#define PORT_STAT_RESET         0x0010
#define PORT_STAT_POWER         0x0100
#define PORT_STAT_LOW_SPEED     0x0200
#define PORT_STAT_HIGH_SPEED    0x0400
#define PORT_STAT_TEST          0x0800
#define PORT_STAT_INDICATOR     0x1000

#define PORT_STAT_C_CONNECTION  0x0001
#define PORT_STAT_C_ENABLE      0x0002
#define PORT_STAT_C_SUSPEND     0x0004
#define PORT_STAT_C_OVERCURRENT 0x0008
#define PORT_STAT_C_RESET       0x0010

#define PORT_CONNECTION         0
#define PORT_ENABLE             1
#define PORT_SUSPEND            2
#define PORT_OVERCURRENT        3
#define PORT_RESET              4
#define PORT_POWER              8
#define PORT_LOWSPEED           9
#define PORT_HIGHSPEED          10
#define PORT_C_CONNECTION       16
#define PORT_C_ENABLE           17
#define PORT_C_SUSPEND          18
#define PORT_C_OVERCURRENT      19
#define PORT_C_RESET            20
#define PORT_TEST               21
#define PORT_INDICATOR          22

/* same as Linux kernel root hubs */

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings remote_hub_desc_strings = {
    [STR_MANUFACTURER] = "QEMU",
    [STR_PRODUCT]      = "chisel-usb Remote Hub",
    [STR_SERIALNUMBER] = "314159",
};

static const USBDescIface remote_hub_desc_iface = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 1,
    .bInterfaceClass               = USB_CLASS_HUB,
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | 0x01,
            .bmAttributes          = USB_ENDPOINT_XFER_INT,
            .wMaxPacketSize        = 1 + DIV_ROUND_UP(MAX_PORTS, 8),
            .bInterval             = 0xff,
        },
    }
};

static const USBDescDevice remote_hub_desc_device_full = {
    .bcdUSB                        = 0x0200,
    .bDeviceClass                  = USB_CLASS_HUB,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE | USB_CFG_ATT_SELFPOWER |
                                     USB_CFG_ATT_WAKEUP,
            .nif = 1,
            .ifs = &remote_hub_desc_iface,
        },
    },
};

static const USBDescIface remote_hub_desc_iface_hs = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 1,
    .bInterfaceClass               = USB_CLASS_HUB,
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | 0x01,
            .bmAttributes          = USB_ENDPOINT_XFER_INT,
            .wMaxPacketSize        = 1 + DIV_ROUND_UP(MAX_PORTS, 8),
            .bInterval             = 0x0c,
        },
    }
};

static const USBDescDevice remote_hub_desc_device = {
    .bcdUSB                        = 0x0200,
    .bDeviceClass                  = USB_CLASS_HUB,
    .bDeviceProtocol               = 1, /* single TT */
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE | USB_CFG_ATT_SELFPOWER |
                                     USB_CFG_ATT_WAKEUP,
            .nif = 1,
            .ifs = &remote_hub_desc_iface_hs,
        },
    },
};

static const USBDesc remote_hub_desc = {
    .id = {
        .idVendor          = 0x0409,
        .idProduct         = 0x55ab,
        .bcdDevice         = 0x0101,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full = &remote_hub_desc_device_full,
    .high = &remote_hub_desc_device,
    .str  = remote_hub_desc_strings,
};

static const uint8_t remote_hub_hub_descriptor[] =
{
        0x00,                   /*  u8  bLength; patched in later */
        0x29,                   /*  u8  bDescriptorType; Hub-descriptor */
        0x00,                   /*  u8  bNbrPorts; (patched later) */
        0x0a,                   /* u16  wHubCharacteristics; */
        0x00,                   /*   (per-port OC, no power switching) */
        0x01,                   /*  u8  bPwrOn2pwrGood; 2ms */
        0x00                    /*  u8  bHubContrCurrent; 0 mA */

        /* DeviceRemovable and PortPwrCtrlMask patched in later */
};

static bool remote_usb_hub_port_change(RemoteUsbHubPort *port, uint16_t status)
{
    bool notify = false;

    if (status & 0x1f) {
        port->wPortChange |= status;
        notify = true;
    }
    return notify;
}

static bool remote_usb_hub_port_set(RemoteUsbHubPort *port, uint16_t status)
{
    if (port->wPortStatus & status) {
        return false;
    }
    port->wPortStatus |= status;
    return remote_usb_hub_port_change(port, status);
}

static bool remote_usb_hub_port_clear(RemoteUsbHubPort *port, uint16_t status)
{
    if (!(port->wPortStatus & status)) {
        return false;
    }
    port->wPortStatus &= ~status;
    return remote_usb_hub_port_change(port, status);
}

static bool remote_usb_hub_port_update(RemoteUsbHubPort *port)
{
    bool notify = false;

    if (port->port.dev && port->port.dev->attached) {
        notify = remote_usb_hub_port_set(port, PORT_STAT_CONNECTION);
        if (port->port.dev->speed == USB_SPEED_LOW) {
            remote_usb_hub_port_set(port, PORT_STAT_LOW_SPEED);
        } else {
            remote_usb_hub_port_clear(port, PORT_STAT_LOW_SPEED);
        }
    }
    return notify;
}

static void remote_usb_hub_port_update_timer(void *opaque)
{
    RemoteUsbHubState *s = opaque;
    bool notify = false;
    int i;

    for (i = 0; i < s->num_ports; i++) {
        notify |= remote_usb_hub_port_update(&s->ports[i]);
    }
    if (notify) {
        usb_wakeup(s->intr, 0);
    }
}

static void remote_usb_hub_attach(USBPort *port1)
{
    RemoteUsbHubState *s = port1->opaque;
    RemoteUsbHubPort *port = &s->ports[port1->index];

    trace_usb_hub_attach(s->dev.addr, port1->index + 1);
    remote_usb_hub_port_update(port);
    usb_wakeup(s->intr, 0);
}

static void remote_usb_hub_detach(USBPort *port1)
{
    RemoteUsbHubState *s = port1->opaque;
    RemoteUsbHubPort *port = &s->ports[port1->index];

    trace_usb_hub_detach(s->dev.addr, port1->index + 1);
    usb_wakeup(s->intr, 0);

    /* Let upstream know the device on this port is gone */
    s->dev.port->ops->child_detach(s->dev.port, port1->dev);

    remote_usb_hub_port_clear(port, PORT_STAT_CONNECTION);
    remote_usb_hub_port_clear(port, PORT_STAT_ENABLE);
    remote_usb_hub_port_clear(port, PORT_STAT_SUSPEND);
    usb_wakeup(s->intr, 0);
}

static void remote_usb_hub_child_detach(USBPort *port1, USBDevice *child)
{
    RemoteUsbHubState *s = port1->opaque;

    /* Pass along upstream */
    s->dev.port->ops->child_detach(s->dev.port, child);
}

static void remote_usb_hub_wakeup(USBPort *port1)
{
    RemoteUsbHubState *s = port1->opaque;
    RemoteUsbHubPort *port = &s->ports[port1->index];

    if (remote_usb_hub_port_clear(port, PORT_STAT_SUSPEND)) {
        usb_wakeup(s->intr, 0);
    }
}

static void remote_usb_hub_complete(USBPort *port, USBPacket *packet)
{
    RemoteUsbHubState *s = port->opaque;

    /*
     * Just pass it along upstream for now.
     *
     * If we ever implement usb 2.0 split transactions this will
     * become a little more complicated ...
     *
     * Can't use usb_packet_complete() here because packet->owner is
     * cleared already, go call the ->complete() callback directly
     * instead.
     */
    s->dev.port->ops->complete(s->dev.port, packet);
}

static USBDevice *remote_usb_hub_find_device(USBDevice *dev, uint8_t addr)
{
    RemoteUsbHubState *s = REMOTE_USB_HUB(dev);
    RemoteUsbHubPort *port;
    USBDevice *downstream;
    int i;

    for (i = 0; i < s->num_ports; i++) {
        port = &s->ports[i];
        if (!(port->wPortStatus & PORT_STAT_ENABLE)) {
            continue;
        }
        downstream = usb_find_device(&port->port, addr);
        if (downstream != NULL) {
            return downstream;
        }
    }
    return NULL;
}

static void remote_usb_hub_handle_reset(USBDevice *dev)
{
    RemoteUsbHubState *s = REMOTE_USB_HUB(dev);
    RemoteUsbHubPort *port;
    int i;

    trace_usb_hub_reset(s->dev.addr);
    for (i = 0; i < s->num_ports; i++) {
        port = s->ports + i;
        port->wPortStatus = 0;
        port->wPortChange = 0;
        remote_usb_hub_port_set(port, PORT_STAT_POWER);
        remote_usb_hub_port_update(port);
    }
}

static const char *remote_hub_feature_name(int feature)
{
    static const char *name[] = {
        [PORT_CONNECTION]    = "connection",
        [PORT_ENABLE]        = "enable",
        [PORT_SUSPEND]       = "suspend",
        [PORT_OVERCURRENT]   = "overcurrent",
        [PORT_RESET]         = "reset",
        [PORT_POWER]         = "power",
        [PORT_LOWSPEED]      = "lowspeed",
        [PORT_HIGHSPEED]     = "highspeed",
        [PORT_C_CONNECTION]  = "change-connection",
        [PORT_C_ENABLE]      = "change-enable",
        [PORT_C_SUSPEND]     = "change-suspend",
        [PORT_C_OVERCURRENT] = "change-overcurrent",
        [PORT_C_RESET]       = "change-reset",
        [PORT_TEST]          = "test",
        [PORT_INDICATOR]     = "indicator",
    };
    if (feature < 0 || feature >= ARRAY_SIZE(name)) {
        return "?";
    }
    return name[feature] ?: "?";
}

static void remote_usb_hub_handle_control(USBDevice *dev, USBPacket *p,
               int request, int value, int index, int length, uint8_t *data)
{
    RemoteUsbHubState *s = (RemoteUsbHubState *)dev;
    int ret;

    trace_usb_hub_control(s->dev.addr, request, value, index, length);

    ret = usb_desc_handle_control(dev, p, request, value, index, length, data);
    if (ret >= 0) {
        return;
    }

    switch(request) {
    case EndpointOutRequest | USB_REQ_CLEAR_FEATURE:
        if (value == 0 && index != 0x81) { /* clear ep halt */
            goto fail;
        }
        break;
        /* usb specific requests */
    case GetHubStatus:
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        p->actual_length = 4;
        break;
    case GetPortStatus:
        {
            unsigned int n = index - 1;
            RemoteUsbHubPort *port;
            if (n >= s->num_ports) {
                goto fail;
            }
            port = &s->ports[n];
            trace_usb_hub_get_port_status(s->dev.addr, index,
                                          port->wPortStatus,
                                          port->wPortChange);
            data[0] = port->wPortStatus;
            data[1] = port->wPortStatus >> 8;
            data[2] = port->wPortChange;
            data[3] = port->wPortChange >> 8;
            p->actual_length = 4;
        }
        break;
    case SetHubFeature:
    case ClearHubFeature:
        if (value != 0 && value != 1) {
            goto fail;
        }
        break;
    case SetPortFeature:
        {
            unsigned int n = index - 1;
            RemoteUsbHubPort *port;
            USBDevice *pdev;

            trace_usb_hub_set_port_feature(s->dev.addr, index,
                                           remote_hub_feature_name(value));

            if (n >= s->num_ports) {
                goto fail;
            }
            port = &s->ports[n];
            pdev = port->port.dev;
            switch(value) {
            case PORT_SUSPEND:
                port->wPortStatus |= PORT_STAT_SUSPEND;
                break;
            case PORT_RESET:
                remote_usb_hub_port_set(port, PORT_STAT_RESET);
                remote_usb_hub_port_clear(port, PORT_STAT_RESET);
                if (pdev && pdev->attached) {
                    usb_device_reset(pdev);
                    remote_usb_hub_port_set(port, PORT_STAT_ENABLE);
                }
                usb_wakeup(s->intr, 0);
                break;
            case PORT_POWER:
                if (s->port_power) {
                    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                    remote_usb_hub_port_set(port, PORT_STAT_POWER);
                    timer_mod(s->port_timer, now + 5000000); /* 5 ms */
                }
                break;
            default:
                goto fail;
            }
        }
        break;
    case ClearPortFeature:
        {
            unsigned int n = index - 1;
            RemoteUsbHubPort *port;

            trace_usb_hub_clear_port_feature(s->dev.addr, index,
                                             remote_hub_feature_name(value));

            if (n >= s->num_ports) {
                goto fail;
            }
            port = &s->ports[n];
            switch(value) {
            case PORT_ENABLE:
                port->wPortStatus &= ~PORT_STAT_ENABLE;
                break;
            case PORT_C_ENABLE:
                port->wPortChange &= ~PORT_STAT_C_ENABLE;
                break;
            case PORT_SUSPEND:
                remote_usb_hub_port_clear(port, PORT_STAT_SUSPEND);
                break;
            case PORT_C_SUSPEND:
                port->wPortChange &= ~PORT_STAT_C_SUSPEND;
                break;
            case PORT_C_CONNECTION:
                port->wPortChange &= ~PORT_STAT_C_CONNECTION;
                break;
            case PORT_C_OVERCURRENT:
                port->wPortChange &= ~PORT_STAT_C_OVERCURRENT;
                break;
            case PORT_C_RESET:
                port->wPortChange &= ~PORT_STAT_C_RESET;
                break;
            case PORT_POWER:
                if (s->port_power) {
                    remote_usb_hub_port_clear(port, PORT_STAT_POWER);
                    remote_usb_hub_port_clear(port, PORT_STAT_CONNECTION);
                    remote_usb_hub_port_clear(port, PORT_STAT_ENABLE);
                    remote_usb_hub_port_clear(port, PORT_STAT_SUSPEND);
                    port->wPortChange = 0;
                }
                break;
            default:
                goto fail;
            }
        }
        break;
    case GetHubDescriptor:
        {
            unsigned int n, limit, var_hub_size = 0;
            memcpy(data, remote_hub_hub_descriptor,
                   sizeof(remote_hub_hub_descriptor));
            data[2] = s->num_ports;

            if (s->port_power) {
                data[3] &= ~0x03;
                data[3] |= 0x01;
            }

            /* fill DeviceRemovable bits */
            limit = DIV_ROUND_UP(s->num_ports + 1, 8) + 7;
            for (n = 7; n < limit; n++) {
                data[n] = 0x00;
                var_hub_size++;
            }

            /* fill PortPwrCtrlMask bits */
            limit = limit + DIV_ROUND_UP(s->num_ports, 8);
            for (;n < limit; n++) {
                data[n] = 0xff;
                var_hub_size++;
            }

            p->actual_length = sizeof(remote_hub_hub_descriptor) + var_hub_size;
            data[0] = p->actual_length;
            break;
        }
    default:
    fail:
        p->status = USB_RET_STALL;
        break;
    }
}

static void remote_usb_hub_handle_data(USBDevice *dev, USBPacket *p)
{
    RemoteUsbHubState *s = (RemoteUsbHubState *)dev;

    switch(p->pid) {
    case USB_TOKEN_IN:
        if (p->ep->nr == 1) {
            RemoteUsbHubPort *port;
            unsigned int status;
            uint8_t buf[4];
            int i, n;
            n = DIV_ROUND_UP(s->num_ports + 1, 8);
            if (p->iov.size == 1) { /* FreeBSD workaround */
                n = 1;
            } else if (n > p->iov.size) {
                p->status = USB_RET_BABBLE;
                return;
            }
            status = 0;
            for (i = 0; i < s->num_ports; i++) {
                port = &s->ports[i];
                if (port->wPortChange)
                    status |= (1 << (i + 1));
            }
            if (status != 0) {
                trace_usb_hub_status_report(s->dev.addr, status);
                for(i = 0; i < n; i++) {
                    buf[i] = status >> (8 * i);
                }
                usb_packet_copy(p, buf, n);
            } else {
                p->status = USB_RET_NAK; /* usb11 11.13.1 */
            }
        } else {
            goto fail;
        }
        break;
    case USB_TOKEN_OUT:
    default:
    fail:
        p->status = USB_RET_STALL;
        break;
    }
}

static void remote_usb_hub_unrealize(USBDevice *dev)
{
    RemoteUsbHubState *s = (RemoteUsbHubState *)dev;
    int i;

    for (i = 0; i < s->num_ports; i++) {
        usb_unregister_port(usb_bus_from_device(dev),
                            &s->ports[i].port);
    }

    timer_free(s->port_timer);
}

static USBPortOps remote_usb_hub_port_ops = {
    .attach = remote_usb_hub_attach,
    .detach = remote_usb_hub_detach,
    .child_detach = remote_usb_hub_child_detach,
    .wakeup = remote_usb_hub_wakeup,
    .complete = remote_usb_hub_complete,
};

static void remote_usb_hub_realize(USBDevice *dev, Error **errp)
{
    RemoteUsbHubState *s = REMOTE_USB_HUB(dev);
    RemoteUsbHubPort *port;
    int i;

    if (s->num_ports < 1 || s->num_ports > MAX_PORTS) {
        error_setg(errp, "num_ports (%d) out of range (1..%d)",
                   s->num_ports, MAX_PORTS);
        return;
    }

    if (dev->port->hubcount == 5) {
        error_setg(errp, "usb hub chain too deep");
        return;
    }

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    s->port_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 remote_usb_hub_port_update_timer, s);
    s->intr = usb_ep_get(dev, USB_TOKEN_IN, 1);
    for (i = 0; i < s->num_ports; i++) {
        port = &s->ports[i];
        usb_register_port(usb_bus_from_device(dev),
                          &port->port, s, i, &remote_usb_hub_port_ops,
                          USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL |
                          USB_SPEED_MASK_HIGH);
        usb_port_location(&port->port, dev->port, i+1);
    }
    remote_usb_hub_handle_reset(dev);
}

static const VMStateDescription vmstate_remote_usb_hub_port = {
    .name = "usb-hub-port",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(wPortStatus, RemoteUsbHubPort),
        VMSTATE_UINT16(wPortChange, RemoteUsbHubPort),
        VMSTATE_END_OF_LIST()
    }
};

static bool remote_usb_hub_port_timer_needed(void *opaque)
{
    RemoteUsbHubState *s = opaque;

    return s->port_power;
}

static const VMStateDescription vmstate_remote_usb_hub_port_timer = {
    .name = "usb-hub/port-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = remote_usb_hub_port_timer_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(port_timer, RemoteUsbHubState),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_remote_usb_hub = {
    .name = "remote-usb-hub",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_USB_DEVICE(dev, RemoteUsbHubState),
        VMSTATE_STRUCT_ARRAY(ports, RemoteUsbHubState, MAX_PORTS, 0,
                             vmstate_remote_usb_hub_port, RemoteUsbHubPort),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_remote_usb_hub_port_timer,
        NULL
    }
};

static const Property remote_usb_hub_properties[] = {
    DEFINE_PROP_UINT32("ports", RemoteUsbHubState, num_ports, 8),
    DEFINE_PROP_BOOL("port-power", RemoteUsbHubState, port_power, false),
};

static void remote_usb_hub_class_initfn(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->realize        = remote_usb_hub_realize;
    uc->product_desc   = "chisel-usb Remote Hub";
    uc->usb_desc       = &remote_hub_desc;
    uc->find_device    = remote_usb_hub_find_device;
    uc->handle_reset   = remote_usb_hub_handle_reset;
    uc->handle_control = remote_usb_hub_handle_control;
    uc->handle_data    = remote_usb_hub_handle_data;
    uc->unrealize      = remote_usb_hub_unrealize;
    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->fw_name = "hub";
    dc->vmsd = &vmstate_remote_usb_hub;
    device_class_set_props(dc, remote_usb_hub_properties);
}

static const TypeInfo remote_hub_info = {
    .name          = TYPE_REMOTE_USB_HUB,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(RemoteUsbHubState),
    .class_init    = remote_usb_hub_class_initfn,
};

static void remote_usb_hub_register_types(void)
{
    type_register_static(&remote_hub_info);
}

type_init(remote_usb_hub_register_types)
