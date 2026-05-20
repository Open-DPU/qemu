// SPDX-License-Identifier: Apache-2.0
// rusb_protocol.h — Remote USB Port wire protocol
//
// Shared between QEMU's remote-usb-port device and any endpoint
// simulation server (Verilator, VCS, FPGA proxy, etc.).
//
// The protocol models a single USB device link at the packet level.
// The QEMU side acts as a host controller (HC); the remote side acts
// as a USB device (which may expose multiple interfaces / endpoints).
//
// Two physical layers are supported:
//  - UTMI+/ULPI byte stream  (USB 2.0 LS/FS/HS)
//  - PIPE 32-bit symbols      (USB 3.x SS/SS+)
//
// At the protocol level both reduce to "framed bytes between host and
// device" plus a small number of side-band signals (reset, suspend,
// resume, attach, detach). The simulation server tells QEMU which
// physical layer it expects via the ATTACH message.
//
// All multi-byte fields are little-endian on the wire.
//
// Copyright 2026 Open-DPU Project

#ifndef RUSB_PROTOCOL_H
#define RUSB_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RUSB_MAGIC            0x52555342u   // "RUSB"
#define RUSB_PROTOCOL_VERSION 1

/** Maximum USB packet size accepted in a single frame. USB 2.0 caps at
 *  1024 (iso/intr HS) + PID + CRC; USB 3.x bursts can carry larger
 *  data-packet payloads (1024 bytes × burst). We pick 8192 as a
 *  generous upper bound. */
#define RUSB_MAX_PACKET 8192

/* ====================================================================
 *  Frame header — every message on the Unix socket
 * ==================================================================== */
//
//  ┌──────────────┬──────────────┬───────────┬──────────────────────┐
//  │ magic (4B)   │ length (4B)  │ seq (4B)  │ payload[length]      │
//  └──────────────┴──────────────┴───────────┴──────────────────────┘
//
//  `length` — byte count of payload (excludes the 12-byte frame header).
//  `seq`    — sequence number for request/completion matching.
//             Downstream messages (QEMU → sim) use odd seq numbers.
//             Upstream messages (sim → QEMU) use even seq numbers.
//
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t length;
    uint32_t seq;
} rusb_frame_hdr_t;

/* ====================================================================
 *  Message types
 * ==================================================================== */

#define RUSB_MSG_HELLO         0x00   /* version handshake (bidir)       */
#define RUSB_MSG_ATTACH        0x01   /* sim declares phy + speed        */
#define RUSB_MSG_DETACH        0x02   /* device disconnected             */
#define RUSB_MSG_BUS_RESET     0x03   /* host issues bus reset           */
#define RUSB_MSG_SUSPEND       0x04   /* host suspended bus              */
#define RUSB_MSG_RESUME        0x05   /* host resumed bus                */
#define RUSB_MSG_REMOTE_WAKEUP 0x06   /* device-initiated wakeup         */
#define RUSB_MSG_PACKET_OUT    0x10   /* host → device packet bytes      */
#define RUSB_MSG_PACKET_IN     0x11   /* device → host packet bytes      */
#define RUSB_MSG_LINE_STATE    0x20   /* UTMI lineState[1:0]             */
#define RUSB_MSG_LTSSM_STATE   0x21   /* USB 3.x LTSSM state             */
#define RUSB_MSG_ERROR         0xFF   /* protocol or simulation error    */

/* ====================================================================
 *  Physical-layer enum (carried in ATTACH)
 * ==================================================================== */

#define RUSB_PHY_UTMI   0x00
#define RUSB_PHY_ULPI   0x01
#define RUSB_PHY_PIPE   0x02

/* ====================================================================
 *  Speed enum (carried in ATTACH / SUSPEND / RESUME)
 * ==================================================================== */

#define RUSB_SPEED_LOW       0x00
#define RUSB_SPEED_FULL      0x01
#define RUSB_SPEED_HIGH      0x02
#define RUSB_SPEED_SUPER     0x03
#define RUSB_SPEED_SUPER_PLUS 0x04

/* ====================================================================
 *  Payload structs
 * ==================================================================== */

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     /* RUSB_MSG_HELLO */
    uint8_t  version;
    uint8_t  reserved[2];
} rusb_hello_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     /* RUSB_MSG_ATTACH */
    uint8_t  phy;          /* RUSB_PHY_* */
    uint8_t  speed;        /* RUSB_SPEED_* */
    uint8_t  reserved;
} rusb_attach_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     /* RUSB_MSG_PACKET_OUT or RUSB_MSG_PACKET_IN */
    uint8_t  reserved[3];
    uint32_t length;       /* number of packet bytes that follow */
    /* uint8_t bytes[length] */
} rusb_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     /* RUSB_MSG_LINE_STATE */
    uint8_t  line_state;   /* 0=SE0, 1=J, 2=K, 3=SE1 */
    uint8_t  reserved[2];
} rusb_line_state_t;

typedef struct __attribute__((packed)) {
    uint8_t  msg_type;     /* RUSB_MSG_ERROR */
    uint8_t  code;         /* implementation-defined */
    uint16_t reserved;
    /* uint8_t message[]   — UTF-8 description (not NUL-terminated) */
} rusb_error_t;

#ifdef __cplusplus
}
#endif

#endif /* RUSB_PROTOCOL_H */
