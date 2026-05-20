// SPDX-License-Identifier: Apache-2.0
// rusb_protocol.h - Remote USB Port wire protocol (v2, transaction-level)
//
// Shared between QEMU's remote-usb-port device and a chisel-usb
// simulation server (Verilator + ChiselSim).
//
// Semantics:
//   QEMU's USB core dispatches per-transaction callbacks:
//     - handle_control(): one whole control transfer (setup + data + status)
//     - handle_data(USB_TOKEN_OUT/IN): one bulk/intr packet
//   Each callback is wrapped into a single transaction message and
//   exchanged with the sim. The sim's UsbHostBfm executes the
//   wire-level USB packets (token + data + handshake with CRCs) and
//   returns a TXN_RESULT.
//
// Connection:
//   QEMU connects(SOCK_STREAM, AF_UNIX) to a listening socket created
//   by the simulator. On connect, QEMU sends HELLO; the simulator
//   processes incoming messages serially.
//
// All multi-byte fields are little-endian.

#ifndef RUSB_PROTOCOL_H
#define RUSB_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RUSB_MAGIC            0x52555342u   // "RUSB"
#define RUSB_PROTOCOL_VERSION 2
#define RUSB_MAX_PACKET       8192

// ------- frame header (always 12 bytes) -------
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t length;    // payload bytes following this header
    uint32_t seq;
} rusb_frame_hdr_t;

// ------- message types -------
#define RUSB_MSG_HELLO          0x00   // QEMU -> sim on connect
#define RUSB_MSG_BUS_RESET      0x03   // QEMU -> sim
#define RUSB_MSG_CONTROL_TXN    0x40   // QEMU -> sim: full control transfer
#define RUSB_MSG_OUT_TXN        0x41   // QEMU -> sim: bulk/intr OUT packet
#define RUSB_MSG_IN_TXN         0x42   // QEMU -> sim: bulk/intr IN poll
#define RUSB_MSG_TXN_RESULT     0x50   // sim -> QEMU: result of any *_TXN

// TXN_RESULT.status values
#define RUSB_STATUS_OK          0   // ACK / success
#define RUSB_STATUS_NAK         1   // device NAK
#define RUSB_STATUS_STALL       2   // device STALL
#define RUSB_STATUS_TIMEOUT     3   // no response

// ------- HELLO -------
typedef struct __attribute__((packed)) {
    uint8_t msg_type;       // 0x00
    uint8_t version;
    uint8_t reserved[2];
} rusb_hello_t;

// ------- BUS_RESET -------
typedef struct __attribute__((packed)) {
    uint8_t msg_type;       // 0x03
    uint8_t reserved[3];
} rusb_bus_reset_t;

// ------- CONTROL_TXN (16-byte fixed header + optional OUT data) -------
//
//  When direction == 0 (OUT):  payload bytes after the fixed header
//                              contain the host-to-device data stage
//                              (length = data_len).
//  When direction == 1 (IN):   data_len is the host's expected
//                              wLength; sim collects up to that many
//                              bytes into the TXN_RESULT.
//
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // 0x40
    uint8_t  addr;           // device USB address (0 before SET_ADDRESS)
    uint8_t  max_packet_size;// ep0 wMaxPacketSize (8/16/32/64)
    uint8_t  direction;      // 0 = OUT, 1 = IN
    uint8_t  setup[8];       // raw 8-byte setup packet
    uint16_t data_len;       // OUT: bytes following; IN: expected wLength
    uint16_t reserved;
    // uint8_t data[data_len]  -- only when direction == 0
} rusb_control_txn_t;

// ------- OUT_TXN (8-byte fixed header + data) -------
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // 0x41
    uint8_t  addr;
    uint8_t  ep;
    uint8_t  data_pid;       // 0 = DATA0, 1 = DATA1
    uint16_t length;
    uint16_t reserved;
    // uint8_t data[length]
} rusb_out_txn_t;

// ------- IN_TXN (8-byte fixed header) -------
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // 0x42
    uint8_t  addr;
    uint8_t  ep;
    uint8_t  reserved;
    uint16_t max_length;
    uint16_t reserved2;
} rusb_in_txn_t;

// ------- TXN_RESULT (8-byte fixed header + optional IN data) -------
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // 0x50
    uint8_t  status;         // RUSB_STATUS_*
    uint8_t  data_pid;       // 0/1 for IN replies, 0 otherwise
    uint8_t  reserved;
    uint16_t length;         // number of payload data bytes following
    uint16_t reserved2;
    // uint8_t data[length]
} rusb_txn_result_t;

#ifdef __cplusplus
}
#endif

#endif /* RUSB_PROTOCOL_H */
