// SPDX-License-Identifier: Apache-2.0
// rpcie_protocol.h — Remote PCIe Port wire protocol
//
// Shared between QEMU's remote-pcie-port device and any endpoint
// simulation server (Verilator, VCS, FPGA proxy, etc.).
//
// The protocol models a single PCIe link at the TLP level.  The QEMU
// side acts as a Root Complex port; the remote side acts as an endpoint
// (which may contain multiple PFs and SR-IOV VFs).
//
// All multi-byte fields are little-endian on the wire.
//
// Copyright 2026 Open-DPU Project

#ifndef RPCIE_PROTOCOL_H
#define RPCIE_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================================================================
//  Constants
// ===================================================================

#define RPCIE_MAGIC            0x52504345u   // "RPCE"
#define RPCIE_PROTOCOL_VERSION 1

// Maximum TLP payload (matches PCIe Max Payload Size of 4096)
#define RPCIE_MAX_PAYLOAD      4096

// Maximum config space (PCIe extended)
#define RPCIE_MAX_CFG_SPACE    4096

// Maximum functions per link (dev[4:0] × fn[2:0] = 256)
#define RPCIE_MAX_FUNCTIONS    256

// Maximum BARs per function
#define RPCIE_MAX_BARS         6

// ===================================================================
//  Frame header — every message on the Unix socket
// ===================================================================
//
//  ┌──────────────┬──────────────┬───────────┬──────────────────────┐
//  │ magic  (4B)  │ length (4B)  │ seq (4B)  │ payload[length]      │
//  └──────────────┴──────────────┴───────────┴──────────────────────┘
//
//  `length` — byte count of payload (excludes the 12-byte frame header).
//  `seq`    — sequence number for request/completion matching.
//             Downstream requests (QEMU → sim) use odd seq numbers.
//             The completion echoes the request's seq.
//             Unsolicited upstream messages (DMA, interrupts, topology)
//             use even seq numbers.
//

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t length;   // payload byte count
    uint32_t seq;      // sequence ID for req/cpl matching
} rpcie_frame_hdr_t;

// ===================================================================
//  PCIe TLP type encodings (matches PCIe Base Spec)
// ===================================================================
//
//  fmt[2:0] << 5 | type[4:0]
//
//  Bit 5 = 1 → 4DW header (64-bit address)
//  Bit 6 = 1 → has data payload
//
//  These values match the Chisel PCIeTlpType constants in
//  chiselpcie/pcie/bundle/PCIeTlp.scala

#define RPCIE_TLP_MRD32      0x00   // Memory Read (32-bit addr)
#define RPCIE_TLP_MRD64      0x20   // Memory Read (64-bit addr)
#define RPCIE_TLP_MRDLK32    0x01   // Memory Read Lock (32-bit)
#define RPCIE_TLP_MRDLK64    0x21   // Memory Read Lock (64-bit)
#define RPCIE_TLP_MWR32      0x40   // Memory Write (32-bit addr)
#define RPCIE_TLP_MWR64      0x60   // Memory Write (64-bit addr)
#define RPCIE_TLP_CPL        0x0A   // Completion without Data
#define RPCIE_TLP_CPLD       0x4A   // Completion with Data
#define RPCIE_TLP_CPLLK      0x0B   // Completion for Locked (no data)
#define RPCIE_TLP_CPLDLK     0x4B   // Completion for Locked (data)
#define RPCIE_TLP_CFGRD0     0x04   // Config Read Type 0
#define RPCIE_TLP_CFGWR0     0x44   // Config Write Type 0
#define RPCIE_TLP_CFGRD1     0x05   // Config Read Type 1
#define RPCIE_TLP_CFGWR1     0x45   // Config Write Type 1
#define RPCIE_TLP_MSG        0x30   // Message Request
#define RPCIE_TLP_MSGD       0x70   // Message Request with Data

// TLP type predicates
#define RPCIE_TLP_IS_64BIT(t)     (((t) & 0x20) != 0)
#define RPCIE_TLP_HAS_DATA(t)     (((t) & 0x40) != 0)
#define RPCIE_TLP_IS_MEM_READ(t)  (((t) == RPCIE_TLP_MRD32) || ((t) == RPCIE_TLP_MRD64))
#define RPCIE_TLP_IS_MEM_WRITE(t) (((t) == RPCIE_TLP_MWR32) || ((t) == RPCIE_TLP_MWR64))
#define RPCIE_TLP_IS_CFG(t)       (((t) & 0x1F) == 0x04 || ((t) & 0x1F) == 0x05)
#define RPCIE_TLP_IS_CPL(t)       (((t) & 0x1F) == 0x0A || ((t) & 0x1F) == 0x0B)
#define RPCIE_TLP_IS_MSG(t)       (((t) & 0x1F) >= 0x10 && ((t) & 0x1F) <= 0x17)

// ===================================================================
//  TLP header structures (on-wire format)
// ===================================================================
//
//  These follow the PCIe Base Spec TLP header layout but are
//  serialised as packed C structs for easy encode/decode.
//
//  The first byte of every TLP payload is the fmt_type field.
//

// --- Common TLP header (first 3 DW / 12 bytes for all TLPs) ---
//
// DW0: fmt_type[7:0] | tc_attr[7:0] | length_dw[15:0]
// DW1: requester_id[15:0] | tag[7:0] | last_be[3:0] | first_be[3:0]
// DW2: depends on TLP type (address, completer_id, etc.)
//
typedef struct __attribute__((packed)) {
    uint8_t  fmt_type;       // PCIe Fmt[2:0] << 5 | Type[4:0]
    uint8_t  tc_attr;        // TC[6:4] | Attr[1:0] (bits 3:2 reserved)
    uint16_t length_dw;      // payload length in DW (0 = 1024 DW)
    uint16_t requester_id;   // Bus[15:8] | Dev[7:3] | Fn[2:0]
    uint8_t  tag;
    uint8_t  byte_enables;   // last_be[7:4] | first_be[3:0]
} rpcie_tlp_hdr_t;

// --- Memory read/write TLP (3DW or 4DW header) ---
typedef struct __attribute__((packed)) {
    rpcie_tlp_hdr_t hdr;
    uint64_t address;        // 32-bit or 64-bit (depends on fmt_type bit 5)
    // Followed by data payload for MWr (length_dw × 4 bytes)
} rpcie_tlp_mem_t;

// --- Config read/write TLP (3DW header, Type 0/1) ---
typedef struct __attribute__((packed)) {
    rpcie_tlp_hdr_t hdr;
    uint16_t completer_id;   // target function: Bus[15:8] | Dev[7:3] | Fn[2:0]
    uint16_t reg_addr;       // register byte offset [11:2] | reserved[1:0]
    // Followed by 4 bytes of write data for CfgWr
} rpcie_tlp_cfg_t;

// --- Completion TLP (3DW header) ---
typedef struct __attribute__((packed)) {
    rpcie_tlp_hdr_t hdr;
    uint16_t completer_id;   // Bus[15:8] | Dev[7:3] | Fn[2:0]
    uint16_t status_bcm_bc;  // status[15:13] | BCM[12] | byte_count[11:0]
    uint16_t requester_id;   // original requester
    uint8_t  tag;            // original tag
    uint8_t  lower_addr;     // lower address (byte offset into first DW)
    // Followed by data payload for CplD (length_dw × 4 bytes)
} rpcie_tlp_cpl_t;

// --- Message TLP ---
typedef struct __attribute__((packed)) {
    rpcie_tlp_hdr_t hdr;
    uint8_t  msg_code;       // message code (INTx, PME, AER, MSI, etc.)
    uint8_t  reserved[3];
    uint64_t msg_addr;       // message address (for MSI/MSI-X)
    // Followed by data payload for MsgD (length_dw × 4 bytes)
} rpcie_tlp_msg_t;

// Completion status codes (in status_bcm_bc field, bits [15:13])
#define RPCIE_CPL_SC   0   // Successful Completion
#define RPCIE_CPL_UR   1   // Unsupported Request
#define RPCIE_CPL_CRS  2   // Configuration Request Retry Status
#define RPCIE_CPL_CA   4   // Completer Abort

// Extract completion status from status_bcm_bc
#define RPCIE_CPL_STATUS(sbc)     (((sbc) >> 13) & 0x7)
#define RPCIE_CPL_BYTE_COUNT(sbc) ((sbc) & 0xFFF)
#define RPCIE_CPL_MAKE_SBC(status, bc) \
    ((uint16_t)(((status) & 0x7) << 13) | ((bc) & 0xFFF))

// ===================================================================
//  Control messages (out-of-band, not real TLPs)
// ===================================================================
//
//  Control messages use fmt_type >= 0xF0 to distinguish them from
//  standard PCIe TLPs.  They are framed identically (rpcie_frame_hdr_t
//  followed by payload).
//

#define RPCIE_CTRL_HANDSHAKE       0xF0
#define RPCIE_CTRL_HANDSHAKE_ACK   0xF1
#define RPCIE_CTRL_LINK_UP         0xF2
#define RPCIE_CTRL_LINK_DOWN       0xF3
#define RPCIE_CTRL_RESET           0xF4
#define RPCIE_CTRL_FN_ADD          0xF5
#define RPCIE_CTRL_FN_REMOVE       0xF6
#define RPCIE_CTRL_SHM_SETUP       0xF7
#define RPCIE_CTRL_SHM_ACK         0xF8
#define RPCIE_CTRL_SRIOV_EVENT     0xF9  // QEMU → sim: VF Enable changed

// Is this a control message (not a real TLP)?
#define RPCIE_IS_CTRL(ft)    ((ft) >= 0xF0)

// --- Handshake (first message after socket connect) ---
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_HANDSHAKE or HANDSHAKE_ACK
    uint8_t  version;        // RPCIE_PROTOCOL_VERSION
    uint16_t reserved;
    uint32_t capabilities;   // bitmask of optional features
} rpcie_ctrl_handshake_t;

// Capability flags for handshake negotiation
#define RPCIE_CAP_SRIOV     (1u << 0)   // SR-IOV VF add/remove flow
#define RPCIE_CAP_AER       (1u << 1)   // AER error reporting messages
#define RPCIE_CAP_HOTPLUG   (1u << 2)   // Hot-add / hot-remove of PFs
#define RPCIE_CAP_SHM       (1u << 3)   // Shared memory DMA (memfd)
#define RPCIE_CAP_PASID     (1u << 4)   // PASID support
#define RPCIE_CAP_ATS       (1u << 5)   // ATS (Address Translation Services)
#define RPCIE_CAP_FLR       (1u << 6)   // Function Level Reset

// --- Reset message (QEMU → sim, or sim → QEMU for FLR ack) ---
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_RESET
    uint8_t  reset_type;     // see RPCIE_RESET_* below
    uint16_t target_bdf;     // 0xFFFF = all functions on this link
} rpcie_ctrl_reset_t;

#define RPCIE_RESET_COLD     0
#define RPCIE_RESET_WARM     1
#define RPCIE_RESET_FLR      2   // Function-Level Reset (targets one BDF)
#define RPCIE_RESET_HOT      3   // Hot reset (targets all functions)

// --- Function add (sim → QEMU): a new PCI function appeared ---
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_FN_ADD
    uint8_t  fn_type;        // 0 = PF, 1 = VF
    uint16_t bdf;            // function's BDF on the secondary bus
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t subsys_vendor_id;
    uint16_t subsys_id;
    uint8_t  class_code[3];  // prog_if[0], subclass[1], base_class[2]
    uint8_t  revision;
    uint16_t pf_bdf;         // parent PF's BDF (only meaningful if fn_type=1)
    uint8_t  num_bars;
    uint8_t  has_msix;       // 1 if MSI-X info follows bar descriptors
    uint8_t  has_msi;        // 1 if MSI info follows (after optional MSI-X)
    uint8_t  flr_capable;    // 1 if function supports FLR
    uint8_t  has_sriov;      // 1 if SR-IOV info follows (PFs only)
    uint8_t  num_custom_caps; // number of rpcie_custom_cap_desc_t following
    uint8_t  num_custom_ext_caps; // number of rpcie_custom_ext_cap_desc_t following
    // Followed by:
    //   num_bars  × rpcie_bar_desc_t
    //   if has_msix:  1 × rpcie_msix_info_t
    //   if has_msi:   1 × rpcie_msi_info_t
    //   if has_sriov: 1 × rpcie_sriov_info_t + num_vf_bars × rpcie_bar_desc_t
    //   num_custom_caps × rpcie_custom_cap_desc_t (each followed by data_len bytes)
    //   num_custom_ext_caps × rpcie_custom_ext_cap_desc_t (each followed by data_len bytes)
} rpcie_ctrl_fn_add_t;

// Custom PCI capability descriptor (appended to FN_ADD)
// Followed immediately by data_len bytes of capability body
// (excluding the 2-byte PCI header: cap_id and next pointer).
typedef struct __attribute__((packed)) {
    uint8_t  cap_id;         // PCI capability ID (e.g. 0x09 for vendor-specific)
    uint8_t  reserved;
    uint16_t data_len;       // length of body data in bytes
} rpcie_custom_cap_desc_t;

// Custom PCIe extended capability descriptor (appended to FN_ADD)
// Followed immediately by data_len bytes of extended capability body
// (excluding the 4-byte ext cap header: cap_id, version, next pointer).
typedef struct __attribute__((packed)) {
    uint16_t cap_id;         // Extended capability ID (e.g. 0x0001 for AER)
    uint8_t  cap_version;    // Capability version (4-bit)
    uint8_t  reserved;
    uint16_t data_len;       // length of body data in bytes
    uint16_t reserved2;
} rpcie_custom_ext_cap_desc_t;

// BAR descriptor (appended to FN_ADD)
typedef struct __attribute__((packed)) {
    uint8_t  bar_index;      // 0-5
    uint8_t  type;           // 0=disabled, 1=MEM32, 2=MEM64, 3=IO
    uint8_t  prefetchable;   // 0 or 1
    uint8_t  reserved;
    uint64_t size;           // BAR region size in bytes (power-of-2)
} rpcie_bar_desc_t;

#define RPCIE_BAR_DISABLED   0
#define RPCIE_BAR_MEM32      1
#define RPCIE_BAR_MEM64      2
#define RPCIE_BAR_IO         3

// MSI-X capability info (appended to FN_ADD if has_msix=1)
typedef struct __attribute__((packed)) {
    uint16_t table_size;     // number of MSI-X vectors (1-2048)
    uint8_t  table_bar;      // BAR index containing MSI-X table
    uint8_t  pba_bar;        // BAR index containing PBA
    uint32_t table_offset;   // byte offset of table within BAR
    uint32_t pba_offset;     // byte offset of PBA within BAR
} rpcie_msix_info_t;

// MSI capability info (appended to FN_ADD if has_msi=1)
typedef struct __attribute__((packed)) {
    uint8_t  num_vectors;    // number of MSI vectors (1, 2, 4, 8, 16, 32)
    uint8_t  is_64bit;       // 1 if 64-bit addressing capable
    uint8_t  per_vector_mask; // 1 if per-vector masking supported
    uint8_t  reserved;
} rpcie_msi_info_t;

// SR-IOV capability info (appended to FN_ADD if has_sriov=1, PFs only)
// Followed by num_vf_bars × rpcie_bar_desc_t for VF BAR sizes/types.
typedef struct __attribute__((packed)) {
    uint16_t total_vfs;      // maximum VFs supported
    uint16_t vf_device_id;   // PCI device ID for VFs
    uint16_t vf_offset;      // first VF offset (BDF routing)
    uint16_t vf_stride;      // VF stride (BDF routing)
    uint16_t supported_page_sizes; // supported page sizes bitmap
    uint8_t  num_vf_bars;    // number of VF BAR descriptors following
    uint8_t  reserved;
    // Followed by: num_vf_bars × rpcie_bar_desc_t
} rpcie_sriov_info_t;

// --- SR-IOV event (QEMU → sim): guest changed VF Enable ---
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_SRIOV_EVENT
    uint8_t  pf_index;       // which PF (0 for single-PF)
    uint8_t  vf_enable;      // 1 = VFs enabled, 0 = VFs disabled
    uint8_t  reserved;
    uint16_t num_vfs;        // number of VFs requested by guest
    uint16_t reserved2;
} rpcie_ctrl_sriov_event_t;

// --- Function remove (sim → QEMU): a function was removed ---
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_FN_REMOVE
    uint8_t  reserved;
    uint16_t bdf;
} rpcie_ctrl_fn_remove_t;

// --- Link up/down (sim → QEMU or QEMU → sim) ---
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_LINK_UP or RPCIE_CTRL_LINK_DOWN
    uint8_t  reserved[3];
} rpcie_ctrl_link_t;

// --- Shared memory setup (QEMU → sim) ---
//
// After handshake, QEMU sends this message with the guest RAM memfd
// file descriptor passed via sendmsg() SCM_RIGHTS.  The simulation
// mmaps it for zero-copy DMA.
//
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_SHM_SETUP
    uint8_t  reserved;
    uint16_t region_id;      // 0 = guest RAM, may extend for other regions
    uint64_t offset;         // offset within the fd
    uint64_t size;           // size of the mapping in bytes
    uint64_t below_4g_size;  // RAM below PCI hole (x86); 0 = no hole
} rpcie_ctrl_shm_setup_t;

// --- Shared memory acknowledgement (sim → QEMU) ---
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;       // RPCIE_CTRL_SHM_ACK
    uint8_t  status;         // 0 = success, non-zero = error
    uint16_t region_id;
} rpcie_ctrl_shm_ack_t;

// ===================================================================
//  BDF helpers
// ===================================================================

#define RPCIE_BDF(bus, dev, fn) \
    ((uint16_t)(((bus) & 0xFF) << 8) | (((dev) & 0x1F) << 3) | ((fn) & 0x07))

#define RPCIE_BDF_BUS(bdf)  (((bdf) >> 8) & 0xFF)
#define RPCIE_BDF_DEV(bdf)  (((bdf) >> 3) & 0x1F)
#define RPCIE_BDF_FN(bdf)   ((bdf) & 0x07)

// Wildcard BDF — used in RESET to mean "all functions"
#define RPCIE_BDF_ALL        0xFFFF

// ===================================================================
//  MSI/MSI-X message routing
// ===================================================================
//
//  MSI-X and MSI interrupts are sent as MsgD TLPs upstream (sim → QEMU).
//  The msg_code field in rpcie_tlp_msg_t distinguishes them:
//

#define RPCIE_MSG_MSIX_NOTIFY     0x80   // MSI-X interrupt (cosim-specific code)
#define RPCIE_MSG_MSI_NOTIFY      0x81   // MSI interrupt (cosim-specific code)
#define RPCIE_MSG_INTX_ASSERT     0x20   // INTx assert
#define RPCIE_MSG_INTX_DEASSERT   0x24   // INTx deassert
#define RPCIE_MSG_PME             0x18   // Power Management Event
#define RPCIE_MSG_AER_ERR_COR     0x30   // AER Correctable Error
#define RPCIE_MSG_AER_ERR_NONFATAL 0x31  // AER Non-Fatal Error
#define RPCIE_MSG_AER_ERR_FATAL   0x33   // AER Fatal Error

// Helper: build a MSI-X notification MsgD TLP
// The requester_id carries the function's BDF, msg_addr = MSI-X address,
// first DW of data = MSI-X data (vector info).

// ===================================================================
//  Inline helpers for building/parsing TLPs
// ===================================================================

static inline uint16_t rpcie_make_bdf(uint8_t bus, uint8_t dev, uint8_t fn) {
    return RPCIE_BDF(bus, dev, fn);
}

static inline uint8_t rpcie_bdf_bus(uint16_t bdf) { return RPCIE_BDF_BUS(bdf); }
static inline uint8_t rpcie_bdf_dev(uint16_t bdf) { return RPCIE_BDF_DEV(bdf); }
static inline uint8_t rpcie_bdf_fn(uint16_t bdf)  { return RPCIE_BDF_FN(bdf);  }

// Compute byte enables from offset and size
// e.g. offset=1, size=2 → first_be=0b0110, last_be=0b0000
static inline uint8_t rpcie_compute_first_be(uint32_t offset, int size) {
    int lane = offset & 0x3;
    uint8_t mask = (uint8_t)((1 << size) - 1) << lane;
    return mask & 0xF;
}

// Build a CfgRd0 TLP header
static inline void rpcie_build_cfgrd0(rpcie_tlp_cfg_t *tlp,
                                       uint16_t req_id, uint8_t tag,
                                       uint16_t target_bdf,
                                       uint16_t reg_offset, int size) {
    tlp->hdr.fmt_type = RPCIE_TLP_CFGRD0;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = 1;  // config reads always return 1 DW
    tlp->hdr.requester_id = req_id;
    tlp->hdr.tag = tag;
    tlp->hdr.byte_enables = rpcie_compute_first_be(reg_offset, size);
    tlp->completer_id = target_bdf;
    tlp->reg_addr = reg_offset & 0xFFFC;  // DW-aligned
}

// Build a CfgWr0 TLP header (data follows immediately after)
static inline void rpcie_build_cfgwr0(rpcie_tlp_cfg_t *tlp,
                                       uint16_t req_id, uint8_t tag,
                                       uint16_t target_bdf,
                                       uint16_t reg_offset, int size) {
    tlp->hdr.fmt_type = RPCIE_TLP_CFGWR0;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = 1;  // config writes always carry 1 DW
    tlp->hdr.requester_id = req_id;
    tlp->hdr.tag = tag;
    tlp->hdr.byte_enables = rpcie_compute_first_be(reg_offset, size);
    tlp->completer_id = target_bdf;
    tlp->reg_addr = reg_offset & 0xFFFC;
}

// Build a Completion with Data (CplD) TLP header
static inline void rpcie_build_cpld(rpcie_tlp_cpl_t *tlp,
                                     uint16_t completer_bdf,
                                     uint16_t requester_bdf,
                                     uint8_t tag,
                                     uint8_t status,
                                     uint16_t byte_count,
                                     uint8_t lower_addr,
                                     uint16_t length_dw) {
    tlp->hdr.fmt_type = RPCIE_TLP_CPLD;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = length_dw;
    tlp->hdr.requester_id = completer_bdf;  // note: in cpl, hdr.req_id = completer
    tlp->hdr.tag = 0;
    tlp->hdr.byte_enables = 0;
    tlp->completer_id = completer_bdf;
    tlp->status_bcm_bc = RPCIE_CPL_MAKE_SBC(status, byte_count);
    tlp->requester_id = requester_bdf;
    tlp->tag = tag;
    tlp->lower_addr = lower_addr;
}

// Build a Completion without Data (Cpl) TLP header
static inline void rpcie_build_cpl(rpcie_tlp_cpl_t *tlp,
                                    uint16_t completer_bdf,
                                    uint16_t requester_bdf,
                                    uint8_t tag,
                                    uint8_t status) {
    tlp->hdr.fmt_type = RPCIE_TLP_CPL;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = 0;
    tlp->hdr.requester_id = completer_bdf;
    tlp->hdr.tag = 0;
    tlp->hdr.byte_enables = 0;
    tlp->completer_id = completer_bdf;
    tlp->status_bcm_bc = RPCIE_CPL_MAKE_SBC(status, 0);
    tlp->requester_id = requester_bdf;
    tlp->tag = tag;
    tlp->lower_addr = 0;
}

// Build a Memory Write TLP (32-bit address)
static inline void rpcie_build_mwr32(rpcie_tlp_mem_t *tlp,
                                      uint16_t req_id, uint8_t tag,
                                      uint32_t addr, uint16_t length_dw,
                                      uint8_t first_be, uint8_t last_be) {
    tlp->hdr.fmt_type = RPCIE_TLP_MWR32;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = length_dw;
    tlp->hdr.requester_id = req_id;
    tlp->hdr.tag = tag;
    tlp->hdr.byte_enables = (last_be << 4) | (first_be & 0xF);
    tlp->address = addr;  // only lower 32 bits used
}

// Build a Memory Write TLP (64-bit address)
static inline void rpcie_build_mwr64(rpcie_tlp_mem_t *tlp,
                                      uint16_t req_id, uint8_t tag,
                                      uint64_t addr, uint16_t length_dw,
                                      uint8_t first_be, uint8_t last_be) {
    tlp->hdr.fmt_type = RPCIE_TLP_MWR64;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = length_dw;
    tlp->hdr.requester_id = req_id;
    tlp->hdr.tag = tag;
    tlp->hdr.byte_enables = (last_be << 4) | (first_be & 0xF);
    tlp->address = addr;
}

// Build a Memory Read TLP (64-bit address)
static inline void rpcie_build_mrd64(rpcie_tlp_mem_t *tlp,
                                      uint16_t req_id, uint8_t tag,
                                      uint64_t addr, uint16_t length_dw,
                                      uint8_t first_be, uint8_t last_be) {
    tlp->hdr.fmt_type = RPCIE_TLP_MRD64;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = length_dw;
    tlp->hdr.requester_id = req_id;
    tlp->hdr.tag = tag;
    tlp->hdr.byte_enables = (last_be << 4) | (first_be & 0xF);
    tlp->address = addr;
}

// Build a Message with Data TLP (for MSI-X / MSI notifications)
static inline void rpcie_build_msix_msgd(rpcie_tlp_msg_t *tlp,
                                          uint16_t req_id,
                                          uint16_t vector,
                                          uint64_t msi_addr,
                                          uint32_t msi_data) {
    tlp->hdr.fmt_type = RPCIE_TLP_MSGD;
    tlp->hdr.tc_attr = 0;
    tlp->hdr.length_dw = 1;   // MSI-X data is 1 DW
    tlp->hdr.requester_id = req_id;
    tlp->hdr.tag = 0;
    tlp->hdr.byte_enables = 0;
    tlp->msg_code = RPCIE_MSG_MSIX_NOTIFY;
    tlp->msg_addr = msi_addr;
    // msi_data and vector are packed in the data payload
    // (appended by the caller after this struct)
    (void)vector;
    (void)msi_data;
}

#ifdef __cplusplus
}
#endif

#endif // RPCIE_PROTOCOL_H
