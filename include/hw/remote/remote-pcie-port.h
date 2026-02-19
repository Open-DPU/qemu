/*
 * remote-pcie-port.h — Remote PCIe Port root port + proxy device
 *
 * A PCIe Root Port whose secondary bus is connected to an external
 * simulation (Verilator, VCS, FPGA) via a Unix socket.  Raw PCIe TLPs
 * are exchanged over the socket using the rpcie wire protocol.
 *
 * Proxy PCI devices appear dynamically on the secondary bus as the
 * simulation sends FN_ADD control messages.  All PFs and SR-IOV VFs
 * are multiplexed on a single socket (one socket = one PCIe link).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2025 Open-DPU Project
 */

#ifndef HW_REMOTE_PCIE_PORT_H
#define HW_REMOTE_PCIE_PORT_H

#include "hw/pci/pci_device.h"
#include "hw/pci/pcie_port.h"
#include "hw/remote/rpcie-protocol.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "qom/object.h"

/* ------------------------------------------------------------------ */
/*  Root Port                                                         */
/* ------------------------------------------------------------------ */

#define TYPE_REMOTE_PCIE_PORT "remote-pcie-port"
OBJECT_DECLARE_SIMPLE_TYPE(RemotePciePort, REMOTE_PCIE_PORT)

#define RPCIE_AER_OFFSET        0x100
#define RPCIE_MSIX_NR_VECTOR    1
#define RPCIE_MAX_PROXIES       256     /* devfn space */

/*
 * Completion slot for blocking request/response matching.
 *
 * Downstream requests (QEMU → sim) use odd sequence numbers.
 * The completion echoes the request's seq.  We map seq→slot via
 * (seq / 2) % ring_size.  With a 256-slot ring and sequences
 * incrementing by 2, collisions only occur with 256 outstanding
 * requests—well beyond real-world use.
 */
typedef struct RpcieCompletion {
    uint32_t seq;
    bool     valid;
    uint8_t  status;          /* PCIe completion status */
    uint32_t data;            /* first DW of completion data */
    uint8_t  extra[RPCIE_MAX_PAYLOAD];
    uint16_t extra_len;
} RpcieCompletion;

#define RPCIE_CPL_RING_SIZE  256  /* must be power of 2 */

struct RemotePciePort {
    /*< private >*/
    PCIESlot parent_obj;
    /*< public >*/

    /* Connection */
    char        *socket_path;
    bool         server;       /* listen mode (default) vs connect */
    int          listen_fd;
    int          conn_fd;

    /* Protocol */
    uint32_t     remote_caps;  /* negotiated capabilities */
    uint32_t     next_seq;     /* atomic; odd numbers for downstream (+=2) */

    /* RX thread */
    QemuThread   rx_thread;
    bool         rx_running;

    /* Completion ring (downstream request → completion matching) */
    QemuMutex    cpl_mutex;
    QemuCond     cpl_cond;
    RpcieCompletion cpl_ring[RPCIE_CPL_RING_SIZE];

    /* Send serialisation */
    QemuMutex    send_mutex;

    /* Proxy device array (indexed by devfn) */
    struct RemotePcieProxy *proxies[RPCIE_MAX_PROXIES];

    /* Shared memory DMA (optional) */
    int          shm_fd;
    void        *shm_ptr;
    uint64_t     shm_size;
    uint64_t     shm_offset;
};

/* ------------------------------------------------------------------ */
/*  Proxy PCI Device (dynamically created on FN_ADD)                  */
/* ------------------------------------------------------------------ */

#define TYPE_REMOTE_PCIE_PROXY "remote-pcie-proxy"
OBJECT_DECLARE_SIMPLE_TYPE(RemotePcieProxy, REMOTE_PCIE_PROXY)

#define RPCIE_PROXY_MAX_BARS    6

struct RemotePcieProxy {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    RemotePciePort *rp;        /* back-pointer to root port */
    uint16_t        remote_bdf;
    bool            is_vf;
    uint16_t        pf_bdf;

    /* PCI identity (cached from FN_ADD for local config space) */
    uint16_t        vendor_id;
    uint16_t        device_id;
    uint16_t        subsys_vendor_id;
    uint16_t        subsys_id;
    uint8_t         class_code[3];  /* prog_if, subclass, base_class */
    uint8_t         revision;

    /* BARs */
    MemoryRegion    bar_mr[RPCIE_PROXY_MAX_BARS];
    uint64_t        bar_size[RPCIE_PROXY_MAX_BARS];
    uint8_t         bar_type[RPCIE_PROXY_MAX_BARS]; /* RPCIE_BAR_* */
    uint8_t         bar_prefetchable[RPCIE_PROXY_MAX_BARS];
    uint8_t         num_bars;

    /* MSI-X (local shadow table for interrupt injection) */
    bool            has_msix;
    uint16_t        msix_vectors;
    uint8_t         msix_table_bar;
    uint32_t        msix_table_offset;
    uint8_t         msix_pba_bar;
    uint32_t        msix_pba_offset;

    /* MSI */
    bool            has_msi;
    uint8_t         msi_vectors;
    bool            msi_64bit;
    bool            msi_per_vector_mask;

    /* FLR */
    bool            flr_capable;

    /* INTx state (for edge detection) */
    int             intx_level;
};

/* ------------------------------------------------------------------ */
/*  Public helpers (used between root port and proxy)                  */
/* ------------------------------------------------------------------ */

/* Send a framed message (acquires send_mutex). */
int rpcie_send_msg(RemotePciePort *rp, uint32_t seq,
                   const void *payload, uint32_t len);

/* Send a downstream TLP and block for its completion.
 * Returns 0 on success, -1 on timeout/error.
 * On success, *cpl_data receives the first DW and *cpl_status the status. */
int rpcie_send_and_wait(RemotePciePort *rp, uint32_t seq,
                        const void *payload, uint32_t len,
                        uint32_t *cpl_data, uint8_t *cpl_status,
                        int timeout_ms);

/* Allocate the next downstream sequence number (odd, thread-safe). */
uint32_t rpcie_alloc_seq(RemotePciePort *rp);

#endif /* HW_REMOTE_PCIE_PORT_H */
