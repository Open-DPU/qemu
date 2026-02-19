/*
 * remote-pcie-port.h — Remote PCIe endpoint (TLP-over-socket bridge)
 *
 * A single PCIe endpoint device whose config space, BARs, interrupts,
 * and DMA are all owned by an external simulation connected via Unix
 * socket.  QEMU acts as a thin TLP pipe — it forwards guest accesses
 * to the simulation and executes DMA/interrupts on the simulation's
 * behalf.  There is intentionally very little local state.
 *
 * The simulation may represent any number of PFs, VFs, or SR-IOV
 * configurations — QEMU doesn't care.  It just forwards TLPs and
 * the simulation handles the rest.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2025 Open-DPU Project
 */

#ifndef HW_REMOTE_PCIE_PORT_H
#define HW_REMOTE_PCIE_PORT_H

#include "hw/pci/pci_device.h"
#include "hw/remote/rpcie-protocol.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "qom/object.h"

#define TYPE_REMOTE_PCIE_PORT "remote-pcie-port"
OBJECT_DECLARE_SIMPLE_TYPE(RemotePciePort, REMOTE_PCIE_PORT)

#define RPCIE_MAX_BARS          6
#define RPCIE_CPL_RING_SIZE     256   /* must be power of 2 */

/* Completion slot for blocking request → response matching. */
typedef struct RpcieCompletion {
    uint32_t seq;
    bool     valid;
    uint8_t  status;
    uint32_t data;
} RpcieCompletion;

struct RemotePciePort {
    /*< private >*/
    PCIDevice parent_obj;
    /*< public >*/

    /* Connection */
    char        *socket_path;
    bool         server;
    int          listen_fd;
    int          conn_fd;

    /* Protocol */
    uint32_t     remote_caps;
    uint32_t     next_seq;        /* atomic; odd numbers, +=2 */

    /* RX thread */
    QemuThread   rx_thread;
    bool         rx_running;

    /* Completion ring */
    QemuMutex    cpl_mutex;
    QemuCond     cpl_cond;
    RpcieCompletion cpl_ring[RPCIE_CPL_RING_SIZE];

    /* Send serialisation */
    QemuMutex    send_mutex;

    /* BARs (configured from simulation during realize) */
    MemoryRegion bar_mr[RPCIE_MAX_BARS];
    uint64_t     bar_size[RPCIE_MAX_BARS];
    uint8_t      bar_type[RPCIE_MAX_BARS];
    uint8_t      num_bars;

    /* Remote BDF (from initial FN_ADD, used in TLP headers) */
    uint16_t     remote_bdf;
};

/* Send a framed message (acquires send_mutex). */
int rpcie_send_msg(RemotePciePort *rp, uint32_t seq,
                   const void *payload, uint32_t len);

/* Send and block for completion.  Returns 0 on success, -1 on timeout. */
int rpcie_send_and_wait(RemotePciePort *rp, uint32_t seq,
                        const void *payload, uint32_t len,
                        uint32_t *cpl_data, uint8_t *cpl_status,
                        int timeout_ms);

/* Allocate the next downstream sequence number (thread-safe). */
uint32_t rpcie_alloc_seq(RemotePciePort *rp);

#endif /* HW_REMOTE_PCIE_PORT_H */
