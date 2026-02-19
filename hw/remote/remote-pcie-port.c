/*
 * remote-pcie-port.c — Remote PCIe Port device implementation
 *
 * A PCIe Root Port connected to an external simulation via Unix socket.
 * Raw PCIe TLPs are exchanged using the rpcie wire protocol.
 *
 * Architecture
 * ~~~~~~~~~~~~
 * ┌────────────────────────────┐   Unix socket   ┌───────────────────┐
 * │  QEMU                      │ ◄═══════════► │  Simulation       │
 * │  ┌─────────────────────┐   │   rpcie TLPs  │  (Verilator, VCS) │
 * │  │ remote-pcie-port    │   │               │                   │
 * │  │   (Root Port)       │   │               │  ┌─────────────┐  │
 * │  │   ├── proxy BDF 0   │   │  FN_ADD/     │  │ PF 0        │  │
 * │  │   ├── proxy BDF 1   │   │  FN_REMOVE   │  │ VF 0..N     │  │
 * │  │   └── proxy BDF N   │   │  ◄──────────── │  │ PF 1..M     │  │
 * │  └─────────────────────┘   │               │  └─────────────┘  │
 * └────────────────────────────┘               └───────────────────┘
 *
 * Key design decisions:
 *  - One socket = one PCIe link; all PFs/VFs share it.
 *  - Proxy devices appear dynamically via FN_ADD control messages.
 *  - BAR MMIO uses absolute physical addresses for PCIe-faithful modeling.
 *  - Config space reads are forwarded to the simulation.
 *  - Config space writes are forwarded AND applied locally so QEMU's
 *    BAR mapping, MSI-X, MSI, and SR-IOV machinery stays in sync.
 *  - MSI-X uses msix_init() with the simulation's BAR/offset info so
 *    guest writes to MSI-X table are captured locally for interrupt
 *    injection, while other BAR writes are forwarded to the simulation.
 *  - DMA uses pci_dma_read/write through the proxy's address space
 *    so IOMMU (VT-d / SMMU) is respected when configured.
 *  - Sequence numbers are allocated atomically for thread safety.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2025 Open-DPU Project
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/main-loop.h"
#include "qemu/thread.h"
#include "qemu/sockets.h"

#include "hw/pci/pci.h"
#include "hw/pci/pci_bus.h"
#include "hw/pci/pci_bridge.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pcie_port.h"
#include "hw/pci/msix.h"
#include "hw/pci/msi.h"
#include "hw/pci/pcie.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/dma.h"

#include "hw/remote/remote-pcie-port.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/mman.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>

/* ================================================================== */
/*  Low-level socket helpers                                          */
/* ================================================================== */

static int rpcie_recv_full(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    size_t   remaining = len;

    while (remaining > 0) {
        ssize_t n = recv(fd, p, remaining, 0);
        if (n <= 0) {
            if (n == 0) {
                return -1; /* peer closed */
            }
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        p         += n;
        remaining -= n;
    }
    return 0;
}

static int rpcie_send_full(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    size_t remaining = len;

    while (remaining > 0) {
        ssize_t n = send(fd, p, remaining, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        p         += n;
        remaining -= n;
    }
    return 0;
}

int rpcie_send_msg(RemotePciePort *rp, uint32_t seq,
                   const void *payload, uint32_t len)
{
    rpcie_frame_hdr_t hdr = {
        .magic  = RPCIE_MAGIC,
        .length = len,
        .seq    = seq,
    };
    int rc;

    qemu_mutex_lock(&rp->send_mutex);
    rc = rpcie_send_full(rp->conn_fd, &hdr, sizeof(hdr));
    if (rc == 0 && len > 0) {
        rc = rpcie_send_full(rp->conn_fd, payload, len);
    }
    qemu_mutex_unlock(&rp->send_mutex);
    return rc;
}

/*
 * Thread-safe sequence number allocation.
 * Downstream requests use odd numbers (+=2).
 * Uses atomic fetch-add so multiple vCPU threads can safely
 * issue concurrent BAR reads / config reads.
 */
uint32_t rpcie_alloc_seq(RemotePciePort *rp)
{
    return qatomic_fetch_add(&rp->next_seq, 2);
}

/* ================================================================== */
/*  Completion matching                                               */
/* ================================================================== */

static void rpcie_deliver_completion(RemotePciePort *rp, uint32_t seq,
                                     uint8_t status, uint32_t data)
{
    qemu_mutex_lock(&rp->cpl_mutex);
    int slot = (seq / 2) % RPCIE_CPL_RING_SIZE;
    rp->cpl_ring[slot].seq    = seq;
    rp->cpl_ring[slot].status = status;
    rp->cpl_ring[slot].data   = data;
    rp->cpl_ring[slot].valid  = true;
    qemu_cond_broadcast(&rp->cpl_cond);
    qemu_mutex_unlock(&rp->cpl_mutex);
}

int rpcie_send_and_wait(RemotePciePort *rp, uint32_t seq,
                        const void *payload, uint32_t len,
                        uint32_t *cpl_data, uint8_t *cpl_status,
                        int timeout_ms)
{
    int slot = (seq / 2) % RPCIE_CPL_RING_SIZE;

    /* Clear slot before sending (prevents stale match) */
    qemu_mutex_lock(&rp->cpl_mutex);
    rp->cpl_ring[slot].valid = false;
    qemu_mutex_unlock(&rp->cpl_mutex);

    /* Send the request */
    if (rpcie_send_msg(rp, seq, payload, len) < 0) {
        return -1;
    }

    /* Wait for completion with timeout */
    qemu_mutex_lock(&rp->cpl_mutex);
    int64_t deadline = qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + timeout_ms;

    while (!rp->cpl_ring[slot].valid || rp->cpl_ring[slot].seq != seq) {
        int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
        if (now >= deadline) {
            qemu_mutex_unlock(&rp->cpl_mutex);
            error_report("rpcie: completion timeout for seq %u (slot %d)",
                         seq, slot);
            return -1;
        }
        qemu_cond_timedwait(&rp->cpl_cond, &rp->cpl_mutex,
                            deadline - now);
    }

    if (cpl_data) {
        *cpl_data = rp->cpl_ring[slot].data;
    }
    if (cpl_status) {
        *cpl_status = rp->cpl_ring[slot].status;
    }
    rp->cpl_ring[slot].valid = false;
    qemu_mutex_unlock(&rp->cpl_mutex);
    return 0;
}

/* ================================================================== */
/*  Helpers                                                           */
/* ================================================================== */

/*
 * Compute the absolute physical address for a BAR region access.
 * reads the BAR base directly from config space so it always
 * reflects what the guest has programmed.
 */
static uint64_t rpcie_bar_phys_addr(PCIDevice *d, int bar_idx,
                                    uint8_t bar_type, hwaddr offset)
{
    uint32_t bar_lo = pci_get_long(
        d->config + PCI_BASE_ADDRESS_0 + 4 * bar_idx);
    uint64_t base;

    if (bar_type == RPCIE_BAR_IO) {
        base = bar_lo & PCI_BASE_ADDRESS_IO_MASK;
    } else {
        base = bar_lo & PCI_BASE_ADDRESS_MEM_MASK;
        if (bar_type == RPCIE_BAR_MEM64) {
            uint32_t bar_hi = pci_get_long(
                d->config + PCI_BASE_ADDRESS_0 + 4 * (bar_idx + 1));
            base |= (uint64_t)bar_hi << 32;
        }
    }
    return base + offset;
}

/* Find the proxy device for a given simulation-side BDF. */
static RemotePcieProxy *rpcie_find_proxy(RemotePciePort *rp, uint16_t bdf)
{
    uint8_t devfn = (RPCIE_BDF_DEV(bdf) << 3) | RPCIE_BDF_FN(bdf);
    return rp->proxies[devfn];
}

/* ================================================================== */
/*  Proxy BAR MMIO operations (forwarded to simulation)               */
/* ================================================================== */

typedef struct RpcieBarContext {
    RemotePcieProxy *proxy;
    int              bar_idx;
} RpcieBarContext;

static uint64_t rpcie_bar_read(void *opaque, hwaddr addr, unsigned size)
{
    RpcieBarContext *ctx = opaque;
    RemotePcieProxy *proxy = ctx->proxy;
    RemotePciePort  *rp = proxy->rp;
    PCIDevice       *pci_dev = PCI_DEVICE(proxy);

    /* Compute absolute physical address for PCIe-faithful TLP */
    uint64_t phys_addr = rpcie_bar_phys_addr(
        pci_dev, ctx->bar_idx, proxy->bar_type[ctx->bar_idx], addr);

    /* Build MRd TLP — use 64-bit header if address exceeds 4 GB */
    rpcie_tlp_mem_t tlp;
    memset(&tlp, 0, sizeof(tlp));
    if (phys_addr > UINT32_MAX) {
        tlp.hdr.fmt_type = RPCIE_TLP_MRD64;
    } else {
        tlp.hdr.fmt_type = RPCIE_TLP_MRD32;
    }
    tlp.hdr.tc_attr      = 0;
    tlp.hdr.length_dw    = (size + 3) / 4;
    tlp.hdr.requester_id = 0; /* root complex */
    tlp.hdr.tag          = 0;
    tlp.hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    tlp.address          = phys_addr;

    uint32_t seq = rpcie_alloc_seq(rp);
    uint32_t cpl_data = 0xFFFFFFFF;
    uint8_t  cpl_status = RPCIE_CPL_UR;

    if (rpcie_send_and_wait(rp, seq, &tlp, sizeof(tlp),
                            &cpl_data, &cpl_status, 5000) < 0) {
        return 0xFFFFFFFF;
    }

    if (cpl_status != RPCIE_CPL_SC) {
        return 0xFFFFFFFF;
    }

    /* Extract requested bytes from completion DW */
    int shift = ((int)addr & 3) * 8;
    uint64_t result = (uint64_t)cpl_data >> shift;
    if (size < 4) {
        result &= (1ULL << (size * 8)) - 1;
    }
    return result;
}

static void rpcie_bar_write(void *opaque, hwaddr addr,
                            uint64_t data, unsigned size)
{
    RpcieBarContext *ctx = opaque;
    RemotePcieProxy *proxy = ctx->proxy;
    RemotePciePort  *rp = proxy->rp;
    PCIDevice       *pci_dev = PCI_DEVICE(proxy);

    uint64_t phys_addr = rpcie_bar_phys_addr(
        pci_dev, ctx->bar_idx, proxy->bar_type[ctx->bar_idx], addr);

    /* Build MWr TLP + data payload */
    uint8_t buf[sizeof(rpcie_tlp_mem_t) + 4];
    rpcie_tlp_mem_t *tlp = (rpcie_tlp_mem_t *)buf;

    memset(buf, 0, sizeof(buf));
    if (phys_addr > UINT32_MAX) {
        tlp->hdr.fmt_type = RPCIE_TLP_MWR64;
    } else {
        tlp->hdr.fmt_type = RPCIE_TLP_MWR32;
    }
    tlp->hdr.tc_attr      = 0;
    tlp->hdr.length_dw    = 1;
    tlp->hdr.requester_id = 0; /* root complex */
    tlp->hdr.tag          = 0;
    tlp->hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    tlp->address          = phys_addr;

    /* Place write data in DW alignment */
    uint32_t wdata = (uint32_t)(data << (((int)addr & 3) * 8));
    memcpy(buf + sizeof(rpcie_tlp_mem_t), &wdata, 4);

    uint32_t seq = rpcie_alloc_seq(rp);
    /* MWr is posted — no completion expected */
    rpcie_send_msg(rp, seq, buf, sizeof(rpcie_tlp_mem_t) + 4);
}

static const MemoryRegionOps rpcie_bar_ops = {
    .read  = rpcie_bar_read,
    .write = rpcie_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ================================================================== */
/*  Proxy config space forwarding                                     */
/* ================================================================== */

static uint32_t rpcie_proxy_config_read(PCIDevice *d,
                                        uint32_t address, int len)
{
    RemotePcieProxy *proxy = REMOTE_PCIE_PROXY(d);
    RemotePciePort  *rp = proxy->rp;

    /* Don't forward if socket is down (return local config) */
    if (rp->conn_fd < 0 || !rp->rx_running) {
        return pci_default_read_config(d, address, len);
    }

    rpcie_tlp_cfg_t tlp;
    memset(&tlp, 0, sizeof(tlp));
    tlp.hdr.fmt_type     = RPCIE_TLP_CFGRD0;
    tlp.hdr.tc_attr      = 0;
    tlp.hdr.length_dw    = 1;
    tlp.hdr.requester_id = 0; /* root complex */
    tlp.hdr.tag          = 0;
    tlp.hdr.byte_enables = rpcie_compute_first_be(address, len);
    tlp.completer_id     = proxy->remote_bdf;
    tlp.reg_addr         = address & 0xFFFC;

    uint32_t seq = rpcie_alloc_seq(rp);
    uint32_t cpl_data  = 0xFFFFFFFF;
    uint8_t  cpl_status = RPCIE_CPL_UR;

    if (rpcie_send_and_wait(rp, seq, &tlp, sizeof(tlp),
                            &cpl_data, &cpl_status, 5000) < 0) {
        return 0xFFFFFFFF;
    }
    if (cpl_status != RPCIE_CPL_SC) {
        return 0xFFFFFFFF;
    }

    /* Extract sub-DW access */
    int shift = (address & 3) * 8;
    uint32_t result = cpl_data >> shift;
    if (len < 4) {
        result &= (1u << (len * 8)) - 1;
    }
    return result;
}

static void rpcie_proxy_config_write(PCIDevice *d, uint32_t address,
                                     uint32_t val, int len)
{
    RemotePcieProxy *proxy = REMOTE_PCIE_PROXY(d);
    RemotePciePort  *rp = proxy->rp;

    /*
     * 1) Apply locally so QEMU's BAR mapping, MSI-X, MSI, and
     *    SR-IOV state tracking stay in sync with the guest.
     *    pci_default_write_config() calls:
     *      - pci_update_mappings() → BAR address changes
     *      - msi_write_config()    → MSI enable/disable
     *      - msix_write_config()   → MSI-X enable/disable
     *      - pcie_sriov_config_write() → SR-IOV
     */
    pci_default_write_config(d, address, val, len);

    /*
     * 2) Forward to simulation.
     */
    if (rp->conn_fd < 0 || !rp->rx_running) {
        return;
    }

    uint8_t buf[sizeof(rpcie_tlp_cfg_t) + 4];
    rpcie_tlp_cfg_t *tlp = (rpcie_tlp_cfg_t *)buf;

    memset(buf, 0, sizeof(buf));
    tlp->hdr.fmt_type     = RPCIE_TLP_CFGWR0;
    tlp->hdr.tc_attr      = 0;
    tlp->hdr.length_dw    = 1;
    tlp->hdr.requester_id = 0; /* root complex */
    tlp->hdr.tag          = 0;
    tlp->hdr.byte_enables = rpcie_compute_first_be(address, len);
    tlp->completer_id     = proxy->remote_bdf;
    tlp->reg_addr         = address & 0xFFFC;

    /* Place write data in DW alignment */
    uint32_t dw = val << ((address & 3) * 8);
    memcpy(buf + sizeof(rpcie_tlp_cfg_t), &dw, 4);

    uint32_t seq = rpcie_alloc_seq(rp);
    uint32_t cpl_data;
    uint8_t  cpl_status;

    /* Config writes get a Cpl (non-posted) */
    rpcie_send_and_wait(rp, seq, buf, sizeof(rpcie_tlp_cfg_t) + 4,
                        &cpl_data, &cpl_status, 5000);

    /*
     * 3) Detect FLR trigger: guest wrote PCI_EXP_DEVCTL with BCR_FLR bit.
     *    Forward as a RESET control message.
     */
    if (proxy->flr_capable && d->exp.exp_cap &&
        ranges_overlap(address, len,
                       d->exp.exp_cap + PCI_EXP_DEVCTL, 2)) {
        uint16_t devctl = pci_get_word(
            d->config + d->exp.exp_cap + PCI_EXP_DEVCTL);
        if (devctl & PCI_EXP_DEVCTL_BCR_FLR) {
            rpcie_ctrl_reset_t rst = {
                .msg_type   = RPCIE_CTRL_RESET,
                .reset_type = RPCIE_RESET_FLR,
                .target_bdf = proxy->remote_bdf,
            };
            rpcie_send_msg(rp, rpcie_alloc_seq(rp), &rst, sizeof(rst));
        }
    }
}

/* ================================================================== */
/*  RX thread — reads incoming messages from simulation               */
/* ================================================================== */

static void rpcie_handle_fn_add_bh(void *opaque);
static void rpcie_handle_fn_remove_bh(void *opaque);

typedef struct RpcieFnAddWork {
    RemotePciePort     *rp;
    rpcie_ctrl_fn_add_t fn;
    rpcie_bar_desc_t    bars[RPCIE_MAX_BARS];
    rpcie_msix_info_t   msix_info;
    rpcie_msi_info_t    msi_info;
    bool                has_msix;
    bool                has_msi;
} RpcieFnAddWork;

typedef struct RpcieFnRemoveWork {
    RemotePciePort *rp;
    uint16_t        bdf;
} RpcieFnRemoveWork;

/* --- Completion handling ------------------------------------------ */

static void rpcie_rx_handle_cpl(RemotePciePort *rp, uint32_t seq,
                                const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_cpl_t)) {
        error_report("rpcie: completion too short (%u bytes)", len);
        return;
    }
    const rpcie_tlp_cpl_t *cpl = (const rpcie_tlp_cpl_t *)payload;
    uint8_t  status = RPCIE_CPL_STATUS(cpl->status_bcm_bc);
    uint32_t data = 0;

    /* If CplD, data follows the header */
    if (cpl->hdr.fmt_type == RPCIE_TLP_CPLD &&
        len >= sizeof(rpcie_tlp_cpl_t) + 4) {
        memcpy(&data, payload + sizeof(rpcie_tlp_cpl_t), 4);
    }

    rpcie_deliver_completion(rp, seq, status, data);
}

/* --- DMA write (sim → host memory) -------------------------------- */

static void rpcie_rx_handle_dma_write(RemotePciePort *rp,
                                      const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_mem_t)) {
        return;
    }
    const rpcie_tlp_mem_t *tlp = (const rpcie_tlp_mem_t *)payload;
    uint64_t addr = tlp->address;
    uint16_t ndw  = tlp->hdr.length_dw ? tlp->hdr.length_dw : 1024;
    uint32_t data_bytes = ndw * 4;
    const uint8_t *data = payload + sizeof(rpcie_tlp_mem_t);

    if (len < sizeof(rpcie_tlp_mem_t) + data_bytes) {
        error_report("rpcie: DMA write truncated (need %u+%u, got %u)",
                     (uint32_t)sizeof(rpcie_tlp_mem_t), data_bytes, len);
        return;
    }

    /*
     * Find the originating proxy by requester BDF so DMA goes through
     * the proxy's AddressSpace (respects IOMMU if configured).
     */
    uint16_t req_bdf = tlp->hdr.requester_id;
    RemotePcieProxy *proxy = rpcie_find_proxy(rp, req_bdf);

    if (proxy) {
        pci_dma_write(PCI_DEVICE(proxy), addr, data, data_bytes);
    } else {
        /* Fallback to flat address space (no IOMMU) */
        dma_memory_write(&address_space_memory, addr, data, data_bytes,
                         MEMTXATTRS_UNSPECIFIED);
    }
}

/* --- DMA read (sim reads host memory) ------------------------------ */

static void rpcie_rx_handle_dma_read(RemotePciePort *rp, uint32_t seq,
                                     const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_mem_t)) {
        return;
    }
    const rpcie_tlp_mem_t *tlp = (const rpcie_tlp_mem_t *)payload;
    uint64_t addr = tlp->address;
    uint16_t ndw  = tlp->hdr.length_dw ? tlp->hdr.length_dw : 1024;
    uint32_t data_bytes = ndw * 4;

    if (data_bytes > RPCIE_MAX_PAYLOAD) {
        data_bytes = RPCIE_MAX_PAYLOAD;
        ndw = data_bytes / 4;
    }

    /* Build CplD response */
    uint8_t buf[sizeof(rpcie_tlp_cpl_t) + RPCIE_MAX_PAYLOAD];
    rpcie_tlp_cpl_t *cpl = (rpcie_tlp_cpl_t *)buf;

    rpcie_build_cpld(cpl,
                     0,                          /* completer = RC */
                     tlp->hdr.requester_id,      /* original requester */
                     tlp->hdr.tag,
                     RPCIE_CPL_SC,
                     data_bytes,
                     (uint8_t)(addr & 0x7F),
                     ndw);

    /* Read through proxy's AddressSpace if available */
    uint16_t req_bdf = tlp->hdr.requester_id;
    RemotePcieProxy *proxy = rpcie_find_proxy(rp, req_bdf);

    if (proxy) {
        pci_dma_read(PCI_DEVICE(proxy), addr,
                     buf + sizeof(rpcie_tlp_cpl_t), data_bytes);
    } else {
        dma_memory_read(&address_space_memory, addr,
                        buf + sizeof(rpcie_tlp_cpl_t), data_bytes,
                        MEMTXATTRS_UNSPECIFIED);
    }

    rpcie_send_msg(rp, seq, buf, sizeof(rpcie_tlp_cpl_t) + data_bytes);
}

/* --- MSI-X / MSI interrupt notification ---------------------------- */

static void rpcie_rx_handle_interrupt(RemotePciePort *rp,
                                      const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_msg_t)) {
        return;
    }
    const rpcie_tlp_msg_t *msg = (const rpcie_tlp_msg_t *)payload;
    uint16_t req_bdf = msg->hdr.requester_id;

    RemotePcieProxy *proxy = rpcie_find_proxy(rp, req_bdf);
    if (!proxy) {
        error_report("rpcie: interrupt for unknown BDF %04x", req_bdf);
        return;
    }

    /* Extract vector from data payload */
    uint32_t msi_data = 0;
    if (len >= sizeof(rpcie_tlp_msg_t) + 4) {
        memcpy(&msi_data, payload + sizeof(rpcie_tlp_msg_t), 4);
    }
    uint16_t vector = msi_data & 0x7FF;

    PCIDevice *pci_dev = PCI_DEVICE(proxy);

    if (msg->msg_code == RPCIE_MSG_MSIX_NOTIFY) {
        if (proxy->has_msix && msix_present(pci_dev)) {
            msix_notify(pci_dev, vector);
        }
    } else if (msg->msg_code == RPCIE_MSG_MSI_NOTIFY) {
        if (proxy->has_msi && msi_enabled(pci_dev)) {
            msi_notify(pci_dev, vector);
        }
    }
}

/* --- INTx assert/deassert ----------------------------------------- */

static void rpcie_rx_handle_intx(RemotePciePort *rp,
                                 const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_msg_t)) {
        return;
    }
    const rpcie_tlp_msg_t *msg = (const rpcie_tlp_msg_t *)payload;
    uint16_t req_bdf = msg->hdr.requester_id;

    RemotePcieProxy *proxy = rpcie_find_proxy(rp, req_bdf);
    if (!proxy) {
        return;
    }

    int level;
    if (msg->msg_code == RPCIE_MSG_INTX_ASSERT) {
        level = 1;
    } else if (msg->msg_code == RPCIE_MSG_INTX_DEASSERT) {
        level = 0;
    } else {
        return;
    }

    if (proxy->intx_level != level) {
        proxy->intx_level = level;
        pci_set_irq(PCI_DEVICE(proxy), level);
    }
}

/* --- AER error message -------------------------------------------- */

static void rpcie_rx_handle_aer(RemotePciePort *rp,
                                const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_msg_t)) {
        return;
    }
    const rpcie_tlp_msg_t *msg = (const rpcie_tlp_msg_t *)payload;

    /*
     * Forward AER error to root port.  In QEMU, AER is handled by
     * the root port's AER capability.  We inject the error using
     * pcie_aer_inject_error() or by directly signaling the root port.
     *
     * For now, log the error and signal the root port MSI.
     * A complete implementation would parse the AER TLP-level error
     * info (header log, source ID) and update the root port's AER
     * registers.
     */
    const char *sev;
    switch (msg->msg_code) {
    case RPCIE_MSG_AER_ERR_COR:
        sev = "correctable";
        break;
    case RPCIE_MSG_AER_ERR_NONFATAL:
        sev = "non-fatal";
        break;
    case RPCIE_MSG_AER_ERR_FATAL:
        sev = "fatal";
        break;
    default:
        sev = "unknown";
        break;
    }

    info_report("rpcie: AER %s error from BDF %04x",
                sev, msg->hdr.requester_id);

    /* Signal root port's AER interrupt (vector 0) */
    PCIDevice *rp_dev = PCI_DEVICE(rp);
    if (msix_present(rp_dev)) {
        msix_notify(rp_dev, 0);
    }
}

/* --- FN_ADD / FN_REMOVE parsing ----------------------------------- */

static void rpcie_rx_handle_fn_add(RemotePciePort *rp,
                                   const uint8_t *payload, uint32_t plen)
{
    if (plen < sizeof(rpcie_ctrl_fn_add_t)) {
        error_report("rpcie: FN_ADD too short (%u bytes)", plen);
        return;
    }

    RpcieFnAddWork *work = g_new0(RpcieFnAddWork, 1);
    work->rp = rp;
    memcpy(&work->fn, payload, sizeof(rpcie_ctrl_fn_add_t));

    /* Parse BAR descriptors */
    const uint8_t *p = payload + sizeof(rpcie_ctrl_fn_add_t);
    uint32_t remaining = plen - sizeof(rpcie_ctrl_fn_add_t);
    int nb = work->fn.num_bars;
    if (nb > RPCIE_MAX_BARS) {
        nb = RPCIE_MAX_BARS;
    }

    for (int i = 0; i < nb && remaining >= sizeof(rpcie_bar_desc_t); i++) {
        memcpy(&work->bars[i], p, sizeof(rpcie_bar_desc_t));
        p         += sizeof(rpcie_bar_desc_t);
        remaining -= sizeof(rpcie_bar_desc_t);
    }

    /* Parse MSI-X info if present */
    if (work->fn.has_msix && remaining >= sizeof(rpcie_msix_info_t)) {
        memcpy(&work->msix_info, p, sizeof(rpcie_msix_info_t));
        work->has_msix = true;
        p         += sizeof(rpcie_msix_info_t);
        remaining -= sizeof(rpcie_msix_info_t);
    }

    /* Parse MSI info if present */
    if (work->fn.has_msi && remaining >= sizeof(rpcie_msi_info_t)) {
        memcpy(&work->msi_info, p, sizeof(rpcie_msi_info_t));
        work->has_msi = true;
        p         += sizeof(rpcie_msi_info_t);
        remaining -= sizeof(rpcie_msi_info_t);
    }

    /* Schedule on main loop (device creation must happen there) */
    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            rpcie_handle_fn_add_bh, work);
}

static void rpcie_rx_handle_fn_remove(RemotePciePort *rp,
                                      const uint8_t *payload, uint32_t plen)
{
    if (plen < sizeof(rpcie_ctrl_fn_remove_t)) {
        return;
    }
    const rpcie_ctrl_fn_remove_t *msg =
        (const rpcie_ctrl_fn_remove_t *)payload;

    RpcieFnRemoveWork *work = g_new0(RpcieFnRemoveWork, 1);
    work->rp  = rp;
    work->bdf = msg->bdf;

    aio_bh_schedule_oneshot(qemu_get_aio_context(),
                            rpcie_handle_fn_remove_bh, work);
}

/* ================================================================== */
/*  RX thread main loop                                               */
/* ================================================================== */

static void *rpcie_rx_thread_fn(void *opaque)
{
    RemotePciePort *rp = opaque;
    rpcie_frame_hdr_t hdr;
    uint8_t buf[sizeof(rpcie_frame_hdr_t) + RPCIE_MAX_PAYLOAD + 256];

    while (rp->rx_running) {
        /* Poll with timeout so we can check rx_running */
        struct pollfd pfd = { .fd = rp->conn_fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 100);
        if (pr <= 0) {
            continue;
        }

        if (rpcie_recv_full(rp->conn_fd, &hdr, sizeof(hdr)) < 0) {
            error_report("rpcie: connection closed by peer");
            rp->rx_running = false;
            break;
        }

        if (hdr.magic != RPCIE_MAGIC) {
            error_report("rpcie: bad magic 0x%08x", hdr.magic);
            rp->rx_running = false;
            break;
        }

        uint32_t plen = hdr.length;
        if (plen > sizeof(buf)) {
            error_report("rpcie: payload too large (%u)", plen);
            rp->rx_running = false;
            break;
        }

        if (plen > 0) {
            if (rpcie_recv_full(rp->conn_fd, buf, plen) < 0) {
                rp->rx_running = false;
                break;
            }
        }

        if (plen == 0) {
            continue;
        }

        uint8_t fmt_type = buf[0];

        if (RPCIE_IS_CTRL(fmt_type)) {
            /* Control message */
            switch (fmt_type) {
            case RPCIE_CTRL_FN_ADD:
                rpcie_rx_handle_fn_add(rp, buf, plen);
                break;
            case RPCIE_CTRL_FN_REMOVE:
                rpcie_rx_handle_fn_remove(rp, buf, plen);
                break;
            case RPCIE_CTRL_LINK_DOWN:
                info_report("rpcie: simulation sent LINK_DOWN");
                rp->rx_running = false;
                break;
            case RPCIE_CTRL_RESET:
                /* FLR ack or reset ack from simulation */
                info_report("rpcie: reset acknowledged");
                break;
            default:
                error_report("rpcie: unknown control msg 0x%02x", fmt_type);
                break;
            }
        } else if (RPCIE_TLP_IS_CPL(fmt_type)) {
            rpcie_rx_handle_cpl(rp, hdr.seq, buf, plen);
        } else if (RPCIE_TLP_IS_MEM_WRITE(fmt_type)) {
            rpcie_rx_handle_dma_write(rp, buf, plen);
        } else if (RPCIE_TLP_IS_MEM_READ(fmt_type)) {
            rpcie_rx_handle_dma_read(rp, hdr.seq, buf, plen);
        } else if (fmt_type == RPCIE_TLP_MSGD &&
                   plen >= sizeof(rpcie_tlp_msg_t)) {
            const rpcie_tlp_msg_t *msg = (const rpcie_tlp_msg_t *)buf;
            switch (msg->msg_code) {
            case RPCIE_MSG_MSIX_NOTIFY:
            case RPCIE_MSG_MSI_NOTIFY:
                rpcie_rx_handle_interrupt(rp, buf, plen);
                break;
            case RPCIE_MSG_INTX_ASSERT:
            case RPCIE_MSG_INTX_DEASSERT:
                rpcie_rx_handle_intx(rp, buf, plen);
                break;
            case RPCIE_MSG_AER_ERR_COR:
            case RPCIE_MSG_AER_ERR_NONFATAL:
            case RPCIE_MSG_AER_ERR_FATAL:
                rpcie_rx_handle_aer(rp, buf, plen);
                break;
            case RPCIE_MSG_PME:
                info_report("rpcie: PME from BDF %04x (TODO: forward)",
                            msg->hdr.requester_id);
                break;
            default:
                error_report("rpcie: unhandled msg code 0x%02x",
                             msg->msg_code);
                break;
            }
        } else if (fmt_type == RPCIE_TLP_MSG &&
                   plen >= sizeof(rpcie_tlp_msg_t)) {
            /* Message without data — same dispatch */
            const rpcie_tlp_msg_t *msg = (const rpcie_tlp_msg_t *)buf;
            switch (msg->msg_code) {
            case RPCIE_MSG_INTX_ASSERT:
            case RPCIE_MSG_INTX_DEASSERT:
                rpcie_rx_handle_intx(rp, buf, plen);
                break;
            default:
                error_report("rpcie: unhandled MSG 0x%02x", msg->msg_code);
                break;
            }
        } else {
            error_report("rpcie: unhandled TLP type 0x%02x", fmt_type);
        }
    }

    return NULL;
}

/* ================================================================== */
/*  FN_ADD / FN_REMOVE handlers (run on main loop via BH)             */
/* ================================================================== */

static void rpcie_handle_fn_add_bh(void *opaque)
{
    RpcieFnAddWork *work = opaque;
    RemotePciePort *rp = work->rp;
    Error *local_err = NULL;

    uint16_t bdf = work->fn.bdf;
    uint8_t  devfn = (RPCIE_BDF_DEV(bdf) << 3) | RPCIE_BDF_FN(bdf);

    if (rp->proxies[devfn]) {
        error_report("rpcie: FN_ADD for already-occupied devfn 0x%02x", devfn);
        g_free(work);
        return;
    }

    /* Get the secondary bus of this root port */
    PCIBus *sec_bus = pci_bridge_get_sec_bus(PCI_BRIDGE(rp));
    if (!sec_bus) {
        error_report("rpcie: no secondary bus for FN_ADD");
        g_free(work);
        return;
    }

    /* Create the proxy device */
    DeviceState *dev = qdev_new(TYPE_REMOTE_PCIE_PROXY);
    RemotePcieProxy *proxy = REMOTE_PCIE_PROXY(dev);

    proxy->rp         = rp;
    proxy->remote_bdf = bdf;
    proxy->is_vf      = (work->fn.fn_type == 1);
    proxy->pf_bdf     = work->fn.pf_bdf;
    proxy->num_bars   = work->fn.num_bars;
    if (proxy->num_bars > RPCIE_PROXY_MAX_BARS) {
        proxy->num_bars = RPCIE_PROXY_MAX_BARS;
    }

    /* PCI identity */
    proxy->vendor_id       = work->fn.vendor_id;
    proxy->device_id       = work->fn.device_id;
    proxy->subsys_vendor_id = work->fn.subsys_vendor_id;
    proxy->subsys_id       = work->fn.subsys_id;
    memcpy(proxy->class_code, work->fn.class_code, 3);
    proxy->revision        = work->fn.revision;

    /* Copy BAR info */
    for (int i = 0; i < proxy->num_bars; i++) {
        proxy->bar_size[i]         = work->bars[i].size;
        proxy->bar_type[i]         = work->bars[i].type;
        proxy->bar_prefetchable[i] = work->bars[i].prefetchable;
    }

    /* Copy MSI-X info */
    proxy->has_msix     = work->has_msix;
    if (work->has_msix) {
        proxy->msix_vectors      = work->msix_info.table_size;
        proxy->msix_table_bar    = work->msix_info.table_bar;
        proxy->msix_table_offset = work->msix_info.table_offset;
        proxy->msix_pba_bar      = work->msix_info.pba_bar;
        proxy->msix_pba_offset   = work->msix_info.pba_offset;
    }

    /* Copy MSI info */
    proxy->has_msi = work->has_msi;
    if (work->has_msi) {
        proxy->msi_vectors        = work->msi_info.num_vectors;
        proxy->msi_64bit          = work->msi_info.is_64bit;
        proxy->msi_per_vector_mask = work->msi_info.per_vector_mask;
    }

    /* FLR */
    proxy->flr_capable  = work->fn.flr_capable;

    /* Place on the secondary bus at the right devfn */
    PCI_DEVICE(dev)->devfn = devfn;
    qdev_prop_set_int32(dev, "addr", PCI_SLOT(devfn));

    if (!qdev_realize_and_unref(dev, BUS(sec_bus), &local_err)) {
        error_reportf_err(local_err, "rpcie: failed to realize proxy: ");
        g_free(work);
        return;
    }

    /*
     * Set PCI identity in local config space.  pci_qdev_realize() writes
     * the class vendor/device_id (0xFFFF) into config[]; overwrite them
     * now so QEMU-internal code (and lspci in the guest, if it falls back
     * to local config) sees correct values.
     */
    PCIDevice *pci_dev = PCI_DEVICE(proxy);
    pci_config_set_vendor_id(pci_dev->config, proxy->vendor_id);
    pci_config_set_device_id(pci_dev->config, proxy->device_id);
    pci_set_word(pci_dev->config + PCI_SUBSYSTEM_VENDOR_ID,
                 proxy->subsys_vendor_id);
    pci_set_word(pci_dev->config + PCI_SUBSYSTEM_ID, proxy->subsys_id);
    pci_config_set_class(pci_dev->config,
                         ((uint16_t)proxy->class_code[2] << 8) |
                         proxy->class_code[1]);
    pci_config_set_prog_interface(pci_dev->config, proxy->class_code[0]);
    pci_set_byte(pci_dev->config + PCI_REVISION_ID, proxy->revision);

    rp->proxies[devfn] = proxy;

    info_report("rpcie: added function %02x:%02x.%x (%s, %04x:%04x, "
                "%d BARs%s%s%s)",
                RPCIE_BDF_BUS(bdf), RPCIE_BDF_DEV(bdf), RPCIE_BDF_FN(bdf),
                proxy->is_vf ? "VF" : "PF",
                proxy->vendor_id, proxy->device_id,
                proxy->num_bars,
                proxy->has_msix ? ", MSI-X" : "",
                proxy->has_msi  ? ", MSI"   : "",
                proxy->flr_capable ? ", FLR" : "");

    g_free(work);
}

static void rpcie_handle_fn_remove_bh(void *opaque)
{
    RpcieFnRemoveWork *work = opaque;
    RemotePciePort *rp = work->rp;
    uint16_t bdf = work->bdf;
    uint8_t  devfn = (RPCIE_BDF_DEV(bdf) << 3) | RPCIE_BDF_FN(bdf);

    RemotePcieProxy *proxy = rp->proxies[devfn];
    if (!proxy) {
        error_report("rpcie: FN_REMOVE for non-existent devfn 0x%02x", devfn);
        g_free(work);
        return;
    }

    info_report("rpcie: removing function %02x:%02x.%x",
                RPCIE_BDF_BUS(bdf), RPCIE_BDF_DEV(bdf), RPCIE_BDF_FN(bdf));

    rp->proxies[devfn] = NULL;
    qdev_unrealize(DEVICE(proxy));

    g_free(work);
}

/* ================================================================== */
/*  Handshake                                                         */
/* ================================================================== */

static int rpcie_do_handshake(RemotePciePort *rp)
{
    /* Send HANDSHAKE */
    rpcie_ctrl_handshake_t hs = {
        .msg_type     = RPCIE_CTRL_HANDSHAKE,
        .version      = RPCIE_PROTOCOL_VERSION,
        .capabilities = RPCIE_CAP_SRIOV | RPCIE_CAP_AER |
                        RPCIE_CAP_SHM | RPCIE_CAP_FLR |
                        RPCIE_CAP_HOTPLUG,
    };

    uint32_t seq = rpcie_alloc_seq(rp);
    if (rpcie_send_msg(rp, seq, &hs, sizeof(hs)) < 0) {
        error_report("rpcie: failed to send handshake");
        return -1;
    }

    /* Receive HANDSHAKE_ACK */
    rpcie_frame_hdr_t hdr;
    if (rpcie_recv_full(rp->conn_fd, &hdr, sizeof(hdr)) < 0) {
        error_report("rpcie: failed to receive handshake ack");
        return -1;
    }
    if (hdr.magic != RPCIE_MAGIC) {
        error_report("rpcie: bad handshake ack magic 0x%08x", hdr.magic);
        return -1;
    }

    rpcie_ctrl_handshake_t ack;
    if (hdr.length < sizeof(ack)) {
        error_report("rpcie: handshake ack too short (%u)", hdr.length);
        return -1;
    }
    if (rpcie_recv_full(rp->conn_fd, &ack, sizeof(ack)) < 0) {
        error_report("rpcie: failed to read handshake ack body");
        return -1;
    }
    /* Drain any extra bytes (forward compatibility) */
    if (hdr.length > sizeof(ack)) {
        uint8_t drain[256];
        uint32_t extra = hdr.length - sizeof(ack);
        while (extra > 0) {
            uint32_t chunk = extra > sizeof(drain) ? sizeof(drain) : extra;
            if (rpcie_recv_full(rp->conn_fd, drain, chunk) < 0) {
                break;
            }
            extra -= chunk;
        }
    }

    if (ack.msg_type != RPCIE_CTRL_HANDSHAKE_ACK) {
        error_report("rpcie: expected HANDSHAKE_ACK, got 0x%02x", ack.msg_type);
        return -1;
    }
    if (ack.version != RPCIE_PROTOCOL_VERSION) {
        error_report("rpcie: version mismatch (remote %d, local %d)",
                     ack.version, RPCIE_PROTOCOL_VERSION);
        return -1;
    }

    rp->remote_caps = hs.capabilities & ack.capabilities;
    info_report("rpcie: handshake OK, version %d, caps 0x%08x",
                ack.version, rp->remote_caps);
    return 0;
}

/* ================================================================== */
/*  Socket setup                                                      */
/* ================================================================== */

static int rpcie_setup_socket(RemotePciePort *rp, Error **errp)
{
    struct sockaddr_un addr;

    if (!rp->socket_path || strlen(rp->socket_path) == 0) {
        error_setg(errp, "rpcie: 'socket' property is required");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, rp->socket_path, sizeof(addr.sun_path) - 1);

    if (rp->server) {
        /*
         * Server mode: create socket, bind, listen, accept.
         * Uses a generous timeout (60s) to avoid blocking QEMU
         * indefinitely if the simulation is slow to start.
         */
        rp->listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (rp->listen_fd < 0) {
            error_setg_errno(errp, errno, "rpcie: socket()");
            return -1;
        }

        unlink(rp->socket_path);
        if (bind(rp->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            error_setg_errno(errp, errno, "rpcie: bind(%s)", rp->socket_path);
            close(rp->listen_fd);
            rp->listen_fd = -1;
            return -1;
        }

        if (listen(rp->listen_fd, 1) < 0) {
            error_setg_errno(errp, errno, "rpcie: listen()");
            close(rp->listen_fd);
            rp->listen_fd = -1;
            return -1;
        }

        info_report("rpcie: listening on %s (waiting up to 60s)", rp->socket_path);

        /* Accept with timeout */
        struct pollfd pfd = { .fd = rp->listen_fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 60000); /* 60 seconds */
        if (pr <= 0) {
            error_setg(errp, "rpcie: accept timeout (no simulation connected "
                       "within 60s)");
            close(rp->listen_fd);
            rp->listen_fd = -1;
            return -1;
        }

        rp->conn_fd = accept(rp->listen_fd, NULL, NULL);
        if (rp->conn_fd < 0) {
            error_setg_errno(errp, errno, "rpcie: accept()");
            close(rp->listen_fd);
            rp->listen_fd = -1;
            return -1;
        }

        info_report("rpcie: simulation connected");
    } else {
        /* Client mode: connect to existing server */
        rp->conn_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (rp->conn_fd < 0) {
            error_setg_errno(errp, errno, "rpcie: socket()");
            return -1;
        }

        /* Retry connection (simulation may not be ready yet) */
        for (int i = 0; i < 100; i++) {
            if (connect(rp->conn_fd, (struct sockaddr *)&addr,
                        sizeof(addr)) == 0) {
                goto connected;
            }
            if (errno != ENOENT && errno != ECONNREFUSED) {
                error_setg_errno(errp, errno, "rpcie: connect(%s)",
                                 rp->socket_path);
                close(rp->conn_fd);
                rp->conn_fd = -1;
                return -1;
            }
            usleep(100000); /* 100ms */
        }
        error_setg(errp, "rpcie: connect(%s) timed out after 10s",
                   rp->socket_path);
        close(rp->conn_fd);
        rp->conn_fd = -1;
        return -1;

    connected:
        info_report("rpcie: connected to %s", rp->socket_path);
    }

    return 0;
}

/* ================================================================== */
/*  Reset forwarding                                                  */
/* ================================================================== */

static void rpcie_send_reset(RemotePciePort *rp, uint8_t reset_type,
                             uint16_t target_bdf)
{
    rpcie_ctrl_reset_t msg = {
        .msg_type   = RPCIE_CTRL_RESET,
        .reset_type = reset_type,
        .target_bdf = target_bdf,
    };
    uint32_t seq = rpcie_alloc_seq(rp);
    rpcie_send_msg(rp, seq, &msg, sizeof(msg));
}

static void rpcie_rp_reset_hold(Object *obj, ResetType type)
{
    PCIDevice *d = PCI_DEVICE(obj);
    RemotePciePort *rp = REMOTE_PCIE_PORT(obj);

    /* Standard root port reset */
    pcie_cap_root_reset(d);
    pcie_cap_deverr_reset(d);
    pcie_cap_slot_reset(d);
    pcie_cap_arifwd_reset(d);
    pcie_aer_root_reset(d);
    pci_bridge_reset(DEVICE(d));
    pci_bridge_disable_base_limit(d);

    /* Forward reset downstream */
    if (rp->conn_fd >= 0 && rp->rx_running) {
        rpcie_send_reset(rp, RPCIE_RESET_HOT, RPCIE_BDF_ALL);
    }
}

/* ================================================================== */
/*  Root Port realize / unrealize                                     */
/* ================================================================== */

static void rpcie_rp_aer_vector_update(PCIDevice *d)
{
    pcie_aer_root_set_vector(d, 0);
}

static int rpcie_rp_interrupts_init(PCIDevice *d, Error **errp)
{
    int rc = msix_init_exclusive_bar(d, RPCIE_MSIX_NR_VECTOR, 0, errp);
    if (rc < 0) {
        assert(rc == -ENOTSUP);
    } else {
        msix_vector_use(d, 0);
    }
    return rc;
}

static void rpcie_rp_interrupts_uninit(PCIDevice *d)
{
    msix_uninit_exclusive_bar(d);
}

static void rpcie_rp_config_write(PCIDevice *d, uint32_t address,
                                  uint32_t val, int len)
{
    uint32_t root_cmd =
        pci_get_long(d->config + d->exp.aer_cap + PCI_ERR_ROOT_COMMAND);
    uint16_t slt_ctl, slt_sta;

    pcie_cap_slot_get(d, &slt_ctl, &slt_sta);
    pci_bridge_write_config(d, address, val, len);
    rpcie_rp_aer_vector_update(d);
    pcie_cap_slot_write_config(d, slt_ctl, slt_sta, address, val, len);
    pcie_aer_write_config(d, address, val, len);
    pcie_aer_root_write_config(d, address, val, len, root_cmd);
}

static void rpcie_rp_realize(DeviceState *dev, Error **errp)
{
    PCIDevice *d = PCI_DEVICE(dev);
    PCIERootPortClass *rpc = PCIE_ROOT_PORT_GET_CLASS(d);
    RemotePciePort *rp = REMOTE_PCIE_PORT(dev);
    Error *local_err = NULL;

    /* Initialize as a standard PCIe Root Port */
    rpc->parent_realize(dev, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    /* Initialize synchronization */
    qemu_mutex_init(&rp->send_mutex);
    qemu_mutex_init(&rp->cpl_mutex);
    qemu_cond_init(&rp->cpl_cond);
    memset(rp->cpl_ring, 0, sizeof(rp->cpl_ring));
    memset(rp->proxies, 0, sizeof(rp->proxies));
    rp->next_seq  = 1; /* odd numbers for downstream */
    rp->listen_fd = -1;
    rp->conn_fd   = -1;
    rp->shm_fd    = -1;
    rp->shm_ptr   = NULL;

    /* Set up socket connection */
    if (rpcie_setup_socket(rp, errp) < 0) {
        return;
    }

    /* Perform protocol handshake */
    if (rpcie_do_handshake(rp) < 0) {
        error_setg(errp, "rpcie: protocol handshake failed");
        close(rp->conn_fd);
        rp->conn_fd = -1;
        return;
    }

    /* Send LINK_UP */
    rpcie_ctrl_link_t link_up = { .msg_type = RPCIE_CTRL_LINK_UP };
    rpcie_send_msg(rp, rpcie_alloc_seq(rp), &link_up, sizeof(link_up));

    /* Start RX thread */
    rp->rx_running = true;
    qemu_thread_create(&rp->rx_thread, "rpcie-rx",
                       rpcie_rx_thread_fn, rp,
                       QEMU_THREAD_JOINABLE);

    info_report("rpcie: root port ready on %s", rp->socket_path);
}

static void rpcie_rp_exit(PCIDevice *d)
{
    RemotePciePort *rp = REMOTE_PCIE_PORT(d);
    PCIERootPortClass *rpc = PCIE_ROOT_PORT_GET_CLASS(d);

    /* Stop RX thread */
    rp->rx_running = false;
    if (rp->conn_fd >= 0) {
        /* Send LINK_DOWN */
        rpcie_ctrl_link_t link_dn = { .msg_type = RPCIE_CTRL_LINK_DOWN };
        rpcie_send_msg(rp, rpcie_alloc_seq(rp), &link_dn, sizeof(link_dn));
        shutdown(rp->conn_fd, SHUT_RDWR);
    }
    qemu_thread_join(&rp->rx_thread);

    /* Remove all proxy devices */
    for (int i = 0; i < RPCIE_MAX_PROXIES; i++) {
        if (rp->proxies[i]) {
            qdev_unrealize(DEVICE(rp->proxies[i]));
            rp->proxies[i] = NULL;
        }
    }

    /* Cleanup shared memory */
    if (rp->shm_ptr) {
        munmap(rp->shm_ptr, rp->shm_size);
        rp->shm_ptr = NULL;
    }
    if (rp->shm_fd >= 0) {
        close(rp->shm_fd);
        rp->shm_fd = -1;
    }

    /* Close sockets */
    if (rp->conn_fd >= 0) {
        close(rp->conn_fd);
        rp->conn_fd = -1;
    }
    if (rp->listen_fd >= 0) {
        close(rp->listen_fd);
        rp->listen_fd = -1;
        if (rp->socket_path) {
            unlink(rp->socket_path);
        }
    }

    /* Destroy sync primitives */
    qemu_mutex_destroy(&rp->send_mutex);
    qemu_mutex_destroy(&rp->cpl_mutex);
    qemu_cond_destroy(&rp->cpl_cond);

    /* Chain to parent exit */
    rpc->parent_class.exit(d);
}

/* ================================================================== */
/*  Root Port class init / type registration                          */
/* ================================================================== */

static const Property rpcie_rp_props[] = {
    DEFINE_PROP_STRING("socket", RemotePciePort, socket_path),
    DEFINE_PROP_BOOL("server", RemotePciePort, server, false),
};

static const VMStateDescription vmstate_rpcie_rp = {
    .name = "remote-pcie-port",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj.parent_obj.parent_obj, PCIESlot),
        VMSTATE_END_OF_LIST()
    }
};

static void rpcie_rp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    PCIERootPortClass *rpc = PCIE_ROOT_PORT_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    k->vendor_id = PCI_VENDOR_ID_REDHAT;
    k->device_id = 0x0013;  /* unique ID for remote-pcie-port */
    dc->desc = "Remote PCIe Port (TLP-over-socket bridge)";
    dc->vmsd = &vmstate_rpcie_rp;
    device_class_set_props(dc, rpcie_rp_props);

    device_class_set_parent_realize(dc, rpcie_rp_realize,
                                    &rpc->parent_realize);
    k->exit = rpcie_rp_exit;
    k->config_write = rpcie_rp_config_write;

    rpc->aer_vector      = NULL; /* use default vector 0 */
    rpc->interrupts_init = rpcie_rp_interrupts_init;
    rpc->interrupts_uninit = rpcie_rp_interrupts_uninit;
    rpc->aer_offset      = RPCIE_AER_OFFSET;

    rc->phases.hold = rpcie_rp_reset_hold;
}

static const TypeInfo rpcie_rp_info = {
    .name          = TYPE_REMOTE_PCIE_PORT,
    .parent        = TYPE_PCIE_ROOT_PORT,
    .instance_size = sizeof(RemotePciePort),
    .class_init    = rpcie_rp_class_init,
};

/* ================================================================== */
/*  Proxy device realize / unrealize                                  */
/* ================================================================== */

static void rpcie_proxy_realize(PCIDevice *d, Error **errp)
{
    RemotePcieProxy *proxy = REMOTE_PCIE_PROXY(d);

    /* Register BARs */
    for (int i = 0; i < proxy->num_bars; i++) {
        if (proxy->bar_type[i] == RPCIE_BAR_DISABLED ||
            proxy->bar_size[i] == 0) {
            continue;
        }

        RpcieBarContext *ctx = g_new0(RpcieBarContext, 1);
        ctx->proxy   = proxy;
        ctx->bar_idx = i;

        char name[32];
        snprintf(name, sizeof(name), "rpcie-bar%d", i);

        memory_region_init_io(&proxy->bar_mr[i], OBJECT(proxy),
                              &rpcie_bar_ops, ctx, name,
                              proxy->bar_size[i]);

        int pci_type = PCI_BASE_ADDRESS_SPACE_MEMORY;
        if (proxy->bar_type[i] == RPCIE_BAR_MEM64) {
            pci_type |= PCI_BASE_ADDRESS_MEM_TYPE_64;
        }
        if (proxy->bar_type[i] == RPCIE_BAR_IO) {
            pci_type = PCI_BASE_ADDRESS_SPACE_IO;
        }
        if (proxy->bar_prefetchable[i]) {
            pci_type |= PCI_BASE_ADDRESS_MEM_PREFETCH;
        }

        pci_register_bar(d, i, pci_type, &proxy->bar_mr[i]);

        /* Skip next BAR index for 64-bit BARs */
        if (proxy->bar_type[i] == RPCIE_BAR_MEM64) {
            i++;
        }
    }

    /*
     * Initialize MSI-X using the simulation's BAR/offset info.
     * The MSI-X table and PBA regions are placed as sub-regions of
     * the appropriate BAR MemoryRegions, so guest writes to the
     * MSI-X table area are captured by QEMU's MSI-X handler for
     * local interrupt injection.  All other BAR writes pass through
     * to the simulation via rpcie_bar_ops.
     *
     * cap_pos is set to a fixed offset (0x40) for QEMU-internal use.
     * The guest sees the real MSI-X cap offset through config read
     * forwarding.
     */
    if (proxy->has_msix && proxy->msix_vectors > 0) {
        uint8_t tbar = proxy->msix_table_bar;
        uint8_t pbar = proxy->msix_pba_bar;

        /* Validate BAR indices */
        if (tbar < proxy->num_bars && pbar < proxy->num_bars &&
            proxy->bar_mr[tbar].size > 0 && proxy->bar_mr[pbar].size > 0) {
            int rc = msix_init(d, proxy->msix_vectors,
                               &proxy->bar_mr[tbar], tbar,
                               proxy->msix_table_offset,
                               &proxy->bar_mr[pbar], pbar,
                               proxy->msix_pba_offset,
                               0x40, /* cap_pos for QEMU internal use */
                               errp);
            if (rc < 0) {
                error_report("rpcie: MSI-X init failed for devfn 0x%02x: %d",
                             d->devfn, rc);
                /* Non-fatal: continue without local MSI-X */
            }
        } else {
            /* BAR indices out of range — fall back to exclusive BAR */
            int msix_bar = proxy->num_bars;
            if (msix_bar >= RPCIE_PROXY_MAX_BARS) {
                msix_bar = RPCIE_PROXY_MAX_BARS - 1;
            }
            int rc = msix_init_exclusive_bar(d, proxy->msix_vectors,
                                             msix_bar, errp);
            if (rc < 0) {
                error_report("rpcie: MSI-X exclusive bar init failed: %d", rc);
            }
        }
    }

    /*
     * Initialize MSI.
     * cap_pos is set to 0x50 to avoid conflict with MSI-X at 0x40.
     */
    if (proxy->has_msi && proxy->msi_vectors > 0) {
        int rc = msi_init(d, 0x50, proxy->msi_vectors,
                          proxy->msi_64bit,
                          proxy->msi_per_vector_mask, errp);
        if (rc < 0) {
            error_report("rpcie: MSI init failed for devfn 0x%02x: %d",
                         d->devfn, rc);
        }
    }
}

static void rpcie_proxy_exit(PCIDevice *d)
{
    RemotePcieProxy *proxy = REMOTE_PCIE_PROXY(d);

    /* Deassert INTx if still active */
    if (proxy->intx_level) {
        pci_set_irq(d, 0);
        proxy->intx_level = 0;
    }

    /* Uninitialize MSI */
    if (proxy->has_msi && msi_present(d)) {
        msi_uninit(d);
    }

    /* Uninitialize MSI-X */
    if (proxy->has_msix && msix_present(d)) {
        uint8_t tbar = proxy->msix_table_bar;
        uint8_t pbar = proxy->msix_pba_bar;
        if (tbar < proxy->num_bars && pbar < proxy->num_bars &&
            proxy->bar_mr[tbar].size > 0 && proxy->bar_mr[pbar].size > 0) {
            msix_uninit(d, &proxy->bar_mr[tbar], &proxy->bar_mr[pbar]);
        } else {
            msix_uninit_exclusive_bar(d);
        }
    }

    /* Free BAR contexts */
    for (int i = 0; i < proxy->num_bars; i++) {
        if (proxy->bar_mr[i].size) {
            void *ctx = proxy->bar_mr[i].opaque;
            if (ctx) {
                g_free(ctx);
            }
        }
    }
}

static void rpcie_proxy_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize      = rpcie_proxy_realize;
    k->exit         = rpcie_proxy_exit;
    k->config_read  = rpcie_proxy_config_read;
    k->config_write = rpcie_proxy_config_write;
    k->vendor_id    = 0xFFFF; /* placeholder — real ID set from FN_ADD */
    k->device_id    = 0xFFFF;
    k->class_id     = PCI_CLASS_OTHERS;

    dc->desc = "Remote PCIe Proxy Device";
    dc->user_creatable = false;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo rpcie_proxy_info = {
    .name          = TYPE_REMOTE_PCIE_PROXY,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(RemotePcieProxy),
    .class_init    = rpcie_proxy_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

/* ================================================================== */
/*  Type registration                                                 */
/* ================================================================== */

static void rpcie_register_types(void)
{
    type_register_static(&rpcie_rp_info);
    type_register_static(&rpcie_proxy_info);
}

type_init(rpcie_register_types)
