/*
 * remote-pcie-port.c — Remote PCIe endpoint (TLP-over-socket bridge)
 *
 * A single PCIe endpoint whose entire state lives in an external
 * simulation (Verilator, VCS, FPGA).  QEMU is a thin pipe:
 *
 *   Guest MMIO/config → build TLP → send over socket → return response
 *   Simulation DMA    → receive TLP → execute on guest memory
 *   Simulation IRQ    → receive MSI addr+data → inject via msi_send_message
 *
 * There are no proxy devices, no dynamic device creation, and minimal
 * local state.  The simulation owns config space, capabilities, BAR
 * contents, interrupt tables — everything.  QEMU only maintains:
 *   - BAR memory region sizes (so QEMU maps the right address ranges)
 *   - A socket and completion ring (the transport)
 *
 * Usage:
 *   -device pcie-root-port,id=rp0,chassis=1 \
 *   -device remote-pcie-port,bus=rp0,socket=/tmp/rpcie.sock
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
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/dma.h"

#include "hw/remote/remote-pcie-port.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>

/* ================================================================== */
/*  Socket helpers                                                    */
/* ================================================================== */

static int rpcie_recv_full(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    size_t remaining = len;

    while (remaining > 0) {
        ssize_t n = recv(fd, p, remaining, 0);
        if (n <= 0) {
            if (n < 0 && errno == EINTR) continue;
            return -1;
        }
        p += n;
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
            if (n < 0 && errno == EINTR) continue;
            return -1;
        }
        p += n;
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
    int slot = (seq / 2) % RPCIE_CPL_RING_SIZE;

    qemu_mutex_lock(&rp->cpl_mutex);
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

    qemu_mutex_lock(&rp->cpl_mutex);
    rp->cpl_ring[slot].valid = false;
    qemu_mutex_unlock(&rp->cpl_mutex);

    if (rpcie_send_msg(rp, seq, payload, len) < 0) {
        return -1;
    }

    qemu_mutex_lock(&rp->cpl_mutex);
    int64_t deadline = qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + timeout_ms;

    while (!rp->cpl_ring[slot].valid || rp->cpl_ring[slot].seq != seq) {
        int64_t now = qemu_clock_get_ms(QEMU_CLOCK_REALTIME);
        if (now >= deadline) {
            qemu_mutex_unlock(&rp->cpl_mutex);
            error_report("rpcie: completion timeout seq %u", seq);
            return -1;
        }
        qemu_cond_timedwait(&rp->cpl_cond, &rp->cpl_mutex, deadline - now);
    }

    if (cpl_data)   *cpl_data   = rp->cpl_ring[slot].data;
    if (cpl_status) *cpl_status = rp->cpl_ring[slot].status;
    rp->cpl_ring[slot].valid = false;
    qemu_mutex_unlock(&rp->cpl_mutex);
    return 0;
}

/* ================================================================== */
/*  BAR MMIO — forward reads/writes as Memory TLPs                    */
/* ================================================================== */

typedef struct RpcieBarContext {
    RemotePciePort *rp;
    int             bar_idx;
} RpcieBarContext;

static uint64_t rpcie_bar_read(void *opaque, hwaddr addr, unsigned size)
{
    RpcieBarContext *ctx = opaque;
    RemotePciePort  *rp = ctx->rp;

    rpcie_tlp_mem_t tlp;
    memset(&tlp, 0, sizeof(tlp));
    tlp.hdr.fmt_type     = RPCIE_TLP_MRD32;
    tlp.hdr.length_dw    = (size + 3) / 4;
    tlp.hdr.requester_id = 0;
    tlp.hdr.tag          = ctx->bar_idx;  /* BAR index in tag */
    tlp.hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    /* Convention: address = (bar_idx << 28) | bar_offset */
    tlp.address          = ((uint64_t)ctx->bar_idx << 28) | (addr & 0x0FFFFFFF);

    uint32_t seq = rpcie_alloc_seq(rp);
    uint32_t data = 0xFFFFFFFF;
    uint8_t  status = RPCIE_CPL_UR;

    if (rpcie_send_and_wait(rp, seq, &tlp, sizeof(tlp),
                            &data, &status, 5000) < 0 ||
        status != RPCIE_CPL_SC) {
        return 0xFFFFFFFF;
    }

    /* Extract sub-DW bytes */
    int shift = ((int)addr & 3) * 8;
    uint64_t result = (uint64_t)data >> shift;
    if (size < 4) {
        result &= (1ULL << (size * 8)) - 1;
    }
    return result;
}

static void rpcie_bar_write(void *opaque, hwaddr addr,
                            uint64_t data, unsigned size)
{
    RpcieBarContext *ctx = opaque;
    RemotePciePort  *rp = ctx->rp;

    uint8_t buf[sizeof(rpcie_tlp_mem_t) + 4];
    rpcie_tlp_mem_t *tlp = (rpcie_tlp_mem_t *)buf;

    memset(buf, 0, sizeof(buf));
    tlp->hdr.fmt_type     = RPCIE_TLP_MWR32;
    tlp->hdr.length_dw    = 1;
    tlp->hdr.requester_id = 0;
    tlp->hdr.tag          = ctx->bar_idx;
    tlp->hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    /* Convention: address = (bar_idx << 28) | bar_offset */
    tlp->address          = ((uint64_t)ctx->bar_idx << 28) | (addr & 0x0FFFFFFF);

    uint32_t wdata = (uint32_t)(data << (((int)addr & 3) * 8));
    memcpy(buf + sizeof(rpcie_tlp_mem_t), &wdata, 4);

    uint32_t seq = rpcie_alloc_seq(rp);
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
/*  VF BAR MMIO — forward reads/writes through PF's socket            */
/* ================================================================== */

typedef struct RpcieVfBarContext {
    RemotePciePortVF *vf;
    int               bar_idx;
} RpcieVfBarContext;

static uint64_t rpcie_vf_bar_read(void *opaque, hwaddr addr, unsigned size)
{
    RpcieVfBarContext *ctx = opaque;
    PCIDevice *vf_dev = PCI_DEVICE(ctx->vf);
    PCIDevice *pf_dev = pcie_sriov_get_pf(vf_dev);
    RemotePciePort *rp = REMOTE_PCIE_PORT(pf_dev);
    uint16_t vf_num = pcie_sriov_vf_number(vf_dev);

    rpcie_tlp_mem_t tlp;
    memset(&tlp, 0, sizeof(tlp));
    tlp.hdr.fmt_type     = RPCIE_TLP_MRD32;
    tlp.hdr.length_dw    = (size + 3) / 4;
    /* Encode VF BDF as requester_id so the bridge routes to the VF */
    tlp.hdr.requester_id = RPCIE_BDF(0, 0,
        RPCIE_BDF_FN(rp->remote_bdf) + rp->sriov_vf_offset + vf_num * rp->sriov_vf_stride);
    tlp.hdr.tag          = ctx->bar_idx;
    tlp.hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    /* Convention: address = (bar_idx << 28) | bar_offset */
    tlp.address          = ((uint64_t)ctx->bar_idx << 28) | (addr & 0x0FFFFFFF);

    uint32_t seq = rpcie_alloc_seq(rp);
    uint32_t data = 0xFFFFFFFF;
    uint8_t  status = RPCIE_CPL_UR;

    if (rpcie_send_and_wait(rp, seq, &tlp, sizeof(tlp),
                            &data, &status, 5000) < 0 ||
        status != RPCIE_CPL_SC) {
        return 0xFFFFFFFF;
    }

    int shift = ((int)addr & 3) * 8;
    uint64_t result = (uint64_t)data >> shift;
    if (size < 4) {
        result &= (1ULL << (size * 8)) - 1;
    }
    return result;
}

static void rpcie_vf_bar_write(void *opaque, hwaddr addr,
                               uint64_t data, unsigned size)
{
    RpcieVfBarContext *ctx = opaque;
    PCIDevice *vf_dev = PCI_DEVICE(ctx->vf);
    PCIDevice *pf_dev = pcie_sriov_get_pf(vf_dev);
    RemotePciePort *rp = REMOTE_PCIE_PORT(pf_dev);
    uint16_t vf_num = pcie_sriov_vf_number(vf_dev);

    uint8_t buf[sizeof(rpcie_tlp_mem_t) + 4];
    rpcie_tlp_mem_t *tlp = (rpcie_tlp_mem_t *)buf;

    memset(buf, 0, sizeof(buf));
    tlp->hdr.fmt_type     = RPCIE_TLP_MWR32;
    tlp->hdr.length_dw    = 1;
    tlp->hdr.requester_id = RPCIE_BDF(0, 0,
        RPCIE_BDF_FN(rp->remote_bdf) + rp->sriov_vf_offset + vf_num * rp->sriov_vf_stride);
    tlp->hdr.tag          = ctx->bar_idx;
    tlp->hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    /* Convention: address = (bar_idx << 28) | bar_offset */
    tlp->address          = ((uint64_t)ctx->bar_idx << 28) | (addr & 0x0FFFFFFF);

    uint32_t wdata = (uint32_t)(data << (((int)addr & 3) * 8));
    memcpy(buf + sizeof(rpcie_tlp_mem_t), &wdata, 4);

    uint32_t seq = rpcie_alloc_seq(rp);
    rpcie_send_msg(rp, seq, buf, sizeof(rpcie_tlp_mem_t) + 4);
}

static const MemoryRegionOps rpcie_vf_bar_ops = {
    .read  = rpcie_vf_bar_read,
    .write = rpcie_vf_bar_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ================================================================== */
/*  Config space — forward everything to simulation                   */
/* ================================================================== */

static uint32_t rpcie_config_read(PCIDevice *d, uint32_t address, int len)
{
    /*
     * Config space is handled locally inside QEMU's PCI config buffer,
     * which was populated from the initial FN_ADD handshake data.
     * The simulation's RTL endpoint (PCIeEndpointIO) only exposes
     * BAR, DMA, and interrupt interfaces — there is no config-space
     * TLP path through CosimHarness.
     *
     * MSI-X table reads/writes go through BAR MMIO (the MSI-X table
     * lives in a BAR region), so they still reach the simulation.
     */
    return pci_default_read_config(d, address, len);
}

static void rpcie_config_write(PCIDevice *d, uint32_t address,
                               uint32_t val, int len)
{
    RemotePciePort *rp = REMOTE_PCIE_PORT(d);

    /*
     * Capture SR-IOV state BEFORE applying the config write.
     * pci_default_write_config modifies d->config in place, so we must
     * snapshot the old VFE/NumVFs state first to detect transitions.
     */
    bool old_vfe = false;
    uint16_t old_num_vfs = 0;
    if (rp->sriov_capable && d->exp.sriov_cap) {
        uint16_t sriov_cap = d->exp.sriov_cap;
        old_vfe = (pci_get_word(d->config + sriov_cap + PCI_SRIOV_CTRL)
                   & PCI_SRIOV_CTRL_VFE) != 0;
        old_num_vfs = old_vfe ?
            pci_get_word(d->config + sriov_cap + PCI_SRIOV_NUM_VF) : 0;
    }

    /*
     * Apply locally so QEMU updates BAR mappings, command register,
     * and capability structures when the guest programs them.
     * As with reads, config-space writes are not forwarded to the
     * simulation because it has no config-space TLP path.
     */
    pci_default_write_config(d, address, val, len);

    /*
     * If SR-IOV capable, check whether VF Enable state changed.
     * Note: pci_default_write_config() already called
     * pcie_sriov_config_write() internally, so do NOT call it again
     * here — a double call would re-enter consume_config().
     */
    if (rp->sriov_capable) {
        uint16_t sriov_cap = d->exp.sriov_cap;
        bool new_vfe = (pci_get_word(d->config + sriov_cap + PCI_SRIOV_CTRL)
                        & PCI_SRIOV_CTRL_VFE) != 0;
        uint16_t new_num_vfs = new_vfe ?
            pci_get_word(d->config + sriov_cap + PCI_SRIOV_NUM_VF) : 0;

        /*
         * Fix VF config-space identity when VFs become enabled.
         *
         * QEMU's pcie_sriov_pf_init() sets VF vendor/device IDs to
         * 0xFFFF (an implementation detail — the normal kernel SR-IOV
         * path never reads them).  However, bus rescans and direct
         * config reads DO check vendor ID, seeing 0xFFFF as "no device".
         *
         * Per PCIe SR-IOV spec §3.4.1.1, VFs present the PF's Vendor ID
         * and the VF Device ID from the SR-IOV capability.  Restore these
         * when VFs are enabled so bus scans discover them correctly.
         */
        if (new_num_vfs > 0 && (!old_vfe || old_num_vfs != new_num_vfs)) {
            uint16_t pf_vendor = pci_get_word(d->config + PCI_VENDOR_ID);
            uint16_t vf_devid  = pci_get_word(
                d->config + sriov_cap + PCI_SRIOV_VF_DID);

            for (int i = 0; i < new_num_vfs; i++) {
                PCIDevice *vf = d->exp.sriov_pf.vf[i];
                pci_config_set_vendor_id(vf->config, pf_vendor);
                pci_config_set_device_id(vf->config, vf_devid);
            }
        }

        /*
         * Detect VF enable state change → notify simulation.
         * Also fix VF vendor/device IDs so bus scan finds them.
         */
        if (old_vfe != new_vfe || old_num_vfs != new_num_vfs) {
            rpcie_ctrl_sriov_event_t ev = {
                .msg_type  = RPCIE_CTRL_SRIOV_EVENT,
                .pf_index  = 0,
                .vf_enable = new_vfe ? 1 : 0,
                .reserved  = 0,
                .num_vfs   = new_num_vfs,
                .reserved2 = 0,
            };
            rpcie_send_msg(rp, rpcie_alloc_seq(rp), &ev, sizeof(ev));
            info_report("rpcie: SR-IOV event → vf_enable=%d num_vfs=%d",
                        ev.vf_enable, new_num_vfs);
        }
    }
}

/* ================================================================== */
/*  RX handlers                                                       */
/* ================================================================== */

static void rpcie_rx_handle_cpl(RemotePciePort *rp, uint32_t seq,
                                const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_cpl_t)) return;

    const rpcie_tlp_cpl_t *cpl = (const rpcie_tlp_cpl_t *)payload;
    uint8_t  status = RPCIE_CPL_STATUS(cpl->status_bcm_bc);
    uint32_t data = 0;

    if (cpl->hdr.fmt_type == RPCIE_TLP_CPLD &&
        len >= sizeof(rpcie_tlp_cpl_t) + 4) {
        memcpy(&data, payload + sizeof(rpcie_tlp_cpl_t), 4);
    }

    rpcie_deliver_completion(rp, seq, status, data);
}

static void rpcie_rx_handle_dma_write(RemotePciePort *rp,
                                      const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_mem_t)) return;

    const rpcie_tlp_mem_t *tlp = (const rpcie_tlp_mem_t *)payload;
    uint16_t ndw = tlp->hdr.length_dw ? tlp->hdr.length_dw : 1024;
    uint32_t nbytes = ndw * 4;

    if (len < sizeof(rpcie_tlp_mem_t) + nbytes) return;

    pci_dma_write(PCI_DEVICE(rp), tlp->address,
                  payload + sizeof(rpcie_tlp_mem_t), nbytes);
}

static void rpcie_rx_handle_dma_read(RemotePciePort *rp, uint32_t seq,
                                     const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_mem_t)) return;

    const rpcie_tlp_mem_t *tlp = (const rpcie_tlp_mem_t *)payload;
    uint16_t ndw = tlp->hdr.length_dw ? tlp->hdr.length_dw : 1024;
    uint32_t nbytes = ndw * 4;
    if (nbytes > RPCIE_MAX_PAYLOAD) {
        nbytes = RPCIE_MAX_PAYLOAD;
        ndw = nbytes / 4;
    }

    uint8_t buf[sizeof(rpcie_tlp_cpl_t) + RPCIE_MAX_PAYLOAD];
    rpcie_tlp_cpl_t *cpl = (rpcie_tlp_cpl_t *)buf;

    rpcie_build_cpld(cpl, 0, tlp->hdr.requester_id, tlp->hdr.tag,
                     RPCIE_CPL_SC, nbytes,
                     (uint8_t)(tlp->address & 0x7F), ndw);

    pci_dma_read(PCI_DEVICE(rp), tlp->address,
                 buf + sizeof(rpcie_tlp_cpl_t), nbytes);

    rpcie_send_msg(rp, seq, buf, sizeof(rpcie_tlp_cpl_t) + nbytes);
}

static void rpcie_rx_handle_interrupt(RemotePciePort *rp,
                                      const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_msg_t)) return;

    const rpcie_tlp_msg_t *msg = (const rpcie_tlp_msg_t *)payload;

    if (msg->msg_code == RPCIE_MSG_INTX_ASSERT) {
        pci_set_irq(PCI_DEVICE(rp), 1);
        return;
    }
    if (msg->msg_code == RPCIE_MSG_INTX_DEASSERT) {
        pci_set_irq(PCI_DEVICE(rp), 0);
        return;
    }

    /*
     * MSI / MSI-X: the simulation sends the raw MSI address + data.
     * We inject directly via msi_send_message — no local MSI-X table
     * or capability init needed.  The simulation owns all of that.
     */
    if (msg->msg_code == RPCIE_MSG_MSIX_NOTIFY ||
        msg->msg_code == RPCIE_MSG_MSI_NOTIFY) {
        uint32_t msi_data = 0;
        if (len >= sizeof(rpcie_tlp_msg_t) + 4) {
            memcpy(&msi_data, payload + sizeof(rpcie_tlp_msg_t), 4);
        }
        MSIMessage msi_msg = {
            .address = msg->msg_addr,
            .data    = msi_data,
        };
        msi_send_message(PCI_DEVICE(rp), msi_msg);
    }
}

/* ================================================================== */
/*  RX thread                                                         */
/* ================================================================== */

static void *rpcie_rx_thread(void *opaque)
{
    RemotePciePort *rp = opaque;
    rpcie_frame_hdr_t hdr;
    uint8_t buf[RPCIE_MAX_PAYLOAD + 256];

    while (rp->rx_running) {
        struct pollfd pfd = { .fd = rp->conn_fd, .events = POLLIN };
        if (poll(&pfd, 1, 100) <= 0) continue;

        if (rpcie_recv_full(rp->conn_fd, &hdr, sizeof(hdr)) < 0) {
            error_report("rpcie: connection closed");
            rp->rx_running = false;
            break;
        }
        if (hdr.magic != RPCIE_MAGIC || hdr.length > sizeof(buf)) {
            error_report("rpcie: bad frame (magic=0x%x len=%u)",
                         hdr.magic, hdr.length);
            rp->rx_running = false;
            break;
        }
        if (hdr.length > 0 &&
            rpcie_recv_full(rp->conn_fd, buf, hdr.length) < 0) {
            rp->rx_running = false;
            break;
        }
        if (hdr.length == 0) continue;

        uint8_t ft = buf[0];

        if (RPCIE_IS_CTRL(ft)) {
            switch (ft) {
            case RPCIE_CTRL_LINK_DOWN:
                info_report("rpcie: simulation sent LINK_DOWN");
                rp->rx_running = false;
                break;
            case RPCIE_CTRL_RESET:
                info_report("rpcie: reset from simulation");
                break;
            default:
                break;
            }
        } else if (RPCIE_TLP_IS_CPL(ft)) {
            rpcie_rx_handle_cpl(rp, hdr.seq, buf, hdr.length);
        } else if (RPCIE_TLP_IS_MEM_WRITE(ft)) {
            rpcie_rx_handle_dma_write(rp, buf, hdr.length);
        } else if (RPCIE_TLP_IS_MEM_READ(ft)) {
            rpcie_rx_handle_dma_read(rp, hdr.seq, buf, hdr.length);
        } else if ((ft == RPCIE_TLP_MSGD || ft == RPCIE_TLP_MSG) &&
                   hdr.length >= sizeof(rpcie_tlp_msg_t)) {
            rpcie_rx_handle_interrupt(rp, buf, hdr.length);
        }
    }
    return NULL;
}

/* ================================================================== */
/*  Handshake + initial FN_ADD                                        */
/* ================================================================== */

static int rpcie_handshake(RemotePciePort *rp)
{
    /*
     * The simulation (C bridge) is the server and initiates the handshake
     * by sending RPCIE_CTRL_HANDSHAKE.  We (QEMU, the client) receive it
     * and reply with RPCIE_CTRL_HANDSHAKE_ACK.
     */

    /* ---- Step 1: receive server's HANDSHAKE ---- */
    rpcie_frame_hdr_t hdr;
    if (rpcie_recv_full(rp->conn_fd, &hdr, sizeof(hdr)) < 0 ||
        hdr.magic != RPCIE_MAGIC) {
        error_report("rpcie: handshake recv failed");
        return -1;
    }

    rpcie_ctrl_handshake_t incoming;
    if (hdr.length < sizeof(incoming) ||
        rpcie_recv_full(rp->conn_fd, &incoming, sizeof(incoming)) < 0) {
        error_report("rpcie: handshake truncated");
        return -1;
    }

    /* Drain extra payload bytes (forward compat) */
    if (hdr.length > sizeof(incoming)) {
        uint8_t drain[256];
        uint32_t extra = hdr.length - sizeof(incoming);
        while (extra > 0) {
            uint32_t n = extra > sizeof(drain) ? sizeof(drain) : extra;
            rpcie_recv_full(rp->conn_fd, drain, n);
            extra -= n;
        }
    }

    if (incoming.msg_type != RPCIE_CTRL_HANDSHAKE ||
        incoming.version != RPCIE_PROTOCOL_VERSION) {
        error_report("rpcie: handshake unexpected (type=0x%x ver=%d)",
                     incoming.msg_type, incoming.version);
        return -1;
    }

    /* ---- Step 2: send HANDSHAKE_ACK ---- */
    rpcie_ctrl_handshake_t ack = {
        .msg_type     = RPCIE_CTRL_HANDSHAKE_ACK,
        .version      = RPCIE_PROTOCOL_VERSION,
        .capabilities = RPCIE_CAP_SRIOV | RPCIE_CAP_AER |
                        RPCIE_CAP_SHM | RPCIE_CAP_FLR |
                        RPCIE_CAP_HOTPLUG,
    };

    if (rpcie_send_msg(rp, rpcie_alloc_seq(rp), &ack, sizeof(ack)) < 0) {
        error_report("rpcie: handshake ack send failed");
        return -1;
    }

    rp->remote_caps = incoming.capabilities & ack.capabilities;
    info_report("rpcie: handshake OK, caps 0x%08x", rp->remote_caps);
    return 0;
}

/*
 * Block until we receive the initial FN_ADD from the simulation.
 * This tells us the device identity and BAR layout so we can
 * configure QEMU's PCI config space and register memory regions.
 */
static int rpcie_recv_initial_fn_add(RemotePciePort *rp, Error **errp)
{
    /* Poll with 60s timeout */
    struct pollfd pfd = { .fd = rp->conn_fd, .events = POLLIN };
    int pr = poll(&pfd, 1, 60000);
    if (pr <= 0) {
        error_setg(errp, "rpcie: timeout waiting for FN_ADD");
        return -1;
    }

    rpcie_frame_hdr_t hdr;
    if (rpcie_recv_full(rp->conn_fd, &hdr, sizeof(hdr)) < 0 ||
        hdr.magic != RPCIE_MAGIC) {
        error_setg(errp, "rpcie: bad frame waiting for FN_ADD");
        return -1;
    }

    uint8_t buf[1024];
    if (hdr.length > sizeof(buf)) {
        error_setg(errp, "rpcie: FN_ADD too large (%u)", hdr.length);
        return -1;
    }
    if (hdr.length > 0 &&
        rpcie_recv_full(rp->conn_fd, buf, hdr.length) < 0) {
        error_setg(errp, "rpcie: FN_ADD recv failed");
        return -1;
    }
    if (hdr.length < sizeof(rpcie_ctrl_fn_add_t) ||
        buf[0] != RPCIE_CTRL_FN_ADD) {
        error_setg(errp, "rpcie: expected FN_ADD, got 0x%02x", buf[0]);
        return -1;
    }

    rpcie_ctrl_fn_add_t fn;
    memcpy(&fn, buf, sizeof(fn));

    PCIDevice *d = PCI_DEVICE(rp);
    rp->remote_bdf = fn.bdf;

    /* Set PCI identity in local config space */
    pci_config_set_vendor_id(d->config, fn.vendor_id);
    pci_config_set_device_id(d->config, fn.device_id);
    pci_set_word(d->config + PCI_SUBSYSTEM_VENDOR_ID, fn.subsys_vendor_id);
    pci_set_word(d->config + PCI_SUBSYSTEM_ID, fn.subsys_id);
    pci_config_set_class(d->config,
                         ((uint16_t)fn.class_code[2] << 8) | fn.class_code[1]);
    pci_config_set_prog_interface(d->config, fn.class_code[0]);
    pci_set_byte(d->config + PCI_REVISION_ID, fn.revision);

    /* Parse BAR descriptors */
    const uint8_t *p = buf + sizeof(rpcie_ctrl_fn_add_t);
    uint32_t remaining = hdr.length - sizeof(rpcie_ctrl_fn_add_t);
    int nb = fn.num_bars;
    if (nb > RPCIE_MAX_BARS) nb = RPCIE_MAX_BARS;
    rp->num_bars = nb;

    for (int i = 0; i < nb && remaining >= sizeof(rpcie_bar_desc_t); i++) {
        rpcie_bar_desc_t bar;
        memcpy(&bar, p, sizeof(bar));
        p += sizeof(bar);
        remaining -= sizeof(bar);

        if (bar.type == RPCIE_BAR_DISABLED || bar.size == 0) continue;

        rp->bar_size[i] = bar.size;
        rp->bar_type[i] = bar.type;

        RpcieBarContext *ctx = g_new0(RpcieBarContext, 1);
        ctx->rp      = rp;
        ctx->bar_idx = i;

        char name[32];
        snprintf(name, sizeof(name), "rpcie-bar%d", i);
        memory_region_init_io(&rp->bar_mr[i], OBJECT(rp),
                              &rpcie_bar_ops, ctx, name, bar.size);

        int pci_type = PCI_BASE_ADDRESS_SPACE_MEMORY;
        if (bar.type == RPCIE_BAR_MEM64) {
            pci_type |= PCI_BASE_ADDRESS_MEM_TYPE_64;
        }
        if (bar.type == RPCIE_BAR_IO) {
            pci_type = PCI_BASE_ADDRESS_SPACE_IO;
        }
        if (bar.prefetchable) {
            pci_type |= PCI_BASE_ADDRESS_MEM_PREFETCH;
        }

        pci_register_bar(d, i, pci_type, &rp->bar_mr[i]);

        if (bar.type == RPCIE_BAR_MEM64) i++;  /* skip next index */
    }

    /* ---- PCIe Endpoint Capability (required for extended capabilities) ---- */
    if (pcie_endpoint_cap_init(d, 0) < 0) {
        error_setg(errp, "rpcie: failed to init PCIe endpoint cap");
        return -1;
    }

    /* ---- Parse MSI-X info ---- */
    if (fn.has_msix && remaining >= sizeof(rpcie_msix_info_t)) {
        rpcie_msix_info_t msix;
        memcpy(&msix, p, sizeof(msix));
        p += sizeof(msix);
        remaining -= sizeof(msix);

        /*
         * Add MSI-X capability to config space manually (without
         * msix_init) so the guest can discover it.  We do NOT use
         * QEMU's msix_init because that intercepts BAR MMIO for the
         * MSI-X table region — we want those accesses forwarded to
         * the simulation's RTL, which owns the actual MSI-X table.
         *
         * Interrupt injection uses msi_send_message(addr, data) with
         * raw addr+data from the simulation, bypassing QEMU's MSI-X
         * table entirely.
         */
        int cap_pos = pci_add_capability(d, PCI_CAP_ID_MSIX, 0,
                                         MSIX_CAP_LENGTH, errp);
        if (cap_pos >= 0) {
            /* Message Control: table size - 1 in bits [10:0] */
            pci_set_word(d->config + cap_pos + PCI_MSIX_FLAGS,
                         (msix.table_size - 1) & PCI_MSIX_FLAGS_QSIZE);
            /* Make enable (bit 15) and function mask (bit 14) writable */
            pci_set_word(d->wmask + cap_pos + PCI_MSIX_FLAGS,
                         PCI_MSIX_FLAGS_ENABLE | PCI_MSIX_FLAGS_MASKALL);

            /* Table BIR + offset */
            pci_set_long(d->config + cap_pos + PCI_MSIX_TABLE,
                         (msix.table_offset & ~0x7u) |
                         (msix.table_bar & 0x7));
            /* PBA BIR + offset */
            pci_set_long(d->config + cap_pos + PCI_MSIX_PBA,
                         (msix.pba_offset & ~0x7u) |
                         (msix.pba_bar & 0x7));
        }
    }

    /* ---- Parse MSI info ---- */
    if (fn.has_msi && remaining >= sizeof(rpcie_msi_info_t)) {
        rpcie_msi_info_t msi_info;
        memcpy(&msi_info, p, sizeof(msi_info));
        p += sizeof(msi_info);
        remaining -= sizeof(msi_info);

        /*
         * Set up MSI capability.  As with MSI-X, interrupt delivery
         * uses msi_send_message and does not depend on QEMU's MSI
         * infrastructure.  We add the capability so the guest can
         * discover and enable MSI if desired.
         */
        int msi_flags = PCI_MSI_FLAGS_64BIT * msi_info.is_64bit;
        if (msi_init(d, 0, msi_info.num_vectors,
                     msi_info.is_64bit, msi_info.per_vector_mask,
                     errp) < 0) {
            info_report("rpcie: msi_init failed (non-fatal), MSI won't be available");
            /* Non-fatal — device works without MSI */
            (void)msi_flags;
        }
    }

    /* ---- Parse SR-IOV info ---- */
    if (fn.has_sriov && remaining >= sizeof(rpcie_sriov_info_t)) {
        rpcie_sriov_info_t sriov;
        memcpy(&sriov, p, sizeof(sriov));
        p += sizeof(sriov);
        remaining -= sizeof(sriov);

        rp->sriov_capable      = true;
        rp->sriov_total_vfs    = sriov.total_vfs;
        rp->sriov_vf_device_id = sriov.vf_device_id;
        rp->sriov_vf_offset    = sriov.vf_offset;
        rp->sriov_vf_stride    = sriov.vf_stride;
        rp->sriov_num_vf_bars  = sriov.num_vf_bars;

        /* Parse VF BAR descriptors */
        for (int i = 0; i < sriov.num_vf_bars && i < RPCIE_MAX_BARS
             && remaining >= sizeof(rpcie_bar_desc_t); i++) {
            rpcie_bar_desc_t vf_bar;
            memcpy(&vf_bar, p, sizeof(vf_bar));
            p += sizeof(vf_bar);
            remaining -= sizeof(vf_bar);
            rp->sriov_vf_bar_size[i] = vf_bar.size;
            rp->sriov_vf_bar_type[i] = vf_bar.type;
        }

        /* Initialize SR-IOV Extended Capability via QEMU's API.
         *
         * The SR-IOV extended capability MUST be placed at the head
         * of the extended capability chain (offset 0x100) when no
         * other extended capabilities precede it.  QEMU's
         * pcie_add_capability asserts that a valid chain exists if
         * the offset is not PCI_CONFIG_SPACE_SIZE (0x100).
         */
        if (!pcie_sriov_pf_init(d, PCI_CONFIG_SPACE_SIZE,
                                TYPE_REMOTE_PCIE_PORT_VF,
                                sriov.vf_device_id,
                                sriov.total_vfs,   /* init_vfs */
                                sriov.total_vfs,   /* total_vfs */
                                sriov.vf_offset,
                                sriov.vf_stride,
                                errp)) {
            info_report("rpcie: pcie_sriov_pf_init failed (non-fatal)");
            rp->sriov_capable = false;
        } else {
            /* Register VF BARs with the SR-IOV subsystem */
            for (int i = 0; i < sriov.num_vf_bars && i < RPCIE_MAX_BARS; i++) {
                if (rp->sriov_vf_bar_size[i] == 0) continue;

                int pci_type = PCI_BASE_ADDRESS_SPACE_MEMORY;
                if (rp->sriov_vf_bar_type[i] == RPCIE_BAR_MEM64) {
                    pci_type |= PCI_BASE_ADDRESS_MEM_TYPE_64;
                }
                pcie_sriov_pf_init_vf_bar(d, i, pci_type,
                                          rp->sriov_vf_bar_size[i]);
            }

            if (sriov.supported_page_sizes) {
                pcie_sriov_pf_add_sup_pgsize(d, sriov.supported_page_sizes);
            }

            /*
             * Set multifunction bit on the PF at init time.
             *
             * QEMU's pci_init_multifunction() skips SR-IOV VFs, so
             * the PF's Header Type never gets the multifunction flag.
             * The Linux kernel caches dev->multifunction at first
             * discovery and never re-reads it.  If the PF is initially
             * seen as single-function, bus rescans skip functions 1-7
             * and VFs are never found.
             *
             * By setting multifunction unconditionally when SR-IOV is
             * configured, the kernel will probe functions 1-7 during
             * initial scan (finding nothing, since VFs start disabled)
             * AND during later rescans after VF Enable is set.
             */
            d->config[PCI_HEADER_TYPE] |= PCI_HEADER_TYPE_MULTI_FUNCTION;

            info_report("rpcie: SR-IOV enabled, total_vfs=%d vf_dev=0x%04x",
                        sriov.total_vfs, sriov.vf_device_id);
        }
    }

    info_report("rpcie: device %04x:%04x, %d BARs",
                fn.vendor_id, fn.device_id, nb);
    return 0;
}

/* ================================================================== */
/*  Socket setup                                                      */
/* ================================================================== */

static int rpcie_setup_socket(RemotePciePort *rp, Error **errp)
{
    struct sockaddr_un addr;

    if (!rp->socket_path || !rp->socket_path[0]) {
        error_setg(errp, "rpcie: 'socket' property required");
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, rp->socket_path, sizeof(addr.sun_path) - 1);

    if (rp->server) {
        rp->listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (rp->listen_fd < 0) {
            error_setg_errno(errp, errno, "rpcie: socket()");
            return -1;
        }
        unlink(rp->socket_path);
        if (bind(rp->listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            error_setg_errno(errp, errno, "rpcie: bind(%s)", rp->socket_path);
            goto fail_listen;
        }
        if (listen(rp->listen_fd, 1) < 0) {
            error_setg_errno(errp, errno, "rpcie: listen()");
            goto fail_listen;
        }

        info_report("rpcie: listening on %s", rp->socket_path);

        struct pollfd pfd = { .fd = rp->listen_fd, .events = POLLIN };
        if (poll(&pfd, 1, 60000) <= 0) {
            error_setg(errp, "rpcie: accept timeout");
            goto fail_listen;
        }
        rp->conn_fd = accept(rp->listen_fd, NULL, NULL);
        if (rp->conn_fd < 0) {
            error_setg_errno(errp, errno, "rpcie: accept()");
            goto fail_listen;
        }
        info_report("rpcie: connected");
    } else {
        rp->conn_fd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (rp->conn_fd < 0) {
            error_setg_errno(errp, errno, "rpcie: socket()");
            return -1;
        }
        for (int i = 0; i < 100; i++) {
            if (connect(rp->conn_fd, (struct sockaddr *)&addr,
                        sizeof(addr)) == 0) {
                info_report("rpcie: connected to %s", rp->socket_path);
                return 0;
            }
            if (errno != ENOENT && errno != ECONNREFUSED) break;
            usleep(100000);
        }
        error_setg(errp, "rpcie: connect(%s) failed", rp->socket_path);
        close(rp->conn_fd);
        rp->conn_fd = -1;
        return -1;
    }
    return 0;

fail_listen:
    close(rp->listen_fd);
    rp->listen_fd = -1;
    return -1;
}

/* ================================================================== */
/*  Realize / unrealize                                               */
/* ================================================================== */

static void rpcie_realize(PCIDevice *d, Error **errp)
{
    RemotePciePort *rp = REMOTE_PCIE_PORT(d);

    qemu_mutex_init(&rp->send_mutex);
    qemu_mutex_init(&rp->cpl_mutex);
    qemu_cond_init(&rp->cpl_cond);
    memset(rp->cpl_ring, 0, sizeof(rp->cpl_ring));
    rp->next_seq  = 1;
    rp->listen_fd = -1;
    rp->conn_fd   = -1;

    if (rpcie_setup_socket(rp, errp) < 0) return;
    if (rpcie_handshake(rp) < 0) {
        error_setg(errp, "rpcie: handshake failed");
        goto fail;
    }

    /* Tell simulation the link is up */
    rpcie_ctrl_link_t link = { .msg_type = RPCIE_CTRL_LINK_UP };
    rpcie_send_msg(rp, rpcie_alloc_seq(rp), &link, sizeof(link));

    /* Wait for FN_ADD — configures identity + BARs */
    if (rpcie_recv_initial_fn_add(rp, errp) < 0) {
        goto fail;
    }

    /* Start RX thread */
    rp->rx_running = true;
    qemu_thread_create(&rp->rx_thread, "rpcie-rx",
                       rpcie_rx_thread, rp,
                       QEMU_THREAD_JOINABLE);


    info_report("rpcie: endpoint ready");
    return;

fail:
    if (rp->conn_fd >= 0) { close(rp->conn_fd); rp->conn_fd = -1; }
    if (rp->listen_fd >= 0) { close(rp->listen_fd); rp->listen_fd = -1; }
}

static void rpcie_exit(PCIDevice *d)
{
    RemotePciePort *rp = REMOTE_PCIE_PORT(d);

    rp->rx_running = false;
    if (rp->conn_fd >= 0) {
        rpcie_ctrl_link_t link = { .msg_type = RPCIE_CTRL_LINK_DOWN };
        rpcie_send_msg(rp, rpcie_alloc_seq(rp), &link, sizeof(link));
        shutdown(rp->conn_fd, SHUT_RDWR);
    }
    qemu_thread_join(&rp->rx_thread);

    if (rp->sriov_capable) {
        pcie_sriov_pf_exit(d);
    }

    pcie_cap_exit(d);

    for (int i = 0; i < rp->num_bars; i++) {
        if (rp->bar_mr[i].size) {
            g_free(rp->bar_mr[i].opaque);
        }
    }

    if (rp->conn_fd >= 0) { close(rp->conn_fd); rp->conn_fd = -1; }
    if (rp->listen_fd >= 0) {
        close(rp->listen_fd); rp->listen_fd = -1;
        if (rp->socket_path) unlink(rp->socket_path);
    }

    qemu_mutex_destroy(&rp->send_mutex);
    qemu_mutex_destroy(&rp->cpl_mutex);
    qemu_cond_destroy(&rp->cpl_cond);
}

/* ================================================================== */
/*  Reset                                                             */
/* ================================================================== */

static void (*rpcie_parent_reset_hold)(Object *obj, ResetType type);

static void rpcie_reset_hold(Object *obj, ResetType type)
{
    if (rpcie_parent_reset_hold) {
        rpcie_parent_reset_hold(obj, type);
    }

    RemotePciePort *rp = REMOTE_PCIE_PORT(obj);

    if (rp->sriov_capable) {
        pcie_sriov_pf_reset(PCI_DEVICE(rp));
    }

    if (rp->conn_fd >= 0 && rp->rx_running) {
        rpcie_ctrl_reset_t msg = {
            .msg_type   = RPCIE_CTRL_RESET,
            .reset_type = RPCIE_RESET_HOT,
            .target_bdf = rp->remote_bdf,
        };
        rpcie_send_msg(rp, rpcie_alloc_seq(rp), &msg, sizeof(msg));
    }
}

/* ================================================================== */
/*  Class init / type registration                                    */
/* ================================================================== */

static const Property rpcie_props[] = {
    DEFINE_PROP_STRING("socket", RemotePciePort, socket_path),
    DEFINE_PROP_BOOL("server", RemotePciePort, server, false),
};

static const VMStateDescription vmstate_rpcie = {
    .name = "remote-pcie-port",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, RemotePciePort),
        VMSTATE_END_OF_LIST()
    }
};

static void rpcie_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    k->realize      = rpcie_realize;
    k->exit         = rpcie_exit;
    k->config_read  = rpcie_config_read;
    k->config_write = rpcie_config_write;
    k->vendor_id    = PCI_VENDOR_ID_REDHAT;
    k->device_id    = 0x0013;
    k->class_id     = PCI_CLASS_OTHERS;

    dc->desc = "Remote PCIe Endpoint (TLP-over-socket bridge)";
    dc->vmsd = &vmstate_rpcie;
    device_class_set_props(dc, rpcie_props);

    rpcie_parent_reset_hold = rc->phases.hold;
    rc->phases.hold = rpcie_reset_hold;
}

static const TypeInfo rpcie_info = {
    .name          = TYPE_REMOTE_PCIE_PORT,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(RemotePciePort),
    .class_init    = rpcie_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

/* ================================================================== */
/*  VF companion device                                               */
/* ================================================================== */

static void rpcie_vf_realize(PCIDevice *d, Error **errp)
{
    RemotePciePortVF *vf = REMOTE_PCIE_PORT_VF(d);
    PCIDevice *pf_dev = pcie_sriov_get_pf(d);
    RemotePciePort *rp = REMOTE_PCIE_PORT(pf_dev);

    /* Register VF BARs that forward MMIO through the PF's socket */
    for (int i = 0; i < rp->sriov_num_vf_bars && i < RPCIE_MAX_BARS; i++) {
        if (rp->sriov_vf_bar_size[i] == 0) continue;

        RpcieVfBarContext *ctx = g_new0(RpcieVfBarContext, 1);
        ctx->vf      = vf;
        ctx->bar_idx = i;

        char name[32];
        snprintf(name, sizeof(name), "rpcie-vf-bar%d", i);
        memory_region_init_io(&vf->bar_mr[i], OBJECT(vf),
                              &rpcie_vf_bar_ops, ctx, name,
                              rp->sriov_vf_bar_size[i]);

        int pci_type = PCI_BASE_ADDRESS_SPACE_MEMORY;
        if (rp->sriov_vf_bar_type[i] == RPCIE_BAR_MEM64) {
            pci_type |= PCI_BASE_ADDRESS_MEM_TYPE_64;
        }
        pci_register_bar(d, i, pci_type, &vf->bar_mr[i]);

        if (rp->sriov_vf_bar_type[i] == RPCIE_BAR_MEM64) i++;
    }

    if (pcie_endpoint_cap_init(d, 0) < 0) {
        error_setg(errp, "rpcie-vf: failed to init PCIe cap");
    }
}

static void rpcie_vf_exit(PCIDevice *d)
{
    RemotePciePortVF *vf = REMOTE_PCIE_PORT_VF(d);

    for (int i = 0; i < RPCIE_MAX_BARS; i++) {
        if (vf->bar_mr[i].size) {
            g_free(vf->bar_mr[i].opaque);
        }
    }
    pcie_cap_exit(d);
}

static void rpcie_vf_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize   = rpcie_vf_realize;
    k->exit      = rpcie_vf_exit;
    k->vendor_id = PCI_VENDOR_ID_REDHAT;
    k->device_id = 0x0014;  /* overridden by pcie_sriov from PF's SR-IOV cap */
    k->class_id  = PCI_CLASS_OTHERS;

    dc->desc = "Remote PCIe Endpoint VF (auto-created by SR-IOV)";
    dc->user_creatable = false;
}

static const TypeInfo rpcie_vf_info = {
    .name          = TYPE_REMOTE_PCIE_PORT_VF,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(RemotePciePortVF),
    .class_init    = rpcie_vf_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void rpcie_register_types(void)
{
    type_register_static(&rpcie_info);
    type_register_static(&rpcie_vf_info);
}

type_init(rpcie_register_types)
