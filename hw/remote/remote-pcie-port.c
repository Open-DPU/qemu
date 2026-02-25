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
#include "hw/core/boards.h"
#include "system/memory.h"
#include "hw/i386/x86.h"

#include "hw/remote/remote-pcie-port.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

static void rpcie_rebuild_ari_chain(RemotePciePort *rp);
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

/*
 * Send a file descriptor via SCM_RIGHTS.
 * The receiver (C bridge) does a separate recvmsg() with 1 dummy byte.
 */
static int rpcie_send_fd(int sock_fd, int fd)
{
    struct msghdr msg = {0};
    struct iovec iov;
    char dummy = 0;

    iov.iov_base = &dummy;
    iov.iov_len  = 1;
    msg.msg_iov    = &iov;
    msg.msg_iovlen = 1;

    union {
        struct cmsghdr align;
        char buf[CMSG_SPACE(sizeof(int))];
    } cmsg_buf;

    msg.msg_control    = cmsg_buf.buf;
    msg.msg_controllen = sizeof(cmsg_buf.buf);

    struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type  = SCM_RIGHTS;
    cmsg->cmsg_len   = CMSG_LEN(sizeof(int));
    memcpy(CMSG_DATA(cmsg), &fd, sizeof(int));

    ssize_t n = sendmsg(sock_fd, &msg, MSG_NOSIGNAL);
    return (n >= 0) ? 0 : -1;
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
    uint16_t        remote_bdf;  /* function BDF for requester_id */
} RpcieBarContext;

static uint64_t rpcie_bar_read(void *opaque, hwaddr addr, unsigned size)
{
    RpcieBarContext *ctx = opaque;
    RemotePciePort  *rp = ctx->rp;

    rpcie_tlp_mem_t tlp;
    memset(&tlp, 0, sizeof(tlp));
    tlp.hdr.fmt_type     = RPCIE_TLP_MRD32;
    tlp.hdr.length_dw    = (size + 3) / 4;
    tlp.hdr.requester_id = ctx->remote_bdf;
    tlp.hdr.tag          = ctx->bar_idx;  /* BAR index in tag */
    tlp.hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    /* Convention: address = (bar_idx << 28) | bar_offset */
    tlp.address          = ((uint64_t)ctx->bar_idx << 28) | (addr & 0x0FFFFFFF);

    uint32_t seq = rpcie_alloc_seq(rp);
    uint32_t data = 0xFFFFFFFF;
    uint8_t  status = RPCIE_CPL_UR;

    if (rpcie_send_and_wait(rp, seq, &tlp, sizeof(tlp),
                            &data, &status, 30000) < 0 ||
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
    tlp->hdr.requester_id = ctx->remote_bdf;
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
    RemotePciePortVF *vf = ctx->vf;
    PCIDevice *vf_dev = PCI_DEVICE(vf);
    RemotePciePort *rp = vf->rp;
    uint16_t vf_num = pcie_sriov_vf_number(vf_dev);

    rpcie_tlp_mem_t tlp;
    memset(&tlp, 0, sizeof(tlp));
    tlp.hdr.fmt_type     = RPCIE_TLP_MRD32;
    tlp.hdr.length_dw    = (size + 3) / 4;
    /* Encode VF BDF as requester_id so the bridge routes to the VF */
    tlp.hdr.requester_id = RPCIE_BDF(0, 0,
        RPCIE_BDF_FN(vf->pf_remote_bdf) + vf->pf_vf_offset + vf_num * vf->pf_vf_stride);
    tlp.hdr.tag          = ctx->bar_idx;
    tlp.hdr.byte_enables = rpcie_compute_first_be((uint32_t)addr, size);
    /* Convention: address = (bar_idx << 28) | bar_offset */
    tlp.address          = ((uint64_t)ctx->bar_idx << 28) | (addr & 0x0FFFFFFF);

    uint32_t seq = rpcie_alloc_seq(rp);
    uint32_t data = 0xFFFFFFFF;
    uint8_t  status = RPCIE_CPL_UR;

    if (rpcie_send_and_wait(rp, seq, &tlp, sizeof(tlp),
                            &data, &status, 30000) < 0 ||
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
    RemotePciePortVF *vf = ctx->vf;
    PCIDevice *vf_dev = PCI_DEVICE(vf);
    RemotePciePort *rp = vf->rp;
    uint16_t vf_num = pcie_sriov_vf_number(vf_dev);

    uint8_t buf[sizeof(rpcie_tlp_mem_t) + 4];
    rpcie_tlp_mem_t *tlp = (rpcie_tlp_mem_t *)buf;

    memset(buf, 0, sizeof(buf));
    tlp->hdr.fmt_type     = RPCIE_TLP_MWR32;
    tlp->hdr.length_dw    = 1;
    tlp->hdr.requester_id = RPCIE_BDF(0, 0,
        RPCIE_BDF_FN(vf->pf_remote_bdf) + vf->pf_vf_offset + vf_num * vf->pf_vf_stride);
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
     * FLR (Function Level Reset) interception.
     * pcie_cap_flr_init() made BCR_FLR writable.  After
     * pci_default_write_config() applies the write, check whether the
     * guest set the FLR initiate bit.  If so, forward to the simulation
     * and clear the bit.  We intentionally do NOT call pci_device_reset()
     * here because the simulation owns all device state — it will apply
     * the reset on its side and we'll see the effects through the normal
     * data path.
     */
    if (d->exp.exp_cap) {
        uint8_t *devctl = d->config + d->exp.exp_cap + PCI_EXP_DEVCTL;
        if (pci_get_word(devctl) & PCI_EXP_DEVCTL_BCR_FLR) {
            pci_word_test_and_clear_mask(devctl, PCI_EXP_DEVCTL_BCR_FLR);

            if (rp->conn_fd >= 0 && qatomic_read(&rp->rx_running)) {
                rpcie_ctrl_reset_t msg = {
                    .msg_type   = RPCIE_CTRL_RESET,
                    .reset_type = RPCIE_RESET_FLR,
                    .target_bdf = rp->remote_bdf,
                };
                rpcie_send_msg(rp, rpcie_alloc_seq(rp), &msg, sizeof(msg));
                info_report("rpcie: FLR initiated for BDF 0x%04x",
                            rp->remote_bdf);
            }
        }
    }

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

            /* Rebuild ARI chain to include/exclude this PF's VFs */
            rpcie_rebuild_ari_chain(rp);
        }
    }
}

/* ================================================================== */
/*  RX handlers                                                       */
/* ================================================================== */

/*
 * Resolve a BDF to the correct PCIDevice* for DMA and interrupt dispatch.
 *
 * The protocol BDF encodes bus:dev:fn.  PFs are at fn 0-7 (same dev as
 * PF0).  VFs have devfn = pf_fn + vf_offset + vf_idx * vf_stride, which
 * may exceed fn 7 and spill into the dev field.
 */
static PCIDevice *rpcie_resolve_device(RemotePciePort *rp, uint16_t bdf)
{
    /* PF0 — fast path */
    if (bdf == rp->remote_bdf) return PCI_DEVICE(rp);

    /* PF companions — compare by stored remote_bdf */
    for (int i = 0; i < 7; i++) {
        if (rp->pf_companions[i] &&
            rp->pf_companions[i]->remote_bdf == bdf) {
            return PCI_DEVICE(rp->pf_companions[i]);
        }
    }

    /* SR-IOV VFs — compute VF index from BDF arithmetic for PF0 */
    if (rp->sriov_capable && rp->sriov_vf_stride > 0) {
        PCIDevice *d = PCI_DEVICE(rp);
        /* Extract 8-bit devfn from protocol BDF (bus=0) */
        int pf0_devfn = (RPCIE_BDF_DEV(rp->remote_bdf) << 3) |
                         RPCIE_BDF_FN(rp->remote_bdf);
        int target_devfn = (RPCIE_BDF_DEV(bdf) << 3) | RPCIE_BDF_FN(bdf);
        int delta = target_devfn - pf0_devfn - rp->sriov_vf_offset;
        if (delta >= 0 && (delta % rp->sriov_vf_stride) == 0) {
            int vf_idx = delta / rp->sriov_vf_stride;
            uint16_t num_vfs = pcie_sriov_num_vfs(d);
            if (vf_idx >= 0 && vf_idx < num_vfs &&
                d->exp.sriov_pf.vf[vf_idx]) {
                return d->exp.sriov_pf.vf[vf_idx];
            }
        }
    }

    /* SR-IOV VFs on companion PFs */
    for (int i = 0; i < 7; i++) {
        RemotePciePortPF *pf = rp->pf_companions[i];
        if (!pf || !pf->sriov_capable || pf->sriov_vf_stride == 0)
            continue;
        PCIDevice *pd = PCI_DEVICE(pf);
        int pf_devfn = (RPCIE_BDF_DEV(pf->remote_bdf) << 3) |
                        RPCIE_BDF_FN(pf->remote_bdf);
        int target_devfn = (RPCIE_BDF_DEV(bdf) << 3) | RPCIE_BDF_FN(bdf);
        int delta = target_devfn - pf_devfn - pf->sriov_vf_offset;
        if (delta >= 0 && (delta % pf->sriov_vf_stride) == 0) {
            int vf_idx = delta / pf->sriov_vf_stride;
            uint16_t num_vfs = pcie_sriov_num_vfs(pd);
            if (vf_idx >= 0 && vf_idx < num_vfs &&
                pd->exp.sriov_pf.vf[vf_idx]) {
                return pd->exp.sriov_pf.vf[vf_idx];
            }
        }
    }

    return PCI_DEVICE(rp);
}

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
    uint32_t available = len - (uint32_t)sizeof(rpcie_tlp_mem_t);

    if (available < 1) return;

    /*
     * PCI TLP payloads should be DW-aligned, but handle sub-dword
     * payloads gracefully by zero-padding if needed.
     */
    if (available >= nbytes) {
        pci_dma_write(rpcie_resolve_device(rp, tlp->hdr.requester_id), tlp->address,
                      payload + sizeof(rpcie_tlp_mem_t), nbytes);
    } else {
        uint8_t padded[RPCIE_MAX_PAYLOAD];
        memset(padded, 0, nbytes);
        memcpy(padded, payload + sizeof(rpcie_tlp_mem_t), available);
        pci_dma_write(rpcie_resolve_device(rp, tlp->hdr.requester_id), tlp->address, padded, nbytes);
    }
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

    pci_dma_read(rpcie_resolve_device(rp, tlp->hdr.requester_id), tlp->address,
                 buf + sizeof(rpcie_tlp_cpl_t), nbytes);

    rpcie_send_msg(rp, seq, buf, sizeof(rpcie_tlp_cpl_t) + nbytes);
}

static void rpcie_rx_handle_interrupt(RemotePciePort *rp,
                                      const uint8_t *payload, uint32_t len)
{
    if (len < sizeof(rpcie_tlp_msg_t)) return;

    const rpcie_tlp_msg_t *msg = (const rpcie_tlp_msg_t *)payload;

    if (msg->msg_code == RPCIE_MSG_INTX_ASSERT) {
        pci_set_irq(rpcie_resolve_device(rp, msg->hdr.requester_id), 1);
        return;
    }
    if (msg->msg_code == RPCIE_MSG_INTX_DEASSERT) {
        pci_set_irq(rpcie_resolve_device(rp, msg->hdr.requester_id), 0);
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
        msi_send_message(rpcie_resolve_device(rp, msg->hdr.requester_id), msi_msg);
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

    while (qatomic_read(&rp->rx_running)) {
        struct pollfd pfd = { .fd = rp->conn_fd, .events = POLLIN };
        if (poll(&pfd, 1, 100) <= 0) continue;

        if (rpcie_recv_full(rp->conn_fd, &hdr, sizeof(hdr)) < 0) {
            error_report("rpcie: connection closed");
            qatomic_set(&rp->rx_running, false);
            break;
        }
        if (hdr.magic != RPCIE_MAGIC || hdr.length > sizeof(buf)) {
            error_report("rpcie: bad frame (magic=0x%x len=%u)",
                         hdr.magic, hdr.length);
            qatomic_set(&rp->rx_running, false);
            break;
        }
        if (hdr.length > 0 &&
            rpcie_recv_full(rp->conn_fd, buf, hdr.length) < 0) {
            qatomic_set(&rp->rx_running, false);
            break;
        }
        if (hdr.length == 0) continue;

        uint8_t ft = buf[0];

        if (RPCIE_IS_CTRL(ft)) {
            switch (ft) {
            case RPCIE_CTRL_LINK_DOWN:
                info_report("rpcie: simulation sent LINK_DOWN");
                qatomic_set(&rp->rx_running, false);
                break;
            case RPCIE_CTRL_RESET:
                info_report("rpcie: reset from simulation");
                break;
            case RPCIE_CTRL_FN_ADD:
                info_report("rpcie: ignoring late FN_ADD in RX thread");
                break;
            case RPCIE_CTRL_LINK_UP:
                info_report("rpcie: received LINK_UP in RX thread");
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
 * Apply a parsed FN_ADD message to a PCI device.
 * Configures identity, BARs, capabilities (MSI-X, MSI, SR-IOV,
 * custom caps) on the given PCIDevice.  Used for both PF0 (the
 * remote-pcie-port device itself) and companion PFs.
 */
static int rpcie_apply_fn_add(PCIDevice *d, RemotePciePort *rp,
                              uint16_t remote_bdf,
                              const uint8_t *buf, uint32_t len,
                              Error **errp)
{
    if (len < sizeof(rpcie_ctrl_fn_add_t)) {
        error_setg(errp, "rpcie: FN_ADD too short (%u)", len);
        return -1;
    }

    rpcie_ctrl_fn_add_t fn;
    memcpy(&fn, buf, sizeof(fn));

    /* Set PCI identity in local config space */
    pci_config_set_vendor_id(d->config, fn.vendor_id);
    pci_config_set_device_id(d->config, fn.device_id);
    pci_set_word(d->config + PCI_SUBSYSTEM_VENDOR_ID, fn.subsys_vendor_id);
    pci_set_word(d->config + PCI_SUBSYSTEM_ID, fn.subsys_id);
    pci_config_set_class(d->config,
                         ((uint16_t)fn.class_code[2] << 8) | fn.class_code[1]);
    pci_config_set_prog_interface(d->config, fn.class_code[0]);
    pci_set_byte(d->config + PCI_REVISION_ID, fn.revision);

    /* Enable legacy INTx routing (INTA) */
    d->config[PCI_INTERRUPT_PIN] = 1;

    /* Parse BAR descriptors */
    const uint8_t *p = buf + sizeof(rpcie_ctrl_fn_add_t);
    uint32_t remaining = len - sizeof(rpcie_ctrl_fn_add_t);
    int nb = fn.num_bars;
    if (nb > RPCIE_MAX_BARS) nb = RPCIE_MAX_BARS;
    int num_bars_local = nb;

    /* For PF0, store bar count */
    if (d == PCI_DEVICE(rp)) {
        rp->num_bars = nb;
    }

    for (int i = 0; i < nb && remaining >= sizeof(rpcie_bar_desc_t); i++) {
        rpcie_bar_desc_t bar;
        memcpy(&bar, p, sizeof(bar));
        p += sizeof(rpcie_bar_desc_t);
        remaining -= sizeof(rpcie_bar_desc_t);

        if (bar.type == RPCIE_BAR_DISABLED || bar.size == 0) continue;

        RpcieBarContext *ctx = g_new0(RpcieBarContext, 1);
        ctx->rp         = rp;
        ctx->bar_idx    = i;
        ctx->remote_bdf = remote_bdf;

        /* Select the correct bar_mr array */
        MemoryRegion *mr;
        if (d == PCI_DEVICE(rp)) {
            mr = &rp->bar_mr[i];
            rp->bar_size[i] = bar.size;
            rp->bar_type[i] = bar.type;
        } else {
            RemotePciePortPF *pf = REMOTE_PCIE_PORT_PF(d);
            mr = &pf->bar_mr[i];
        }

        char name[48];
        snprintf(name, sizeof(name), "rpcie-bar%d-fn%d", i,
                 RPCIE_BDF_FN(remote_bdf));
        memory_region_init_io(mr, OBJECT(d),
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

        pci_register_bar(d, i, pci_type, mr);

        if (bar.type == RPCIE_BAR_MEM64) i++;
    }

    /* ---- PCIe Endpoint Capability ---- */
    if (pcie_endpoint_cap_init(d, 0) < 0) {
        error_setg(errp, "rpcie: failed to init PCIe endpoint cap");
        return -1;
    }

    /* ---- FLR (Function Level Reset) ---- */
    if (fn.flr_capable) {
        pcie_cap_flr_init(d);
        info_report("rpcie: FLR capability enabled for BDF 0x%04x",
                    remote_bdf);
    }

    /* ---- Parse MSI-X info ---- */
    if (fn.has_msix && remaining >= sizeof(rpcie_msix_info_t)) {
        rpcie_msix_info_t msix;
        memcpy(&msix, p, sizeof(msix));
        p += sizeof(rpcie_msix_info_t);
        remaining -= sizeof(rpcie_msix_info_t);

        int cap_pos = pci_add_capability(d, PCI_CAP_ID_MSIX, 0,
                                         MSIX_CAP_LENGTH, NULL);
        if (cap_pos >= 0) {
            pci_set_word(d->config + cap_pos + PCI_MSIX_FLAGS,
                         (msix.table_size - 1) & PCI_MSIX_FLAGS_QSIZE);
            pci_set_word(d->wmask + cap_pos + PCI_MSIX_FLAGS,
                         PCI_MSIX_FLAGS_ENABLE | PCI_MSIX_FLAGS_MASKALL);
            pci_set_long(d->config + cap_pos + PCI_MSIX_TABLE,
                         (msix.table_offset & ~0x7u) |
                         (msix.table_bar & 0x7));
            pci_set_long(d->config + cap_pos + PCI_MSIX_PBA,
                         (msix.pba_offset & ~0x7u) |
                         (msix.pba_bar & 0x7));
        }
    }

    /* ---- Parse MSI info ---- */
    if (fn.has_msi && remaining >= sizeof(rpcie_msi_info_t)) {
        rpcie_msi_info_t msi_info;
        memcpy(&msi_info, p, sizeof(msi_info));
        p += sizeof(rpcie_msi_info_t);
        remaining -= sizeof(rpcie_msi_info_t);

        int msi_flags = PCI_MSI_FLAGS_64BIT * msi_info.is_64bit;
        if (msi_init(d, 0, msi_info.num_vectors,
                     msi_info.is_64bit, msi_info.per_vector_mask,
                     NULL) < 0) {
            info_report("rpcie: msi_init failed (non-fatal)");
            (void)msi_flags;
        }
    }

    /* ---- Parse SR-IOV info — save for deferred init ---- */
    if (fn.has_sriov && remaining >= sizeof(rpcie_sriov_info_t)) {
        rpcie_sriov_info_t sriov;
        memcpy(&sriov, p, sizeof(sriov));
        p += sizeof(rpcie_sriov_info_t);
        remaining -= sizeof(rpcie_sriov_info_t);

        /* Parse VF BAR descriptors into temporaries */
        uint64_t vf_bsz[RPCIE_MAX_BARS] = {0};
        uint8_t  vf_bty[RPCIE_MAX_BARS] = {0};
        uint8_t  vf_bpf[RPCIE_MAX_BARS] = {0};
        for (int i = 0; i < sriov.num_vf_bars && i < RPCIE_MAX_BARS
             && remaining >= sizeof(rpcie_bar_desc_t); i++) {
            rpcie_bar_desc_t vf_bar;
            memcpy(&vf_bar, p, sizeof(vf_bar));
            p += sizeof(rpcie_bar_desc_t);
            remaining -= sizeof(rpcie_bar_desc_t);
            vf_bsz[i] = vf_bar.size;
            vf_bty[i] = vf_bar.type;
            vf_bpf[i] = vf_bar.prefetchable;
        }

        /* Store into the correct PF struct */
        if (d == PCI_DEVICE(rp)) {
            rp->sriov_capable      = true;
            rp->sriov_total_vfs    = sriov.total_vfs;
            rp->sriov_vf_device_id = sriov.vf_device_id;
            rp->sriov_vf_offset    = sriov.vf_offset;
            rp->sriov_vf_stride    = sriov.vf_stride;
            rp->sriov_num_vf_bars  = sriov.num_vf_bars;
            rp->sriov_sup_pgsize   = sriov.supported_page_sizes;
            memcpy(rp->sriov_vf_bar_size, vf_bsz, sizeof(vf_bsz));
            memcpy(rp->sriov_vf_bar_type, vf_bty, sizeof(vf_bty));
            memcpy(rp->sriov_vf_bar_prefetch, vf_bpf, sizeof(vf_bpf));
        } else {
            RemotePciePortPF *pf = REMOTE_PCIE_PORT_PF(d);
            pf->sriov_capable      = true;
            pf->sriov_total_vfs    = sriov.total_vfs;
            pf->sriov_vf_device_id = sriov.vf_device_id;
            pf->sriov_vf_offset    = sriov.vf_offset;
            pf->sriov_vf_stride    = sriov.vf_stride;
            pf->sriov_num_vf_bars  = sriov.num_vf_bars;
            pf->sriov_sup_pgsize   = sriov.supported_page_sizes;
            memcpy(pf->sriov_vf_bar_size, vf_bsz, sizeof(vf_bsz));
            memcpy(pf->sriov_vf_bar_type, vf_bty, sizeof(vf_bty));
            memcpy(pf->sriov_vf_bar_prefetch, vf_bpf, sizeof(vf_bpf));
        }

        info_report("rpcie: SR-IOV data saved for BDF 0x%04x "
                    "(total_vfs=%d vf_dev=0x%04x), deferred",
                    remote_bdf, sriov.total_vfs, sriov.vf_device_id);
    }

    /* ---- Parse custom PCI capabilities ---- */
    for (int i = 0; i < fn.num_custom_caps
             && remaining >= sizeof(rpcie_custom_cap_desc_t); i++) {
        rpcie_custom_cap_desc_t desc;
        memcpy(&desc, p, sizeof(desc));
        p += sizeof(rpcie_custom_cap_desc_t);
        remaining -= sizeof(rpcie_custom_cap_desc_t);

        if (desc.data_len > remaining) {
            info_report("rpcie: custom cap %d truncated", i);
            break;
        }

        int total_len = 2 + desc.data_len;
        int aligned = (total_len + 3) & ~3;
        int cap_pos = pci_add_capability(d, desc.cap_id, 0, aligned, NULL);
        if (cap_pos >= 0) {
            memcpy(d->config + cap_pos + 2, p, desc.data_len);
            info_report("rpcie: added custom cap 0x%02x at 0x%x (%d bytes)",
                        desc.cap_id, cap_pos, desc.data_len);

            /* PM capability (ID 0x01): set PMCSR writable mask */
            if (desc.cap_id == 0x01 && desc.data_len >= 4) {
                /* PMCSR is at cap_pos + 4 (after 2B header + 2B PMC) */
                /* Bits [1:0] = PowerState (RW) */
                /* Bit [8] = PME_En (RW) */
                pci_set_word(d->wmask + cap_pos + 4, 0x0103);
                /* Bit [15] = PME_Status (W1C) */
                pci_set_word(d->w1cmask + cap_pos + 4, 0x8000);
                info_report("rpcie: PM cap PMCSR writable mask set at 0x%x",
                            cap_pos + 4);
            }
        }

        p += desc.data_len;
        remaining -= desc.data_len;
    }

    /* ---- Parse custom PCIe extended capabilities ---- */
    {
        /* Allocate ext caps sequentially starting at 0x100 */
        uint16_t ext_cap_offset = PCI_CONFIG_SPACE_SIZE; /* 0x100 */

        for (int i = 0; i < fn.num_custom_ext_caps
                 && remaining >= sizeof(rpcie_custom_ext_cap_desc_t); i++) {
            rpcie_custom_ext_cap_desc_t desc;
            memcpy(&desc, p, sizeof(desc));
            p += sizeof(rpcie_custom_ext_cap_desc_t);
            remaining -= sizeof(rpcie_custom_ext_cap_desc_t);

            if (desc.data_len > remaining) {
                info_report("rpcie: custom ext cap %d truncated", i);
                break;
            }

            /* Extended cap header is 4 bytes, body follows */
            int total_len = 4 + desc.data_len;
            int aligned = (total_len + 3) & ~3;
            if (aligned < 8) aligned = 8;  /* QEMU requires size >= 8 */

            if (ext_cap_offset + aligned > PCIE_CONFIG_SPACE_SIZE) {
                info_report("rpcie: ext cap %d (0x%04x) doesn't fit at 0x%x",
                            i, desc.cap_id, ext_cap_offset);
                p += desc.data_len;
                remaining -= desc.data_len;
                continue;
            }

            pcie_add_capability(d, desc.cap_id, desc.cap_version,
                                ext_cap_offset, aligned);
            /* Copy body data after the 4-byte ext cap header */
            if (desc.data_len > 0) {
                memcpy(d->config + ext_cap_offset + 4, p, desc.data_len);
            }

            info_report("rpcie: added ext cap 0x%04x v%d at 0x%x (%d bytes)",
                        desc.cap_id, desc.cap_version, ext_cap_offset,
                        desc.data_len);

            /* AER (ext cap ID 0x0001): set writable/W1C masks */
            if (desc.cap_id == 0x0001 && desc.data_len >= 20) {
                uint16_t o = ext_cap_offset;
                /* Uncorrectable Error Status (offset +0x04): W1C */
                pci_set_long(d->w1cmask + o + 0x04, 0xFFFFFFFF);
                /* Uncorrectable Error Mask (offset +0x08): RW */
                pci_set_long(d->wmask + o + 0x08, 0xFFFFFFFF);
                /* Uncorrectable Error Severity (offset +0x0C): RW */
                pci_set_long(d->wmask + o + 0x0C, 0xFFFFFFFF);
                /* Correctable Error Status (offset +0x10): W1C */
                pci_set_long(d->w1cmask + o + 0x10, 0xFFFFFFFF);
                if (desc.data_len >= 24) {
                    /* Correctable Error Mask (offset +0x14): RW */
                    pci_set_long(d->wmask + o + 0x14, 0xFFFFFFFF);
                }
                info_report("rpcie: AER ext cap masks set at 0x%x", o);
            }

            ext_cap_offset += aligned;
            p += desc.data_len;
            remaining -= desc.data_len;
        }
    }

    info_report("rpcie: fn %02x:%02x.%x device %04x:%04x, %d BARs",
                RPCIE_BDF_BUS(remote_bdf), RPCIE_BDF_DEV(remote_bdf),
                RPCIE_BDF_FN(remote_bdf),
                fn.vendor_id, fn.device_id, num_bars_local);
    return 0;
}

/*
 * Read a single framed message from the socket.
 * Returns the msg_type (first byte of payload), or -1 on error/timeout.
 */
static int rpcie_recv_one_msg(int fd, uint8_t *buf, uint32_t bufsize,
                              uint32_t *out_len, int timeout_ms)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr <= 0) return -1;

    rpcie_frame_hdr_t hdr;
    if (rpcie_recv_full(fd, &hdr, sizeof(hdr)) < 0 ||
        hdr.magic != RPCIE_MAGIC) {
        return -1;
    }

    if (hdr.length > bufsize) {
        uint8_t drain[256];
        uint32_t left = hdr.length;
        while (left > 0) {
            uint32_t n = left > sizeof(drain) ? sizeof(drain) : left;
            if (rpcie_recv_full(fd, drain, n) < 0) return -1;
            left -= n;
        }
        *out_len = 0;
        return -1;
    }

    if (hdr.length > 0 &&
        rpcie_recv_full(fd, buf, hdr.length) < 0) {
        return -1;
    }

    *out_len = hdr.length;
    return (hdr.length > 0) ? buf[0] : -1;
}

/*
 * Finalize SR-IOV after all PF companions have been created.
 * Adjusts vf_offset to skip past PF function slots, then creates VFs.
 */

/*
 * Find the first free offset in the PCIe extended capability space
 * (0x100..0xFFF) for a device, by walking the existing ext-cap chain.
 * Returns the offset suitable for pcie_add_capability(), or 0 if no
 * room for `needed` bytes.
 */
static uint16_t rpcie_find_free_ext_cap_offset(PCIDevice *dev, uint16_t needed)
{
    uint16_t pos = PCI_CONFIG_SPACE_SIZE; /* 0x100 */

    while (pos >= PCI_CONFIG_SPACE_SIZE &&
           pos + needed <= PCIE_CONFIG_SPACE_SIZE) {
        uint32_t header = pci_get_long(dev->config + pos);
        if (header == 0) {
            return pos;  /* empty slot */
        }
        uint16_t next = PCI_EXT_CAP_NEXT(header);
        if (next == 0) {
            /* Last cap in chain — skip past it using QEMU's internal
             * tracking.  pcie_find_capability returns the offset if it
             * exists, and QEMU always chains caps via next pointers.
             * Since there's no next pointer, we need to estimate the
             * size of this last capability. */
            uint16_t cap_id = PCI_EXT_CAP_ID(header);
            uint16_t cap_size;
            switch (cap_id) {
            case PCI_EXT_CAP_ID_ERR:   cap_size = PCI_ERR_SIZEOF;          break;
            case PCI_EXT_CAP_ID_SRIOV: cap_size = PCI_EXT_CAP_SRIOV_SIZEOF; break;
            case PCI_EXT_CAP_ID_ARI:   cap_size = PCI_ARI_SIZEOF;          break;
            case PCI_EXT_CAP_ID_VNDR: /* Vendor-Specific (VSEC) */ {
                uint32_t vsec_hdr = pci_get_long(dev->config + pos + 4);
                cap_size = (vsec_hdr >> 20) & 0xFFF;
                if (cap_size < 8) cap_size = 8;
                break;
            }
            default:                   cap_size = 8; /* QEMU minimum */    break;
            }
            uint16_t candidate = (pos + cap_size + 3) & ~3; /* 4-byte align */
            return (candidate + needed <= PCIE_CONFIG_SPACE_SIZE)
                   ? candidate : 0;
        }
        pos = next;
    }
    return 0;  /* no room */
}

/*
 * Common SR-IOV PF initialisation for both PF0 and companion PFs.
 * Places the SR-IOV extended capability after any existing ext caps.
 * Returns true on success.
 */
static bool rpcie_init_sriov_on_pf(PCIDevice *d,
                                   uint16_t vf_device_id,
                                   uint16_t total_vfs,
                                   uint16_t vf_offset,
                                   uint16_t vf_stride,
                                   uint8_t  num_vf_bars,
                                   const uint64_t *vf_bar_size,
                                   const uint8_t  *vf_bar_type,
                                   const uint8_t  *vf_bar_prefetch,
                                   uint32_t sup_pgsize)
{
    uint16_t sriov_off = rpcie_find_free_ext_cap_offset(
        d, PCI_EXT_CAP_SRIOV_SIZEOF);
    if (sriov_off == 0) {
        info_report("rpcie: no room for SR-IOV ext cap on devfn %d",
                    d->devfn);
        return false;
    }

    if (!pcie_sriov_pf_init(d, sriov_off,
                            TYPE_REMOTE_PCIE_PORT_VF,
                            vf_device_id,
                            total_vfs, total_vfs,
                            vf_offset, vf_stride,
                            NULL)) {
        return false;
    }

    for (int i = 0; i < num_vf_bars && i < RPCIE_MAX_BARS; i++) {
        if (vf_bar_size[i] == 0) continue;
        int pci_type = PCI_BASE_ADDRESS_SPACE_MEMORY;
        if (vf_bar_type[i] == RPCIE_BAR_MEM64)
            pci_type |= PCI_BASE_ADDRESS_MEM_TYPE_64;
        if (vf_bar_prefetch && vf_bar_prefetch[i])
            pci_type |= PCI_BASE_ADDRESS_MEM_PREFETCH;
        pcie_sriov_pf_init_vf_bar(d, i, pci_type, vf_bar_size[i]);
    }

    if (sup_pgsize)
        pcie_sriov_pf_add_sup_pgsize(d, sup_pgsize);

    d->config[PCI_HEADER_TYPE] |= PCI_HEADER_TYPE_MULTI_FUNCTION;

    /* Set ARI Capable Hierarchy (bit 1) in SR-IOV Capabilities.
     * This tells Linux the device supports ARI-based VF addressing,
     * enabling VFs at devfn > 7. */
    pci_set_long(d->config + d->exp.sriov_cap + PCI_SRIOV_CAP,
                 pci_get_long(d->config + d->exp.sriov_cap
                              + PCI_SRIOV_CAP) | 0x02);
    return true;
}

static void rpcie_finalize_sriov(RemotePciePort *rp)
{
    PCIDevice *d = PCI_DEVICE(rp);

    if (!rp->sriov_capable)
        return;

    /* Ensure VF offset doesn't collide with PF companion functions.
     * If we have num_pfs PFs occupying functions 0..(num_pfs-1),
     * VFs must start at function num_pfs or later. */
    uint16_t vf_offset = rp->sriov_vf_offset;
    uint16_t vf_stride = rp->sriov_vf_stride;

    if (rp->num_pfs > 1) {
        /* With multiple PFs, stride must equal num_pfs so that each
         * PF's VFs interleave: PF0 gets devfn 4,8,12,16; PF1 gets
         * 5,9,13,17; etc. */
        if (vf_stride != rp->num_pfs) {
            info_report("rpcie: adjusted VF stride %d -> %d for multi-PF interleave",
                        vf_stride, rp->num_pfs);
            vf_stride = rp->num_pfs;
            rp->sriov_vf_stride = vf_stride;
        }

        uint8_t pf0_fn = PCI_FUNC(d->devfn);
        uint16_t first_vf_fn = pf0_fn + vf_offset;

        /* Check if any VF would land on a PF companion function */
        for (int i = 0; i < rp->sriov_total_vfs; i++) {
            uint8_t vf_fn = (first_vf_fn + i * vf_stride) & 0x7;
            for (int pf = 0; pf < rp->num_pfs - 1 && pf < 7; pf++) {
                if (rp->pf_companions[pf] &&
                    PCI_FUNC(PCI_DEVICE(rp->pf_companions[pf])->devfn) == vf_fn) {
                    /* Collision detected — bump vf_offset past all PFs */
                    vf_offset = rp->num_pfs;
                    info_report("rpcie: adjusted VF offset %d -> %d to avoid PF collision",
                                rp->sriov_vf_offset, vf_offset);
                    rp->sriov_vf_offset = vf_offset;
                    goto offset_fixed;
                }
            }
        }
    }
offset_fixed:

    if (!rpcie_init_sriov_on_pf(d, rp->sriov_vf_device_id,
                                rp->sriov_total_vfs,
                                vf_offset, vf_stride,
                                rp->sriov_num_vf_bars,
                                rp->sriov_vf_bar_size,
                                rp->sriov_vf_bar_type,
                                rp->sriov_vf_bar_prefetch,
                                rp->sriov_sup_pgsize)) {
        info_report("rpcie: SR-IOV init failed for PF0 (non-fatal)");
        rp->sriov_capable = false;
    } else {
        info_report("rpcie: SR-IOV enabled, total_vfs=%d vf_dev=0x%04x offset=%d stride=%d",
                    rp->sriov_total_vfs, rp->sriov_vf_device_id,
                    vf_offset, vf_stride);
    }
}

/*
 * (Re)build the ARI Extended Capability chain across all active devices
 * on the secondary bus.  This enables Linux to discover VFs at devfn > 7
 * via ARI-aware bus scanning after SR-IOV VF Enable.
 *
 * The chain links all devices with non-0xFFFF vendor ID in devfn order.
 * Called:
 *   - Once at init (PF-only chain)
 *   - Each time any PF's VFE bit changes (includes/excludes VFs)
 */
static void rpcie_rebuild_ari_chain(RemotePciePort *rp)
{
    PCIDevice *pf0 = PCI_DEVICE(rp);
    PCIBus *bus = pci_get_bus(pf0);
    int bus_num = pci_bus_num(bus);

    /* Collect all devfns with active (VID != 0xFFFF) devices */
    int active[256];
    int count = 0;

    for (int devfn = 0; devfn < 256; devfn++) {
        PCIDevice *dev = pci_find_device(bus, bus_num, devfn);
        if (!dev) continue;
        uint16_t vid = pci_get_word(dev->config + PCI_VENDOR_ID);
        if (vid == 0xFFFF || vid == 0x0000) continue;
        active[count++] = devfn;
    }

    if (count <= 1) return;  /* ARI only needed for multi-function */

    /* Link each active device to the next via ARI Extended Capability */
    for (int i = 0; i < count; i++) {
        PCIDevice *dev = pci_find_device(bus, bus_num, active[i]);
        uint8_t next_fn = (i + 1 < count) ? (uint8_t)active[i + 1] : 0;

        /* Add ARI cap if not already present */
        uint16_t ari_off = pcie_find_capability(dev, PCI_EXT_CAP_ID_ARI);
        if (!ari_off) {
            uint16_t offset = rpcie_find_free_ext_cap_offset(
                dev, PCI_ARI_SIZEOF);
            if (offset == 0) {
                info_report("rpcie: no room for ARI cap on devfn %d",
                            active[i]);
                continue;
            }

            pcie_add_capability(dev, PCI_EXT_CAP_ID_ARI, PCI_ARI_VER,
                                offset, PCI_ARI_SIZEOF);
            ari_off = offset;
        }

        /* Set Next Function Number in ARI Capability register */
        pci_set_long(dev->config + ari_off + PCI_ARI_CAP,
                     (uint32_t)next_fn << 8);
    }

    info_report("rpcie: ARI chain rebuilt — %d active device(s), "
                "first=devfn %d, last=devfn %d",
                count, active[0], active[count - 1]);
}

/*
 * Receive all FN_ADD messages, then LINK_UP.
 * First FN_ADD configures PF0 (self).  Additional FN_ADD messages
 * create companion PCI devices at the same slot but different function
 * numbers, enabling multi-function endpoints over a single socket.
 */
static int rpcie_recv_fn_adds(RemotePciePort *rp, Error **errp)
{
    PCIDevice *d = PCI_DEVICE(rp);
    uint8_t buf[4096];
    uint32_t len;
    int fn_count = 0;

    for (;;) {
        int timeout = (fn_count == 0) ? 60000 : 5000;
        int msg_type = rpcie_recv_one_msg(rp->conn_fd, buf, sizeof(buf),
                                          &len, timeout);

        if (msg_type == RPCIE_CTRL_FN_ADD) {
            if (len < sizeof(rpcie_ctrl_fn_add_t)) {
                error_setg(errp, "rpcie: FN_ADD too short (%u)", len);
                return -1;
            }

            rpcie_ctrl_fn_add_t fn_hdr;
            memcpy(&fn_hdr, buf, sizeof(fn_hdr));
            uint8_t fn_num = RPCIE_BDF_FN(fn_hdr.bdf);

            if (fn_count == 0) {
                /* First FN_ADD: configure PF0 (self) */
                rp->remote_bdf = fn_hdr.bdf;
                rp->num_pfs = 1;
                if (rpcie_apply_fn_add(d, rp, fn_hdr.bdf,
                                       buf, len, errp) < 0) {
                    return -1;
                }
            } else {
                /* Additional FN_ADD: create companion PF device */
                if (fn_num == 0 || fn_num > 7) {
                    info_report("rpcie: ignoring FN_ADD with fn=%d", fn_num);
                    fn_count++;
                    continue;
                }

                info_report("rpcie: creating companion PF%d (fn=%d)",
                            fn_count, fn_num);

                /* Set multifunction bit on PF0 (both cap_present and config) */
                d->cap_present |= QEMU_PCI_CAP_MULTIFUNCTION;
                d->config[PCI_HEADER_TYPE] |= PCI_HEADER_TYPE_MULTI_FUNCTION;

                /* Create companion on same bus/slot, different function */
                int slot = PCI_SLOT(d->devfn);
                int devfn = PCI_DEVFN(slot, fn_num);

                PCIDevice *pf_dev = pci_new_multifunction(devfn,
                    TYPE_REMOTE_PCIE_PORT_PF);
                if (!pf_dev) {
                    error_setg(errp, "rpcie: pci_new_multifunction failed");
                    return -1;
                }
                RemotePciePortPF *pf = REMOTE_PCIE_PORT_PF(pf_dev);

                /* Set companion fields — realization deferred until
                 * after LINK_UP so that num_pfs is known and
                 * pcie_sriov_pf_init() can run during realize. */
                pf->pf0 = rp;
                pf->remote_bdf = fn_hdr.bdf;
                pf->pf_index = fn_num;
                uint32_t copy_len = len < sizeof(pf->fn_add_buf) ? len : sizeof(pf->fn_add_buf);
                memcpy(pf->fn_add_buf, buf, copy_len);
                pf->fn_add_len = copy_len;

                rp->pf_companions[fn_num - 1] = pf;
                rp->num_pfs++;
            }

            fn_count++;

        } else if (msg_type == RPCIE_CTRL_LINK_UP) {
            info_report("rpcie: LINK_UP after %d FN_ADD(s)", fn_count);
            break;

        } else if (msg_type < 0) {
            if (fn_count == 0) {
                error_setg(errp, "rpcie: timeout waiting for FN_ADD");
                return -1;
            }
            info_report("rpcie: no more msgs after %d FN_ADD(s)", fn_count);
            break;

        } else {
            info_report("rpcie: unexpected 0x%02x during FN_ADD phase",
                        msg_type);
        }
    }

    if (fn_count == 0) {
        error_setg(errp, "rpcie: no FN_ADD received");
        return -1;
    }

    /* Finalize SR-IOV on PF0 (already realized) */
    rpcie_finalize_sriov(rp);

    /* Now realize companion PFs.  num_pfs is final, so
     * rpcie_pf_realize() can call pcie_sriov_pf_init() safely
     * (the device is not yet realized at that point). */
    for (int i = 0; i < 7; i++) {
        RemotePciePortPF *pf = rp->pf_companions[i];
        if (!pf) continue;
        PCIBus *comp_bus = pci_get_bus(d);
        if (!pci_realize_and_unref(PCI_DEVICE(pf), comp_bus, errp)) {
            return -1;
        }
        info_report("rpcie: companion PF%d realized successfully",
                    pf->pf_index);
    }

    /* Build initial ARI chain (PFs only — VFs are disabled/VID=0xFFFF) */
    rpcie_rebuild_ari_chain(rp);

    info_report("rpcie: %d PF(s) configured", rp->num_pfs);
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

    /* Wait for FN_ADD(s) — configures identity + BARs for all PFs */
    if (rpcie_recv_fn_adds(rp, errp) < 0) {
        goto fail;
    }

    /*
     * Set up shared memory (SHM) for zero-copy DMA.
     *
     * If both sides negotiated RPCIE_CAP_SHM during the handshake, send
     * the guest RAM memfd to the simulation so it can mmap it directly.
     * This eliminates the TLP round-trip for every DMA read/write,
     * dramatically improving DMA-heavy workloads (Virtio, NIC, etc.).
     *
     * Protocol: send framed SHM_SETUP message, then pass the RAM fd
     * via a separate sendmsg() with SCM_RIGHTS, then wait for SHM_ACK.
     */
    if (rp->remote_caps & RPCIE_CAP_SHM) {
        MachineState *ms = MACHINE(qdev_get_machine());
        MemoryRegion *ram = ms->ram;
        int ram_fd = memory_region_get_fd(ram);
        uint64_t ram_size = memory_region_size(ram);

        if (ram_fd >= 0) {
            /* Determine x86 below-4G RAM size for PCI hole translation */
            uint64_t below_4g = 0;
            if (object_dynamic_cast(OBJECT(ms), TYPE_X86_MACHINE)) {
                X86MachineState *x86ms = X86_MACHINE(ms);
                below_4g = x86ms->below_4g_mem_size;
            }

            rpcie_ctrl_shm_setup_t setup = {
                .msg_type      = RPCIE_CTRL_SHM_SETUP,
                .reserved      = 0,
                .region_id     = 0,    /* 0 = main guest RAM */
                .offset        = 0,
                .size          = ram_size,
                .below_4g_size = below_4g,
            };

            /* Send the SHM_SETUP framed message */
            if (rpcie_send_msg(rp, rpcie_alloc_seq(rp),
                               &setup, sizeof(setup)) < 0) {
                info_report("rpcie: SHM_SETUP send failed, "
                            "falling back to TLP DMA");
            } else {
                /* Send the fd via SCM_RIGHTS (separate sendmsg) */
                qemu_mutex_lock(&rp->send_mutex);
                int fd_rc = rpcie_send_fd(rp->conn_fd, ram_fd);
                qemu_mutex_unlock(&rp->send_mutex);

                if (fd_rc < 0) {
                    info_report("rpcie: SHM fd send failed, "
                                "falling back to TLP DMA");
                } else {
                    /* Wait for SHM_ACK (comes as a framed control message) */
                    uint8_t ack_buf[64];
                    uint32_t ack_len;
                    int ack_type = rpcie_recv_one_msg(rp->conn_fd, ack_buf,
                                                     sizeof(ack_buf),
                                                     &ack_len, 5000);
                    if (ack_type == RPCIE_CTRL_SHM_ACK &&
                        ack_len >= sizeof(rpcie_ctrl_shm_ack_t)) {
                        rpcie_ctrl_shm_ack_t ack;
                        memcpy(&ack, ack_buf, sizeof(ack));
                        if (ack.status == 0) {
                            info_report("rpcie: SHM enabled, %lu MB "
                                        "guest RAM mapped for zero-copy DMA",
                                        (unsigned long)(ram_size / (1024*1024)));
                        } else {
                            info_report("rpcie: SHM_ACK error %d, "
                                        "falling back to TLP DMA", ack.status);
                        }
                    } else {
                        info_report("rpcie: SHM_ACK timeout or unexpected "
                                    "msg (type=0x%02x), falling back to "
                                    "TLP DMA", ack_type);
                    }
                }
            }
        } else {
            info_report("rpcie: RAM not backed by fd, "
                        "SHM disabled (TLP DMA only)");
        }
    }

    /* Start RX thread */
    qatomic_set(&rp->rx_running, true);
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

    qatomic_set(&rp->rx_running, false);
    if (rp->conn_fd >= 0) {
        rpcie_ctrl_link_t link = { .msg_type = RPCIE_CTRL_LINK_DOWN };
        rpcie_send_msg(rp, rpcie_alloc_seq(rp), &link, sizeof(link));
        shutdown(rp->conn_fd, SHUT_RDWR);
    }
    qemu_thread_join(&rp->rx_thread);

    /* Destroy companion PF devices */
    for (int i = 0; i < 7; i++) {
        if (rp->pf_companions[i]) {
            object_unparent(OBJECT(rp->pf_companions[i]));
            rp->pf_companions[i] = NULL;
        }
    }

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

    if (rp->conn_fd >= 0 && qatomic_read(&rp->rx_running)) {
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
/*  PF companion device (multi-function support)                      */
/* ================================================================== */

static void rpcie_pf_realize(PCIDevice *d, Error **errp)
{
    RemotePciePortPF *pf = REMOTE_PCIE_PORT_PF(d);

    if (!pf->pf0 || pf->fn_add_len == 0) {
        error_setg(errp, "rpcie-pf: not properly initialized");
        return;
    }

    /* Apply FN_ADD data — configures identity, BARs, capabilities */
    if (rpcie_apply_fn_add(d, pf->pf0, pf->remote_bdf,
                           pf->fn_add_buf, pf->fn_add_len, errp) < 0) {
        return;
    }

    /* Initialize SR-IOV during realize (before device is marked realized)
     * so that pcie_sriov_pf_init() can set properties freely. */
    if (pf->sriov_capable) {
        int num_pfs = pf->pf0->num_pfs;
        uint16_t vf_offset = (uint16_t)num_pfs;
        uint16_t vf_stride = (num_pfs > 1) ? (uint16_t)num_pfs
                                            : pf->sriov_vf_stride;
        uint8_t pf_fn = PCI_FUNC(d->devfn);

        pf->sriov_vf_offset = vf_offset;
        pf->sriov_vf_stride = vf_stride;
        info_report("rpcie: PF%d SR-IOV init during realize: "
                    "offset=%d stride=%d (first VF at fn %d)",
                    pf->pf_index, vf_offset, vf_stride,
                    pf_fn + vf_offset);

        if (!rpcie_init_sriov_on_pf(d, pf->sriov_vf_device_id,
                                    pf->sriov_total_vfs,
                                    vf_offset, pf->sriov_vf_stride,
                                    pf->sriov_num_vf_bars,
                                    pf->sriov_vf_bar_size,
                                    pf->sriov_vf_bar_type,
                                    pf->sriov_vf_bar_prefetch,
                                    pf->sriov_sup_pgsize)) {
            info_report("rpcie: SR-IOV init failed for PF%d (non-fatal)",
                        pf->pf_index);
            pf->sriov_capable = false;
        } else {
            info_report("rpcie: SR-IOV enabled on PF%d, total_vfs=%d "
                        "vf_dev=0x%04x offset=%d stride=%d",
                        pf->pf_index, pf->sriov_total_vfs,
                        pf->sriov_vf_device_id, vf_offset,
                        pf->sriov_vf_stride);
        }
    }
}

static void rpcie_pf_exit(PCIDevice *d)
{
    RemotePciePortPF *pf = REMOTE_PCIE_PORT_PF(d);

    if (pf->sriov_capable) {
        pcie_sriov_pf_exit(d);
    }

    for (int i = 0; i < RPCIE_MAX_BARS; i++) {
        if (pf->bar_mr[i].size) {
            g_free(pf->bar_mr[i].opaque);
        }
    }
    pcie_cap_exit(d);
}

/*
 * Config-space write handler for companion PFs.
 * Mirrors rpcie_config_write() logic for FLR and SR-IOV event forwarding.
 */
static void rpcie_pf_config_write(PCIDevice *d, uint32_t address,
                                  uint32_t val, int len)
{
    RemotePciePortPF *pf = REMOTE_PCIE_PORT_PF(d);
    RemotePciePort *rp = pf->pf0;

    /* Snapshot SR-IOV state before write */
    bool old_vfe = false;
    uint16_t old_num_vfs = 0;
    if (pf->sriov_capable && d->exp.sriov_cap) {
        uint16_t sriov_cap = d->exp.sriov_cap;
        old_vfe = (pci_get_word(d->config + sriov_cap + PCI_SRIOV_CTRL)
                   & PCI_SRIOV_CTRL_VFE) != 0;
        old_num_vfs = old_vfe ?
            pci_get_word(d->config + sriov_cap + PCI_SRIOV_NUM_VF) : 0;
    }

    pci_default_write_config(d, address, val, len);

    /* FLR interception */
    if (d->exp.exp_cap) {
        uint8_t *devctl = d->config + d->exp.exp_cap + PCI_EXP_DEVCTL;
        if (pci_get_word(devctl) & PCI_EXP_DEVCTL_BCR_FLR) {
            pci_word_test_and_clear_mask(devctl, PCI_EXP_DEVCTL_BCR_FLR);
            if (rp->conn_fd >= 0 && qatomic_read(&rp->rx_running)) {
                rpcie_ctrl_reset_t msg = {
                    .msg_type   = RPCIE_CTRL_RESET,
                    .reset_type = RPCIE_RESET_FLR,
                    .target_bdf = pf->remote_bdf,
                };
                rpcie_send_msg(rp, rpcie_alloc_seq(rp), &msg, sizeof(msg));
                info_report("rpcie: FLR initiated for PF%d BDF 0x%04x",
                            pf->pf_index, pf->remote_bdf);
            }
        }
    }

    /* SR-IOV VFE state change → notify simulation */
    if (pf->sriov_capable && d->exp.sriov_cap) {
        uint16_t sriov_cap = d->exp.sriov_cap;
        bool new_vfe = (pci_get_word(d->config + sriov_cap + PCI_SRIOV_CTRL)
                        & PCI_SRIOV_CTRL_VFE) != 0;
        uint16_t new_num_vfs = new_vfe ?
            pci_get_word(d->config + sriov_cap + PCI_SRIOV_NUM_VF) : 0;

        /* Fix VF config-space identity (vendor/device IDs) */
        if (new_num_vfs > 0 && (!old_vfe || old_num_vfs != new_num_vfs)) {
            uint16_t pf_vendor = pci_get_word(d->config + PCI_VENDOR_ID);
            uint16_t vf_devid  = pci_get_word(
                d->config + sriov_cap + PCI_SRIOV_VF_DID);
            for (int i = 0; i < new_num_vfs; i++) {
                PCIDevice *vf_dev = d->exp.sriov_pf.vf[i];
                pci_config_set_vendor_id(vf_dev->config, pf_vendor);
                pci_config_set_device_id(vf_dev->config, vf_devid);
            }
        }

        if (old_vfe != new_vfe || old_num_vfs != new_num_vfs) {
            rpcie_ctrl_sriov_event_t ev = {
                .msg_type  = RPCIE_CTRL_SRIOV_EVENT,
                .pf_index  = (uint8_t)pf->pf_index,
                .vf_enable = new_vfe ? 1 : 0,
                .reserved  = 0,
                .num_vfs   = new_num_vfs,
                .reserved2 = 0,
            };
            rpcie_send_msg(rp, rpcie_alloc_seq(rp), &ev, sizeof(ev));
            info_report("rpcie: PF%d SR-IOV event → vf_enable=%d num_vfs=%d",
                        pf->pf_index, ev.vf_enable, new_num_vfs);

            /* Rebuild ARI chain to include/exclude this PF's VFs */
            rpcie_rebuild_ari_chain(rp);
        }
    }
}

static void rpcie_pf_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize      = rpcie_pf_realize;
    k->exit         = rpcie_pf_exit;
    k->config_write = rpcie_pf_config_write;
    k->vendor_id = PCI_VENDOR_ID_REDHAT;
    k->device_id = 0x0015;  /* overridden by rpcie_apply_fn_add */
    k->class_id  = PCI_CLASS_OTHERS;

    dc->desc = "Remote PCIe Endpoint PF (multi-function companion)";
    dc->user_creatable = false;
}

static const TypeInfo rpcie_pf_info = {
    .name          = TYPE_REMOTE_PCIE_PORT_PF,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(RemotePciePortPF),
    .class_init    = rpcie_pf_class_init,
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

    /* Determine parent PF type and extract SR-IOV fields */
    RemotePciePort *rp;
    uint8_t  num_vf_bars;
    uint64_t *vf_bar_size;
    uint8_t  *vf_bar_type;
    uint8_t  *vf_bar_prefetch;
    uint16_t pf_remote_bdf;
    uint16_t pf_vf_offset;
    uint16_t pf_vf_stride;

    RemotePciePort *rp0 = (RemotePciePort *)
        object_dynamic_cast(OBJECT(pf_dev), TYPE_REMOTE_PCIE_PORT);
    if (rp0) {
        /* Parent is PF0 */
        rp             = rp0;
        num_vf_bars    = rp0->sriov_num_vf_bars;
        vf_bar_size    = rp0->sriov_vf_bar_size;
        vf_bar_type    = rp0->sriov_vf_bar_type;
        vf_bar_prefetch = rp0->sriov_vf_bar_prefetch;
        pf_remote_bdf  = rp0->remote_bdf;
        pf_vf_offset   = rp0->sriov_vf_offset;
        pf_vf_stride   = rp0->sriov_vf_stride;
    } else {
        /* Parent is a companion PF */
        RemotePciePortPF *pf = REMOTE_PCIE_PORT_PF(pf_dev);
        rp             = pf->pf0;
        num_vf_bars    = pf->sriov_num_vf_bars;
        vf_bar_size    = pf->sriov_vf_bar_size;
        vf_bar_type    = pf->sriov_vf_bar_type;
        vf_bar_prefetch = pf->sriov_vf_bar_prefetch;
        pf_remote_bdf  = pf->remote_bdf;
        pf_vf_offset   = pf->sriov_vf_offset;
        pf_vf_stride   = pf->sriov_vf_stride;
    }

    /* Store routing info for fast-path BAR MMIO */
    vf->rp            = rp;
    vf->pf_remote_bdf = pf_remote_bdf;
    vf->pf_vf_offset  = pf_vf_offset;
    vf->pf_vf_stride  = pf_vf_stride;

    /* Register VF BARs that forward MMIO through the PF's socket */
    for (int i = 0; i < num_vf_bars && i < RPCIE_MAX_BARS; i++) {
        if (vf_bar_size[i] == 0) continue;

        RpcieVfBarContext *ctx = g_new0(RpcieVfBarContext, 1);
        ctx->vf      = vf;
        ctx->bar_idx = i;

        char name[32];
        snprintf(name, sizeof(name), "rpcie-vf-bar%d", i);
        memory_region_init_io(&vf->bar_mr[i], OBJECT(vf),
                              &rpcie_vf_bar_ops, ctx, name,
                              vf_bar_size[i]);

        int pci_type = PCI_BASE_ADDRESS_SPACE_MEMORY;
        if (vf_bar_type[i] == RPCIE_BAR_MEM64) {
            pci_type |= PCI_BASE_ADDRESS_MEM_TYPE_64;
        }
        if (vf_bar_prefetch && vf_bar_prefetch[i]) {
            pci_type |= PCI_BASE_ADDRESS_MEM_PREFETCH;
        }
        pci_register_bar(d, i, pci_type, &vf->bar_mr[i]);

        if (vf_bar_type[i] == RPCIE_BAR_MEM64) i++;
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
    type_register_static(&rpcie_pf_info);
    type_register_static(&rpcie_vf_info);
}

type_init(rpcie_register_types)
