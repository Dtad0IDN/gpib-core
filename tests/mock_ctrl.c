#include <string.h>

#include "mock_ctrl.h"

#define DEV_ID 0
#define CTRL_ID 1

static mock_ctrl_port_t *self(void *user) {
    return (mock_ctrl_port_t *)user;
}

static bool c_arm(void *user) {
    (void)user;
    return true;
}

static bool c_detached(void *user) {
    (void)user;
    return true;
}

static bool c_quiesced(void *user) {
    (void)user;
    return true;
}

static void c_inhibit(void *user) {
    (void)user;
}

static uint64_t c_now(void *user) {
    return self(user)->c.now_us;
}

static size_t c_transfer(void *user, gpib_dir_t dir, uint8_t *buf,
    size_t cap) {
    (void)user;
    (void)dir;
    (void)buf;
    (void)cap;
    return 0;
}

static bool c_poll(void *user, gpib_port_event_t *out) {
    (void)user;
    (void)out;
    return false;
}

static uint8_t c_holdoff;

static void ctrl_apply(mock_ctrl_t *m) {
    if (m->dav) {
        vbus_assert(&m->bus, CTRL_ID, VBL_DAV);
    } else {
        vbus_release(&m->bus, CTRL_ID, VBL_DAV);
    }
    bool rfd = (m->ah.state == GPIB_AH_AIDS || m->ah.state == GPIB_AH_ACRS);
    bool dac = (m->ah.state == GPIB_AH_AIDS || m->ah.state == GPIB_AH_AWNS);
    if (rfd) {
        vbus_release(&m->bus, CTRL_ID, VBL_NRFD);
    } else {
        vbus_assert(&m->bus, CTRL_ID, VBL_NRFD);
    }
    if (dac) {
        vbus_release(&m->bus, CTRL_ID, VBL_NDAC);
    } else {
        vbus_assert(&m->bus, CTRL_ID, VBL_NDAC);
    }
    /* ATN is asserted ONLY in the command phase; the data phases run with
     * ATN released (the device would otherwise P1-interrupt correctly). */
    if (m->st == MCS_CMD) {
        vbus_assert(&m->bus, CTRL_ID, VBL_ATN);
    } else {
        vbus_release(&m->bus, CTRL_ID, VBL_ATN);
    }
    /* Data lines: drive the current byte while a source phase offers it;
     * command bytes in MCS_CMD, data bytes in MCS_TX. The byte pointer is
     * position-based (NOT state-based) so SDYS/STRS hold it stable. */
    {
        const uint8_t *byte = NULL;
        if (m->st == MCS_CMD && m->cmd_pos < m->n_cmds) {
            byte = &m->cmds[m->cmd_pos];
        } else if (m->st == MCS_TX && m->tx_pos < m->tx_len) {
            byte = &m->tx[m->tx_pos];
        }
        if (byte != NULL &&
            (m->sh.state == GPIB_SH_SGNS || m->sh.state == GPIB_SH_SDYS ||
             m->sh.state == GPIB_SH_STRS)) {
            if (m->sh.state == GPIB_SH_SGNS) {
                vbus_drive_dio(&m->bus, CTRL_ID, *byte);
            } /* SDYS/STRS: hold the last driven value */
        } else {
            vbus_release_dio(&m->bus, CTRL_ID);
        }
    }
    if (m->st == MCS_TX && m->tx_pos + 1u == m->tx_len &&
        (m->sh.state == GPIB_SH_SDYS || m->sh.state == GPIB_SH_STRS)) {
        vbus_assert(&m->bus, CTRL_ID, VBL_EOI);
    } else {
        vbus_release(&m->bus, CTRL_ID, VBL_EOI);
    }
}

static void ctrl_round(mock_ctrl_t *m) {
    bool atn = vbus_level(&m->bus, VBL_ATN);
    bool dav = vbus_level(&m->bus, VBL_DAV);

    if (m->st == MCS_CMD) {
        gpib_sh_in_t sin;
        gpib_sh_out_t sout;
        memset(&sin, 0, sizeof(sin));
        sin.atn = atn; /* we drive it ourselves */
        sin.cacs = true; /* we ARE the active controller while commanding */
        sin.nba = (m->sh.state == GPIB_SH_SGNS) && (m->cmd_pos < m->n_cmds);
        sin.rfd = !vbus_level(&m->bus, VBL_NRFD);
        sin.dac = !vbus_level(&m->bus, VBL_NDAC);
        sin.now_us = m->now_us;
        sin.t1_us = 2u;
        gpib_sh_step(&m->sh, &sin, &sout);
        m->dav = sout.dav_requested;
        if (sout.byte_accepted) {
            m->cmd_pos++;
        }
        if (m->cmd_pos >= m->n_cmds && m->sh.state == GPIB_SH_SWNS) {
            /* Prologue done: switch to the data phase; a pure command
             * script (no TX, no RX) is finished immediately. */
            gpib_sh_init(&m->sh);
            m->sh.require_dac_low = true;
            m->tx_pos = 0;
            if (m->tx_len > 0) {
                m->st = MCS_TX;
            } else if (m->rx_expect > 0) {
                m->st = MCS_RX;
            } else {
                m->st = MCS_DONE;
                m->done = true;
            }
        }
    } else if (m->st == MCS_TX) {
        gpib_sh_in_t sin;
        gpib_sh_out_t sout;
        memset(&sin, 0, sizeof(sin));
        sin.atn = false;
        sin.tacs = true;
        sin.nba = (m->sh.state == GPIB_SH_SGNS) && (m->tx_pos < m->tx_len);
        sin.rfd = !vbus_level(&m->bus, VBL_NRFD);
        sin.dac = !vbus_level(&m->bus, VBL_NDAC);
        sin.now_us = m->now_us;
        sin.t1_us = 2u;
        gpib_sh_step(&m->sh, &sin, &sout);
        m->dav = sout.dav_requested;
        if (sout.byte_accepted && m->tx_pos + 1u < m->tx_len) {
            m->tx_pos++;
        } else if (sout.byte_accepted) {
            m->tx_pos++;
            m->st = MCS_DONE;
            m->done = true;
            m->dav = false;
        }
    } else if (m->st == MCS_RX) {
        gpib_ah_in_t ain;
        gpib_ah_out_t aout;
        memset(&ain, 0, sizeof(ain));
        ain.atn = false;
        ain.dav = dav;
        ain.lads = m->addr.lads;
        ain.lacs = m->addr.lads;
        ain.rdy = (c_holdoff == 0) && (m->rx_len < m->rx_expect);
        ain.now_us = m->now_us;
        ain.t3_us = 1u;
        gpib_ah_step(&m->ah, &ain, &aout);
        if (aout.byte_valid) {
            uint8_t dio = 0;
            if (vbus_dio_level(&m->bus, &dio) == 0 &&
                m->rx_len < MCTRL_RX_MAX) {
                m->rx[m->rx_len++] = dio;
            }
            c_holdoff = 2;
        }
        /* Leave only AFTER the byte was accepted (ACDS -> AWNS released
         * NDAC): otherwise the source never sees DAC for the last byte. */
        if (aout.accepted_event && m->rx_len >= m->rx_expect) {
            m->st = MCS_DONE;
            m->done = true;
        }
    }
    if (m->st == MCS_CMD || m->st == MCS_TX || m->st == MCS_RX) {
        /* The controller's own AH handshakes command bytes it sends? No:
         * it is the SOURCE there. Keep the AH stepping only in RX. */
    }
    if (c_holdoff > 0) {
        c_holdoff--;
    }
    ctrl_apply(m);
    vbus_tick(&m->bus);
    m->now_us++;
}

static bool c_read_lines(void *user, gpib_line_snap_t *out) {
    mock_ctrl_t *m = &self(user)->c;
    ctrl_round(m);
    memset(out, 0, sizeof(*out));
    out->dav = vbus_level(&m->bus, VBL_DAV);
    out->nrfd = vbus_level(&m->bus, VBL_NRFD);
    out->ndac = vbus_level(&m->bus, VBL_NDAC);
    out->atn = vbus_level(&m->bus, VBL_ATN);
    out->eoi = vbus_level(&m->bus, VBL_EOI);
    out->srq = vbus_level(&m->bus, VBL_SRQ);
    out->ifc = vbus_level(&m->bus, VBL_IFC);
    out->ren = vbus_level(&m->bus, VBL_REN);
    uint8_t dio = 0;
    if (vbus_dio_level(&m->bus, &dio) == 0) {
        out->dio = dio;
    }
    out->now_us = m->now_us;
    return true;
}

static void c_drive(void *user, const gpib_line_drive_t *req) {
    mock_ctrl_t *m = &self(user)->c;
    if (req->dav) {
        vbus_assert(&m->bus, DEV_ID, VBL_DAV);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_DAV);
    }
    if (req->nrfd) {
        vbus_assert(&m->bus, DEV_ID, VBL_NRFD);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_NRFD);
    }
    if (req->ndac) {
        vbus_assert(&m->bus, DEV_ID, VBL_NDAC);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_NDAC);
    }
    if (req->atn) {
        vbus_assert(&m->bus, DEV_ID, VBL_ATN);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_ATN);
    }
    if (req->ifc) {
        vbus_assert(&m->bus, DEV_ID, VBL_IFC);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_IFC);
    }
    if (req->ren) {
        vbus_assert(&m->bus, DEV_ID, VBL_REN);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_REN);
    }
    if (req->srq) {
        vbus_assert(&m->bus, DEV_ID, VBL_SRQ);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_SRQ);
    }
    if (req->eoi) {
        vbus_assert(&m->bus, DEV_ID, VBL_EOI);
    } else {
        vbus_release(&m->bus, DEV_ID, VBL_EOI);
    }
}

static void c_drive_data(void *user, const uint8_t *dio) {
    mock_ctrl_t *m = &self(user)->c;
    if (dio != NULL) {
        vbus_drive_dio(&m->bus, DEV_ID, *dio);
    } else {
        vbus_release_dio(&m->bus, DEV_ID);
    }
}

static void ctrl_set_prologue(mock_ctrl_port_t *p, uint8_t dev_addr,
    bool write) {
    mock_ctrl_t *m = &p->c;
    uint8_t own = m->addr.own_primary;
    gpib_sh_init(&m->sh);
    m->sh.require_dac_low = true;
    gpib_ah_init(&m->ah); /* fresh acceptor: stale AWNS would hold NRFD */
    gpib_addr_init(&m->addr, own); /* fresh addressing state */
    m->n_cmds = 0;
    m->cmds[m->n_cmds++] = 0x3Fu; /* UNL */
    if (write) {
        m->cmds[m->n_cmds++] = (uint8_t)(0x20u | dev_addr); /* MLA(dev) */
        m->cmds[m->n_cmds++] = (uint8_t)(0x40u | own);      /* MTA(ctrl) */
    } else {
        m->cmds[m->n_cmds++] = (uint8_t)(0x20u | own);      /* MLA(ctrl) */
        m->cmds[m->n_cmds++] = (uint8_t)(0x40u | dev_addr); /* MTA(dev) */
    }
    /* Self-tracking of the sent prologue (mirror of the sent bytes). */
    for (size_t i = 0; i < m->n_cmds; i++) {
        gpib_ifcmd_t cmd = gpib_decode_atn_byte(m->cmds[i]);
        gpib_addr_step(&m->addr, &cmd);
    }
    m->cmd_pos = 0;
    m->st = MCS_CMD;
}

void mock_ctrl_init(mock_ctrl_port_t *p, uint8_t ctrl_addr) {
    memset(p, 0, sizeof(*p));
    vbus_init(&p->c.bus);
    vbus_attach_as(&p->c.bus, DEV_ID);
    vbus_attach_as(&p->c.bus, CTRL_ID);
    gpib_sh_init(&p->c.sh);
    gpib_ah_init(&p->c.ah);
    gpib_addr_init(&p->c.addr, ctrl_addr);
    p->base.name = "mock-ctrl";
    p->base.arm = c_arm;
    p->base.safe_detached = c_detached;
    p->base.executor_quiesced = c_quiesced;
    p->base.emergency_inhibit = c_inhibit;
    p->base.now_us = c_now;
    p->base.transfer = c_transfer;
    p->base.poll_event = c_poll;
    p->base.read_lines = c_read_lines;
    p->base.drive = c_drive;
    p->base.drive_data = c_drive_data;
    p->base.user = p;
}

void mock_ctrl_write(mock_ctrl_port_t *p, uint8_t dev_addr,
    const uint8_t *data, size_t len) {
    ctrl_set_prologue(p, dev_addr, true);
    p->c.tx = data;
    p->c.tx_len = len;
    p->c.tx_pos = 0;
    p->c.done = false;
}

void mock_ctrl_read(mock_ctrl_port_t *p, uint8_t dev_addr, size_t expect) {
    ctrl_set_prologue(p, dev_addr, false);
    p->c.tx_len = 0;
    p->c.rx_len = 0;
    p->c.rx_expect = expect;
    p->c.done = false;
}

void mock_ctrl_cmds(mock_ctrl_port_t *p, const uint8_t *bytes, size_t n) {
    mock_ctrl_t *m = &p->c;
    gpib_sh_init(&m->sh);
    m->sh.require_dac_low = true;
    gpib_ah_init(&m->ah); /* fresh acceptor: stale AWNS would hold NRFD */
    m->n_cmds = (n <= sizeof(m->cmds)) ? n : sizeof(m->cmds);
    for (size_t i = 0; i < m->n_cmds; i++) {
        m->cmds[i] = bytes[i];
    }
    m->cmd_pos = 0;
    m->tx_len = 0;
    m->rx_expect = 0;
    m->st = MCS_CMD;
    m->done = false;
}
