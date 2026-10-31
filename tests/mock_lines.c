#include <string.h>

#include "mock_lines.h"

#define CORE_ID 0
#define INS_ID 1
#define INS2_ID 3
#define INS_ID 1
#define THIEF_ID 2

static mock_line_port_t *self(void *user) {
    return (mock_line_port_t *)user;
}

static bool l_arm(void *user) {
    (void)user;
    return true;
}

static bool l_detached(void *user) {
    mock_line_port_t *p = self(user);
    /* T8-01: honest evidence -- the core holds nothing on this wire. */
    return vbus_participant_clean(&p->own_bus, CORE_ID);
}

static bool l_quiesced(void *user) {
    (void)user;
    return true;
}

static void l_inhibit(void *user) {
    (void)user;
}

static uint64_t l_now(void *user) {
    return self(user)->ins.now_us;
}

static size_t l_transfer(void *user, gpib_dir_t dir, uint8_t *buf,
    size_t cap) {
    (void)user;
    (void)dir;
    (void)buf;
    (void)cap;
    return 0; /* line ports do not use the byte pump */
}

static bool l_poll(void *user, gpib_port_event_t *out) {
    (void)user;
    (void)out;
    return false;
}

/* --- instrument side ------------------------------------------------------ */

static void ins_apply(mock_instrument_t *m) {
    if (m->dav) {
        vbus_assert(m->bus, m->id, VBL_DAV);
    } else {
        vbus_release(m->bus, m->id, VBL_DAV);
    }
    /* PPAS output mapping (Table 25 / 2.9.3.3): the assigned PPR line
     * is driven TRUE iff ist == S; ist != S sends PASSIVE FALSE (the
     * driver stays released — open-collector wired-OR on real wire). */
    m->pp_driving_dio = (m->pp.state == GPIB_PP_PPAS) &&
                        (m->pp_ist == m->pp.sense);
    if (m->pp_driving_dio) {
        /* Drive our PPR bit in the PP (wired-OR) domain: other PPAS
         * devices OR theirs; a talker crossing in is a counted conflict. */
        uint8_t ppr = (uint8_t)(1u << m->pp.dio_line);
        vbus_pp_drive(m->bus, m->id, ppr);
    } else {
        vbus_pp_release(m->bus, m->id);
    }
    bool rfd = (m->ah.state == GPIB_AH_AIDS || m->ah.state == GPIB_AH_ACRS);
    bool dac = (m->ah.state == GPIB_AH_AIDS || m->ah.state == GPIB_AH_AWNS);
    if (rfd) {
        vbus_release(m->bus, m->id, VBL_NRFD);
    } else {
        vbus_assert(m->bus, m->id, VBL_NRFD);
    }
    if (dac) {
        vbus_release(m->bus, m->id, VBL_NDAC);
    } else {
        vbus_assert(m->bus, m->id, VBL_NDAC);
    }
    /* SR function output: SRQ line pulled while in SRQS. */
    if (m->sr.state == GPIB_SR_SRQS) {
        vbus_assert(m->bus, m->id, VBL_SRQ);
    } else {
        vbus_release(m->bus, m->id, VBL_SRQ);
    }
    /* talker data lines: drive only while a source byte is being offered;
     * SWNS/idle must RELEASE the drivers, otherwise the next controller
     * command phase drives DIO into a two-driver contention. */
    if (m->tx_active &&
        (m->sh.state == GPIB_SH_SGNS || m->sh.state == GPIB_SH_SDYS ||
         m->sh.state == GPIB_SH_STRS)) {
        if (m->sh.state == GPIB_SH_SGNS) {
            vbus_drive_dio(m->bus, m->id, m->tx_byte);
        } /* SDYS/STRS: hold the last driven value (stability) */
    } else if (!m->pp_driving_dio) {
        /* Do not release DIO while PPAS is driving the PPR bit: the SH
         * is idle (SIDS) during IDY, but the PP function owns DIO. */
        vbus_release_dio(m->bus, m->id);
    }
    if (m->sh.state == GPIB_SH_SIDS || m->sh.state == GPIB_SH_SIWS) {
        vbus_release(m->bus, m->id, VBL_EOI);
    }
    if (m->tx_active && m->tx_with_eoi &&
        (m->sh.state == GPIB_SH_SDYS || m->sh.state == GPIB_SH_STRS)) {
        vbus_assert(m->bus, m->id, VBL_EOI);
    } else {
        vbus_release(m->bus, m->id, VBL_EOI);
    }
}

/* per-instrument holdoff now a struct field */

static void ins_round(mock_instrument_t *m) {
    gpib_ah_in_t ain;
    gpib_ah_out_t aout;
    bool atn = vbus_level(m->bus, VBL_ATN);
    bool dav = vbus_level(m->bus, VBL_DAV);
    bool ifc = vbus_level(m->bus, VBL_IFC);
    bool eoi = vbus_level(m->bus, VBL_EOI);

    /* PP1 IDY evaluation (488.1 2.9 Fig. 11): PPSS -> PPAS on ATN&&EOI
     * (IDY) unconditionally in ist — ist==S only gates the PPR output
     * level (Table 25); PPAS -> PPSS when either line releases. */
    if (m->pp.capable && (atn || m->pp.state == GPIB_PP_PPAS)) {
        gpib_pp_in_t pin;
        gpib_pp_out_t pout;
        memset(&pin, 0, sizeof(pin));
        pin.atn = atn;
        pin.eoi = eoi;
        pin.ist = m->pp_ist;
        pin.lads = m->addr.lads;
        gpib_pp_step(&m->pp, &pin, &pout);
        (void)pout;
    }

    if (ifc) {
        /* IFC resets the INTERFACE functions within t4 (488.1): talker/
         * listener idle, serial-poll pair disarmed, SPAS aborted. The
         * device's local rsv and pending STB are DEVICE state, not
         * interface state: the SR function returns to NPRS and re-raises
         * SRQ from rsv on the next rounds (488.1 2.9). */
        m->talker.state = GPIB_T_TIDS;
        m->talker.sp = GPIB_T_SPIS;
        m->addr.lads = false;
        m->addr.tads = false;
        m->ah.state = GPIB_AH_AIDS;
        m->sh.state = GPIB_SH_SIDS;
        m->dav = false;
        gpib_sr_init(&m->sr); /* NPRS; rsv re-asserts SRQ if still armed */
        gpib_pp_init(&m->pp, m->pp.capable); /* IFC disarms PP too */
        m->pp_driving_dio = false;
        ins_apply(m);
        vbus_tick(m->bus);
        m->now_us++;
        return;
    }

    /* Acceptor: always handshakes ATN command bytes; data when addressed. */
    memset(&ain, 0, sizeof(ain));
    ain.atn = atn;
    ain.dav = dav;
    ain.lads = m->addr.lads;
    ain.lacs = m->addr.lads;
    ain.rdy = (m->holdoff == 0) &&
              (atn || m->addr.lads) &&
              (m->rxlen < MINS_RX_LOG);
    ain.now_us = m->now_us;
    ain.t3_us = 1u;
    gpib_ah_step(&m->ah, &ain, &aout);
    if (aout.byte_valid) {
        uint8_t dio = 0;
        if (vbus_dio_level(m->bus, &dio) == 0) {
            if (atn) {
                gpib_ifcmd_t cmd = gpib_decode_atn_byte(dio);
                gpib_addr_step(&m->addr, &cmd);
                /* Feed the talker FSM with the same accepted command. */
                {
                    gpib_t_in_t tin;
                    gpib_t_out_t tout;
                    memset(&tin, 0, sizeof(tin));
                    tin.atn = true;
                    tin.cmd_valid = true;
                    tin.cmd = cmd;
                    gpib_talker_step(&m->talker, &tin, &tout);
                    (void)tout;
                }
                /* DF function events: GET requires LADS, DCL universal. */
                if (cmd.cls == GPIB_CMD_GET && m->addr.lads) {
                    m->dt_triggers++;
                }
                if (cmd.cls == GPIB_CMD_DCL) {
                    m->dc_clears++;
                }
                /* PP1 (2.9): every accepted ATN byte. Group B needs the
                 * listener state as of this byte (MLA(own) set lads
                 * above via gpib_addr_step; PPC requires LADS, other
                 * PCG bytes close the configure window). */
                {
                    gpib_pp_in_t pin;
                    gpib_pp_out_t pout;
                    memset(&pin, 0, sizeof(pin));
                    pin.cmd_valid = true;
                    pin.cmd = cmd;
                    pin.atn = true;
                    pin.eoi = false;
                    pin.ist = m->pp_ist;
                    pin.lads = m->addr.lads;
                    gpib_pp_step(&m->pp, &pin, &pout);
                    (void)pout;
                }
                if (m->n_cmdlog < MINS_CMD_LOG) {
                    m->cmdlog[m->n_cmdlog++] = cmd;
                }
            } else if (m->rxlen < MINS_RX_LOG) {
                m->rx[m->rxlen++] = dio;
            }
        }
        m->holdoff = 2; /* mandatory consumer not-ready window */
    }
    if (aout.accepted_event) {
        m->accepted_total++;
    }
    if (m->holdoff > 0) {
        m->holdoff--;
    }

    /* Keep the talker's ATN view current every round (it only receives
     * commands through the AH capture path above); runs BEFORE the SR step
     * so SR sees a fresh SPAS in the same round the STB send starts. */
    {
        gpib_t_in_t tin;
        gpib_t_out_t tout;
        memset(&tin, 0, sizeof(tin));
        tin.atn = atn;
        gpib_talker_step(&m->talker, &tin, &tout);
        (void)tout;
    }

    /* SR function: rsv from the scenario, SPAS from the talker. */
    {
        gpib_sr_in_t sin;
        gpib_sr_out_t sout;
        sin.rsv = m->rsv;
        sin.spas = (m->talker.state == GPIB_T_SPAS);
        gpib_sr_step(&m->sr, &sin, &sout);
        (void)sout;
    }

    /* Talker source selection: serial poll STB first, then the scripted
     * reply (they never overlap in the scenarios). */
    m->tx_active = false;
    m->tx_with_eoi = false;
    if (!atn && m->talker.state == GPIB_T_SPAS && m->stb_pending) {
        m->tx_active = true;
        m->tx_byte = (uint8_t)(m->stb |
            ((m->sr.state == GPIB_SR_APRS) ? 0x40u : 0x00u));
    } else if (!atn &&
               (m->talker.state == GPIB_T_TADS ||
                m->talker.state == GPIB_T_TACS) &&
               m->talker.sp == GPIB_T_SPIS && m->reply_armed &&
               m->txpos < m->reply_len) {
        m->tx_active = true;
        m->tx_byte = m->reply[m->txpos];
        m->tx_with_eoi = (m->txpos + 1u == m->reply_len);
    }
    if (m->tx_active) {
        gpib_sh_in_t sin;
        gpib_sh_out_t sout;
        memset(&sin, 0, sizeof(sin));
        sin.atn = atn;
        sin.tacs = !atn;
        sin.nba = (m->sh.state == GPIB_SH_SGNS);
        sin.rfd = !vbus_level(m->bus, VBL_NRFD);
        sin.dac = !vbus_level(m->bus, VBL_NDAC);
        sin.now_us = m->now_us;
        sin.t1_us = 2u;
        gpib_sh_step(&m->sh, &sin, &sout);
        m->dav = sout.dav_requested;
        if (sout.byte_accepted) {
            if (m->talker.state == GPIB_T_SPAS) {
                m->stb_pending = false; /* one STB per poll */
                m->rsv = false;         /* request serviced */
            } else {
                m->txpos++;
                if (m->txpos >= m->reply_len) {
                    m->reply_armed = false; /* one-shot */
                }
            }
        }
    } else {
        m->dav = false;
    }
    ins_apply(m);
}

/* --- port callbacks ------------------------------------------------------- */

static bool l_read_lines(void *user, gpib_line_snap_t *out) {
    mock_line_port_t *p = self(user);
    mock_instrument_t *m = &p->ins;
    ins_round(m); /* the device advances with every core observation */
    if (p->has_second) {
        ins_round(&p->ins2);
    }
    vbus_tick(&p->own_bus); /* T16: the wire belongs to THIS fixture */
    p->ins.now_us++;
    p->ins2.now_us++;
    memset(out, 0, sizeof(*out));
    out->dav = vbus_level(m->bus, VBL_DAV);
    out->nrfd = vbus_level(m->bus, VBL_NRFD);
    out->ndac = vbus_level(m->bus, VBL_NDAC);
    out->atn = vbus_level(m->bus, VBL_ATN);
    out->eoi = vbus_level(m->bus, VBL_EOI);
    out->srq = vbus_level(m->bus, VBL_SRQ);
    out->ifc = vbus_level(m->bus, VBL_IFC);
    out->ren = vbus_level(m->bus, VBL_REN);
    uint8_t dio = 0;
    if (vbus_dio_level(m->bus, &dio) == 0) {
        out->dio = dio;
    }
    out->now_us = m->now_us;
    return true;
}

static void l_drive(void *user, const gpib_line_drive_t *req) {
    mock_instrument_t *m = &self(user)->ins;
    if (req->dav) {
        vbus_assert(m->bus, CORE_ID, VBL_DAV);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_DAV);
    }
    if (req->nrfd) {
        vbus_assert(m->bus, CORE_ID, VBL_NRFD);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_NRFD);
    }
    if (req->ndac) {
        vbus_assert(m->bus, CORE_ID, VBL_NDAC);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_NDAC);
    }
    if (req->atn) {
        vbus_assert(m->bus, CORE_ID, VBL_ATN);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_ATN);
    }
    if (req->ifc) {
        vbus_assert(m->bus, CORE_ID, VBL_IFC);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_IFC);
    }
    if (req->ren) {
        vbus_assert(m->bus, CORE_ID, VBL_REN);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_REN);
    }
    if (req->srq) {
        vbus_assert(m->bus, CORE_ID, VBL_SRQ);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_SRQ);
    }
    if (req->eoi) {
        vbus_assert(m->bus, CORE_ID, VBL_EOI);
    } else {
        vbus_release(m->bus, CORE_ID, VBL_EOI);
    }
}

static void l_drive_data(void *user, const uint8_t *dio) {
    mock_instrument_t *m = &self(user)->ins;
    (void)m; /* CORE_ID is the fixed core participant on this bus */
    if (dio != NULL) {
        vbus_drive_dio(self(user)->ins.bus, CORE_ID, *dio);
    } else {
        vbus_release_dio(self(user)->ins.bus, CORE_ID);
    }
}

void mock_lines_init(mock_line_port_t *p) {
    memset(p, 0, sizeof(*p));
    p->ins.id = INS_ID;
    p->ins.bus = &p->own_bus; /* T16: per-fixture wire */
    vbus_init(&p->own_bus);
    vbus_attach_as(&p->own_bus, CORE_ID);
    vbus_attach_as(&p->own_bus, INS_ID);
    gpib_sh_init(&p->ins.sh);
    gpib_ah_init(&p->ins.ah);
    gpib_addr_init(&p->ins.addr, 6); /* the instrument lives at address 6 */
    gpib_talker_init(&p->ins.talker, 6, true, false); /* T2-style */
    gpib_sr_init(&p->ins.sr);
    p->base.name = "mock-lines";
    p->base.arm = l_arm;
    p->base.safe_detached = l_detached;
    p->base.executor_quiesced = l_quiesced;
    p->base.emergency_inhibit = l_inhibit;
    p->base.now_us = l_now;
    p->base.transfer = l_transfer;
    p->base.poll_event = l_poll;
    p->base.read_lines = l_read_lines;
    p->base.drive = l_drive;
    p->base.drive_data = l_drive_data;
    p->base.user = p;
}

void mock_lines_add_second(mock_line_port_t *p, uint8_t addr) {
    mock_instrument_t *s = &p->ins2;
    memset(s, 0, sizeof(*s));
    s->id = INS2_ID;
    s->bus = p->ins.bus; /* share the same wired-OR bus */
    gpib_sh_init(&s->sh);
    gpib_ah_init(&s->ah);
    gpib_addr_init(&s->addr, addr);
    gpib_talker_init(&s->talker, addr, true, false);
    gpib_sr_init(&s->sr);
    vbus_attach_as(s->bus, INS2_ID);
    p->has_second = true;
}

void mock_lines_arm_reply(mock_line_port_t *p, const uint8_t *data,
    size_t len) {
    p->ins.reply = data;
    p->ins.reply_len = len;
    p->ins.txpos = 0;
    p->ins.reply_armed = true;
    gpib_sh_init(&p->ins.sh); /* fresh source FSM for the new reply */
}

void mock_lines_set_stb(mock_line_port_t *p, uint8_t stb, bool rsv) {
    p->ins.stb = stb;
    p->ins.rsv = rsv;
    p->ins.stb_pending = true;
    gpib_sh_init(&p->ins.sh); /* fresh source FSM for the STB byte */
}

void mock_lines_set_stb_second(mock_line_port_t *p, uint8_t stb, bool rsv) {
    if (p->has_second) {
        p->ins2.stb = stb;
        p->ins2.rsv = rsv;
        p->ins2.stb_pending = true;
        gpib_sh_init(&p->ins2.sh);
    }
}

void mock_lines_steal_atn(mock_line_port_t *p, bool steal) {
    /* T16: attach the thief participant to THIS fixture's wire. */
    vbus_attach_as(&p->own_bus, THIEF_ID);
    if (steal) {
        vbus_assert(&p->own_bus, THIEF_ID, VBL_ATN);
    } else {
        vbus_release(&p->own_bus, THIEF_ID, VBL_ATN);
    }
}
