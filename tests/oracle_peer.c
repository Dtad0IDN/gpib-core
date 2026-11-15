#include <string.h>

#include "oracle_peer.h"

/* T04 independent oracle peer. Everything below is written from the
 * wire rules directly; it must NOT import src/proto.c machinery. */

static bool lvl(oracle_peer_t *p, vbus_line_t line) {
    return vbus_level(p->bus, line);
}

static void o_set(oracle_peer_t *p, vbus_line_t line, bool on) {
    if (on) {
        vbus_assert(p->bus, p->self_id, line);
    } else {
        vbus_release(p->bus, p->self_id, line);
    }
}

/* --- independent command interpretation (own table) ---------------------- */

static void log_cmd(oracle_peer_t *p, oracle_cmd_t c, uint8_t raw,
    uint8_t arg) {
    if (p->n_cmds < ORACLE_CMD_LOG_MAX) {
        p->cmds[p->n_cmds].cmd = c;
        p->cmds[p->n_cmds].raw = raw;
        p->cmds[p->n_cmds].arg = arg;
        p->n_cmds++;
    }
}

static void handle_command(oracle_peer_t *p, uint8_t b) {
    /* Address ranges and fixed codes per the wire encoding. Any PRIMARY
     * command other than PPC closes the parallel-poll configure window. */
    if (b >= 0x20u && b <= 0x3Eu) {
        uint8_t arg = (uint8_t)(b - 0x20u);
        p->listen = (arg == p->addr);
        p->pp_window = false;
        log_cmd(p, OC_MLA, b, arg);
    } else if (b == 0x3Fu) {
        p->listen = false;
        p->pp_window = false;
        log_cmd(p, OC_UNL, b, 0);
    } else if (b >= 0x40u && b <= 0x5Eu) {
        uint8_t arg = (uint8_t)(b - 0x40u);
        p->talk = (arg == p->addr);
        p->pp_window = false;
        log_cmd(p, OC_MTA, b, arg);
    } else if (b == 0x5Fu) {
        p->talk = false;
        p->pp_window = false;
        log_cmd(p, OC_UNT, b, 0);
    } else if (b == 0x18u) {
        p->spe = true;
        log_cmd(p, OC_SPE, b, 0);
    } else if (b == 0x19u) {
        p->spe = false;
        p->stb_sent = false; /* re-arm: the next SPE cycle may poll again */
        log_cmd(p, OC_SPD, b, 0);
    } else if (b == 0x14u) {
        p->dcl_count++;
        log_cmd(p, OC_DCL, b, 0);
    } else if (b == 0x05u) {
        if (p->listen) {
            p->pp_window = true; /* configure window opens for the listener */
        }
        log_cmd(p, OC_PPC, b, 0);
    } else if (b == 0x15u) {
        p->pp_configured = false;
        p->pp_window = false;
        log_cmd(p, OC_PPU, b, 0);
    } else if (b >= 0x60u) {
        uint8_t arg = (uint8_t)(b & 0x1Fu);
        if (p->pp_window) {
            if ((b & 0x10u) != 0u) {
                p->pp_configured = false; /* PPD: parallel poll disable */
            } else {
                p->pp_configured = true; /* PPE: arm with S and P1-P3 */
                p->pp_sense = ((b & 0x08u) != 0u);
                p->pp_line = (uint8_t)(b & 0x07u);
            }
            p->pp_window = false;
        }
        log_cmd(p, OC_SEC, b, arg);
    } else {
        switch (b) {
        case 0x01u:
            if (p->listen) {
                p->gtl_count++;
            }
            log_cmd(p, OC_GTL, b, 0);
            break;
        case 0x04u:
            if (p->listen) {
                p->sdc_count++;
            }
            log_cmd(p, OC_SDC, b, 0);
            break;
        case 0x08u:
            if (p->listen) {
                p->get_count++;
            }
            log_cmd(p, OC_GET, b, 0);
            break;
        case 0x09u:
            log_cmd(p, OC_TCT, b, 0);
            break;
        case 0x11u:
            log_cmd(p, OC_LLO, b, 0);
            break;
        default:
            log_cmd(p, OC_OTHER, b, 0);
            break;
        }
        p->pp_window = false;
    }
}

/* --- hand-written acceptor ------------------------------------------------ */

static void acceptor_round(oracle_peer_t *p, bool atn, bool dav,
    uint8_t dio) {
    bool participating = atn || p->listen;
    if (!participating) {
        o_set(p, VBL_NRFD, false);
        o_set(p, VBL_NDAC, false);
        p->byte_taken = false;
        return;
    }
    bool room = atn || (p->rx_len < ORACLE_RX_MAX);
    if (p->hold_byte && !atn && dav && !p->byte_taken) {
        /* O15: deliberately stall INSIDE the byte cycle: NRFD released
         * (was ready), NDAC held -- the source waits with DAV asserted,
         * the byte is provably in flight. */
        o_set(p, VBL_NRFD, false);
        o_set(p, VBL_NDAC, true);
        return;
    }
    if (dav && !p->byte_taken) {
        if (room) {
            if (atn) {
                handle_command(p, dio);
            } else {
                p->rx[p->rx_len++] = dio;
            }
            p->byte_taken = true;
            o_set(p, VBL_NDAC, false); /* accept pulse */
            o_set(p, VBL_NRFD, !room ? true : false); /* ready for next */
        } else {
            o_set(p, VBL_NRFD, true); /* backpressure */
            o_set(p, VBL_NDAC, true);
        }
        return;
    }
    if (p->byte_taken && !dav) {
        p->byte_taken = false; /* source saw the accept: byte cycle done */
    }
    o_set(p, VBL_NDAC, true); /* nothing pending: not-accepted idle level */
    /* Final audit 7.2: a STRICT 488.1 acceptor holds NRFD (not ready)
     * while the just-accepted byte's DAV is still asserted (the AWNS
     * output function: rfd released only in AIDS/ACRS). The lenient
     * version (release both simultaneously) masked source timing
     * errors. Readiness for the NEXT byte resumes once DAV drops. */
    o_set(p, VBL_NRFD, !(room && !dav));
}

/* --- hand-written talker --------------------------------------------------- */

static void talker_round(oracle_peer_t *p, bool atn, bool nrfd, bool ndac) {
    if (p->tx_stall) {
        /* O6 fault injection: a wedged source never asserts DAV; the
         * core under test must end the RECEIVE through its deadline. */
        o_set(p, VBL_DAV, false);
        o_set(p, VBL_EOI, false);
        vbus_release_dio(p->bus, p->self_id);
        p->tx_phase = 0;
        return;
    }
    bool want = p->talk && !atn &&
                ((p->spe && !p->stb_sent) ||
                 (p->tx_armed && p->tx_pos < p->tx_len));
    if (!want) {
        o_set(p, VBL_DAV, false);
        o_set(p, VBL_EOI, false);
        if (!(p->pp_configured && atn)) {
            vbus_release_dio(p->bus, p->self_id);
        }
        if (p->tx_phase != 0) {
            p->tx_phase = 0;
        }
        return;
    }
    bool stb_mode = p->spe && !p->stb_sent;
    size_t len = stb_mode ? 1u : p->tx_len;
    size_t pos = stb_mode ? 0u : p->tx_pos;
    /* Rev6 §5.3: eoi_last is honored -- a reply without END is testable.
     * The serial-poll STB always carries EOI (the O3 16.2.6 regression). */
    bool eoi_on = (pos + 1u == len) && (stb_mode || p->tx_eoi_last);
    switch (p->tx_phase) {
    case 0: /* wait for ready (NRFD released) and not-accepted idle */
        o_set(p, VBL_DAV, false);
        o_set(p, VBL_EOI, false);
        if (!nrfd && ndac) {
            uint8_t b = stb_mode
                ? (uint8_t)(p->stb | (p->rsv ? 0x40u : 0x00u))
                : p->tx[pos];
            vbus_drive_dio(p->bus, p->self_id, b);
            p->tx_phase = 1;
        }
        break;
    case 1: /* settle, then assert DAV (EOI together with the last byte) */
        o_set(p, VBL_DAV, true);
        o_set(p, VBL_EOI, eoi_on);
        p->tx_phase = 2;
        break;
    case 2: /* hold; the acceptor signals with NDAC released */
        o_set(p, VBL_DAV, true);
        o_set(p, VBL_EOI, eoi_on);
        if (!ndac) {
            o_set(p, VBL_DAV, false);
            o_set(p, VBL_EOI, false);
            /* 488.1: the source may drop its DIO drivers as soon as the
             * byte is accepted (NDAC released) -- holding them until
             * NDAC re-asserts opens a two-talker window against the
             * controller's next command byte (found by the conflict
             * invariant, Rev6 §5.1). */
            vbus_release_dio(p->bus, p->self_id);
            if (stb_mode) {
                p->stb_sent = true;
                p->rsv = false; /* serviced: SRQ clears with the STB */
            } else {
                p->tx_pos++;
            }
            p->tx_phase = 3;
        }
        break;
    case 3: /* wait for the acceptor's NDAC to return */
        o_set(p, VBL_DAV, false);
        o_set(p, VBL_EOI, false);
        if (ndac) {
            p->tx_phase = 0;
        }
        break;
    default:
        p->tx_phase = 0;
        break;
    }
}

/* --- port callbacks -------------------------------------------------------- */

static bool o_read_lines(void *user, gpib_line_snap_t *snap);

bool oracle_port_read_lines(void *user, gpib_line_snap_t *snap) {
    return o_read_lines(user, snap);
}

static bool o_read_lines(void *user, gpib_line_snap_t *snap) {
    oracle_peer_t *p = (oracle_peer_t *)user;
    if (p->read_fail) {
        /* Rev10 §7 fault injection: the port cannot sample -- the core
         * must cope (no progress, no crash) until the port recovers. */
        return false;
    }
    bool atn = lvl(p, VBL_ATN);
    bool eoi = lvl(p, VBL_EOI);
    bool dav = lvl(p, VBL_DAV);
    bool nrfd = lvl(p, VBL_NRFD);
    bool ndac = lvl(p, VBL_NDAC);
    bool ifc = lvl(p, VBL_IFC);
    p->ifc_seen_round = ifc; /* one round of legality exemptions */
    bool ren = lvl(p, VBL_REN);
    bool srq = lvl(p, VBL_SRQ);
    uint8_t dio = 0;
    {
        int derr = vbus_dio_level(p->bus, &dio);
        /* Rev10 §5.3: DAV high with an unresolved DIO contention is an
         * immediate invariant breach -- EXCEPT during an IFC pulse: IFC
         * legitimately aborts byte cycles mid-flight (the reset releases
         * the data drivers while the source's DAV edge is still up),
         * which is normal wire behavior, not a fault (found by the
         * chaos stress under an injected IFC). */
        if (derr != 0 && dav && !ifc) {
            p->violations++;
            p->viol_conflict++;
        }
    }

    if (ifc) {
        p->ifc_seen_round = true; /* AUD-11/derr exemptions this round */
        /* Wire rule (488.1 3.3): IFC clears the INTERFACE functions --
         * addressing, poll arm, and any in-flight HANDSHAKE (drivers
         * MUST come off or the controller's next command phase contends
         * DIO with our stale byte). The application's response DATA
         * survives: a re-addressed talker resumes from tx_pos, exactly
         * like a real instrument's output queue (found by the chaos
         * stress under an injected IFC; arming-then-start ordering in
         * the directed tests caught the over-eager abandon). */
        p->listen = false;
        p->talk = false;
        p->spe = false;
        p->byte_taken = false;
        p->tx_phase = 0;
        p->pp_configured = false;
        p->pp_window = false;
        p->saw_ifc = true;
        vbus_release(p->bus, p->self_id, VBL_DAV);
        vbus_release(p->bus, p->self_id, VBL_EOI);
        vbus_release_dio(p->bus, p->self_id);
    }
    if (ren) {
        p->saw_ren = true;
    }

    acceptor_round(p, atn, dav, dio);
    talker_round(p, atn, nrfd, ndac);

    /* Parallel poll responder: a direct rule reading -- during IDY a
     * configured responder drives its PPR line (wired-OR domain) iff
     * ist == S. */
    if (p->pp_configured && atn && eoi && (p->pp_ist == p->pp_sense)) {
        vbus_pp_drive(p->bus, p->self_id, (uint8_t)(1u << p->pp_line));
    } else {
        vbus_pp_release(p->bus, p->self_id);
    }

    /* SRQ level: the service request stands until the poll services it. */
    p->srq_asserted = p->rsv;
    o_set(p, VBL_SRQ, p->srq_asserted);

    vbus_tick(p->bus);
    if (!p->freeze_clock) {
        p->now_us++; /* O14 freezes the snapshot time: every FSM timing
                      * window (T1/T3/probe/deadline) stops elapsing */
    }

    /* The CORE observes the post-tick levels: everything driven this
     * round (ours included) is visible in the same snapshot. */
    atn = lvl(p, VBL_ATN);
    eoi = lvl(p, VBL_EOI);
    dav = lvl(p, VBL_DAV);
    nrfd = lvl(p, VBL_NRFD);
    ndac = lvl(p, VBL_NDAC);
    ifc = lvl(p, VBL_IFC);
    ren = lvl(p, VBL_REN);
    srq = lvl(p, VBL_SRQ);
    dio = 0;
    (void)vbus_dio_level(p->bus, &dio);

    snap->atn = atn;
    snap->eoi = eoi;
    snap->dav = dav;
    snap->nrfd = nrfd;
    snap->ndac = ndac;
    snap->ifc = ifc;
    snap->ren = ren;
    snap->srq = srq;
    snap->dio = dio;
    snap->now_us = p->now_us;

    /* Waveform trace (what the CORE observed this round) + invariants.
     * Rev6 §5.1/§5.4: any NEW vbus conflict is a violation; trace ring
     * exhaustion is flagged, never silently wrapped. */
    if (p->bus->conflicts != p->conflicts_prev) {
        p->violations++;
        p->viol_conflict++;
        p->conflicts_prev = p->bus->conflicts;
    }
    if (p->trace_enabled) {
        if (p->trace_len < ORACLE_TRACE_MAX) {
            oracle_trace_t *t = &p->trace[p->trace_len++];
            t->atn = atn ? 1 : 0;
            t->eoi = eoi ? 1 : 0;
            t->dav = dav ? 1 : 0;
            t->nrfd = nrfd ? 1 : 0;
            t->ndac = ndac ? 1 : 0;
            t->ifc = ifc ? 1 : 0;
            t->ren = ren ? 1 : 0;
            t->srq = srq ? 1 : 0;
            t->dio = dio;
        } else {
            p->trace_overflow = true;
        }
    }
    p->dav_prev = dav;
    /* ring-independent level for INV2: the BUS DAV (for acceptor-side
     * stability checks); o_drive maintains the core's OWN DAV level
     * separately for the AUD-11 release check. */
    return true;
}

static void o_drive(void *user, const gpib_line_drive_t *req) {
    oracle_peer_t *p = (oracle_peer_t *)user;
    /* Rev10 §4.3: journal EVERY drive call -- intra-round glitches
     * (forbidden release + correct value inside one round) land here
     * even though the per-round snapshot ring only sees the final
     * state of the round. Gated together with the trace ring: wedged
     * scenarios (O14) opt out of diagnostic recording, not of the
     * online invariants. */
    if (p->trace_enabled) {
        if (p->dr_n < ORACLE_DRIVE_JOURNAL_MAX) {
            oracle_drive_rec_t *r = &p->dr_journal[p->dr_n++];
            r->atn = req->atn ? 1 : 0;
            r->eoi = req->eoi ? 1 : 0;
            r->dav = req->dav ? 1 : 0;
            r->nrfd = req->nrfd ? 1 : 0;
            r->ndac = req->ndac ? 1 : 0;
            r->ifc = req->ifc ? 1 : 0;
            r->ren = req->ren ? 1 : 0;
            r->srq = req->srq ? 1 : 0;
            r->round = (uint16_t)p->trace_len;
        } else {
            p->dr_overflow = true;
        }
    }
    /* INV (Rev6 §5.2): ATN must not flip while DAV is held -- releasing
     * BOTH in one drive is legal (end of a command phase); holding DAV
     * across an ATN change is a mid-byte phase break. */
    if (p->trace_len > 0u && p->trace[p->trace_len - 1u].dav &&
        p->core_atn_prev != req->atn && req->dav) {
        p->violations++;
        p->viol_atn++;
    }
    p->core_atn_prev = req->atn;
    /* AUD-11: track the core's OWN driven DAV (the snapshot lags by a
     * round; the release-after-own-DAV-drop must not false-positive). */
    p->last_dav_lvl = req->dav;
    p->have_last_lvl = true;
    if (req->atn) {
        vbus_assert(p->bus, p->core_id, VBL_ATN);
    } else {
        vbus_release(p->bus, p->core_id, VBL_ATN);
    }
    if (req->eoi) {
        vbus_assert(p->bus, p->core_id, VBL_EOI);
    } else {
        vbus_release(p->bus, p->core_id, VBL_EOI);
    }
    if (req->dav) {
        vbus_assert(p->bus, p->core_id, VBL_DAV);
    } else {
        vbus_release(p->bus, p->core_id, VBL_DAV);
    }
    if (req->nrfd) {
        vbus_assert(p->bus, p->core_id, VBL_NRFD);
    } else {
        vbus_release(p->bus, p->core_id, VBL_NRFD);
    }
    if (req->ndac) {
        vbus_assert(p->bus, p->core_id, VBL_NDAC);
    } else {
        vbus_release(p->bus, p->core_id, VBL_NDAC);
    }
    if (req->ifc) {
        vbus_assert(p->bus, p->core_id, VBL_IFC);
    } else {
        vbus_release(p->bus, p->core_id, VBL_IFC);
    }
    if (req->ren) {
        vbus_assert(p->bus, p->core_id, VBL_REN);
    } else {
        vbus_release(p->bus, p->core_id, VBL_REN);
    }
    if (req->srq) {
        vbus_assert(p->bus, p->core_id, VBL_SRQ);
    } else {
        vbus_release(p->bus, p->core_id, VBL_SRQ);
    }
}

static void o_drive_data(void *user, const uint8_t *value) {
    oracle_peer_t *p = (oracle_peer_t *)user;
    if (value == NULL) {
        /* AUD-11: DIO drivers released while DAV is still asserted --
         * the acceptor may be mid-latch. DAV must fall first.
         * EXCEPT during an IFC pulse: the reset legitimately aborts
         * byte cycles mid-flight (same exemption as the DIO-read
         * check; found by the chaos stress under an injected IFC). */
        if (p->have_last_lvl && p->last_dav_lvl &&
            p->core_dio_prev != 0xFFu && !p->ifc_seen_round) {
            p->violations++;
            p->viol_conflict++;
        }
        vbus_release_dio(p->bus, p->core_id);
        return;
    }
    /* INV2: the source must hold DIO stable while DAV is asserted.
     * Rev10 §6.4: uses the ring-independent last levels, so the check
     * survives tracing-disabled wedged scenarios. */
    if (p->have_last_lvl && p->last_dav_lvl &&
        p->core_dio_prev != 0xFFu && *value != p->core_dio_prev) {
        p->violations++;
    }
    p->core_dio_prev = *value;
    vbus_drive_dio(p->bus, p->core_id, *value);
}

static bool o_poll_event(void *user, gpib_port_event_t *ev) {
    (void)user;
    (void)ev;
    return false;
}

static uint64_t o_now_us(void *user) {
    oracle_peer_t *p = (oracle_peer_t *)user;
    return p->now_us;
}

static bool o_arm(void *user) {
    (void)user;
    return true; /* nothing to arm on the simulated wire */
}

static bool o_detached(void *user) {
    /* T8-01: PROOF, not a stub -- true only when the core participant
     * asserts nothing on the wire. The core earns this by releasing at
     * the STOPPING->DRAINING boundary; a lying port would certify an
     * electrically live detach. */
    oracle_peer_t *p = (oracle_peer_t *)user;
    return vbus_participant_clean(p->bus, p->core_id);
}

bool oracle_port_safe_detached(void *user) {
    /* Public alias for tests that swap safe_detached and restore it. */
    return o_detached(user);
}

static bool o_quiesced(void *user) {
    oracle_peer_t *p = (oracle_peer_t *)user;
    return !p->quiesce_fail; /* N11-06: injectable delayed quiescence */
}

static void o_inhibit(void *user) {
    oracle_peer_t *p = (oracle_peer_t *)user;
    /* H5d (final audit): emergency_inhibit MUST exist -- the first call
     * of gpib_emergency_stop used to jump through a NULL pointer. The
     * electrical effect: every line and DIO driver of the core
     * participant released on the wired-OR bus. */
    vbus_release(p->bus, p->core_id, VBL_ATN);
    vbus_release(p->bus, p->core_id, VBL_EOI);
    vbus_release(p->bus, p->core_id, VBL_DAV);
    vbus_release(p->bus, p->core_id, VBL_NRFD);
    vbus_release(p->bus, p->core_id, VBL_NDAC);
    vbus_release(p->bus, p->core_id, VBL_IFC);
    vbus_release(p->bus, p->core_id, VBL_REN);
    vbus_release(p->bus, p->core_id, VBL_SRQ);
    vbus_release_dio(p->bus, p->core_id);
    vbus_pp_release(p->bus, p->core_id);
}

/* --- setup helpers --------------------------------------------------------- */

void oracle_init(oracle_peer_t *p, vbus_t *bus, int core_id, uint8_t addr) {
    memset(p, 0, sizeof(*p));
    p->bus = bus;
    p->core_id = core_id;
    p->self_id = 1;
    p->addr = addr;
    p->base.user = p;
    p->base.read_lines = o_read_lines;
    p->base.drive = o_drive;
    p->base.drive_data = o_drive_data;
    p->base.poll_event = o_poll_event;
    p->base.now_us = o_now_us;
    p->base.arm = o_arm;
    p->base.safe_detached = o_detached;
    p->base.executor_quiesced = o_quiesced;
    p->base.emergency_inhibit = o_inhibit;
    p->core_dio_prev = 0xFFu;
    p->trace_enabled = true; /* the ring records by default (O14 opts out) */
    vbus_attach_as(bus, p->self_id);
}

void oracle_arm_reply(oracle_peer_t *p, const uint8_t *data, size_t n,
    bool eoi_last) {
    p->tx = data;
    p->tx_len = n;
    p->tx_pos = 0;
    p->tx_armed = true;
    p->tx_eoi_last = eoi_last;
}

void oracle_stall_talker(oracle_peer_t *p, bool stall) {
    p->tx_stall = stall;
}

void oracle_set_stb(oracle_peer_t *p, uint8_t stb, bool rsv) {
    p->stb = stb;
    p->rsv = rsv;
}

void oracle_set_pp(oracle_peer_t *p, bool capable, bool ist) {
    p->pp_capable = capable;
    p->pp_ist = ist;
    if (!capable) {
        p->pp_configured = false;
    }
}
