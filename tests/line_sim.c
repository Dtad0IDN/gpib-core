#include "line_sim.h"

#include <string.h>

void line_bus_init(line_bus_t *lb) {
    vbus_init(&lb->bus);
    lb->now_us = 0;
}

void sh_peer_init(sh_peer_t *p, int id, const uint8_t *data, size_t len,
    uint32_t t1_us, bool data_mode) {
    memset(p, 0, sizeof(*p));
    p->id = id;
    gpib_sh_init(&p->sh);
    p->data = data;
    p->len = len;
    p->t1_us = t1_us;
    p->data_mode = data_mode;
}

void ah_peer_init(ah_peer_t *p, int id, uint8_t *rx, size_t cap,
    uint32_t t3_us, uint32_t ready_delay_us, bool listen) {
    memset(p, 0, sizeof(*p));
    p->id = id;
    gpib_ah_init(&p->ah);
    p->rx = rx;
    p->cap = cap;
    p->t3_us = t3_us;
    p->ready_delay_us = ready_delay_us;
    p->listen = listen;
}

/* Logical level helpers: IEEE TRUE = wire LOW (asserted). */
static bool atn_true(const line_bus_t *lb) {
    return vbus_level(&lb->bus, VBL_ATN);
}

static bool dav_true(const line_bus_t *lb) {
    return vbus_level(&lb->bus, VBL_DAV);
}

static bool rfd_true(const line_bus_t *lb) {
    return !vbus_level(&lb->bus, VBL_NRFD); /* NRFD low = not ready */
}

static bool dac_true(const line_bus_t *lb) {
    return !vbus_level(&lb->bus, VBL_NDAC); /* NDAC low = not accepted */
}

static void sh_apply(sh_peer_t *p, line_bus_t *lb) {
    if (p->dav_requested) {
        vbus_assert(&lb->bus, p->id, VBL_DAV);
    } else {
        vbus_release(&lb->bus, p->id, VBL_DAV);
    }
    if (!p->data_mode) {
        /* Command mode: the controller source holds ATN asserted for the
         * whole command phase of its own transfer. */
        if (!p->role_holdoff) {
            vbus_assert(&lb->bus, p->id, VBL_ATN);
        } else {
            vbus_release(&lb->bus, p->id, VBL_ATN);
        }
    }
    /* Byte stability: DIO is (re)driven only when the FSM permits changes;
     * in SDYS/STRS the last driven value stays untouched. */
    bool may = (p->sh.state == GPIB_SH_SIDS || p->sh.state == GPIB_SH_SGNS ||
                p->sh.state == GPIB_SH_SWNS || p->sh.state == GPIB_SH_SIWS);
    if (may) {
        if (p->pos < p->len && p->sh.state == GPIB_SH_SGNS) {
            vbus_drive_dio(&lb->bus, p->id, p->data[p->pos]);
        }
    }
    if (p->sh.state == GPIB_SH_SIDS || p->sh.state == GPIB_SH_SIWS) {
        vbus_release_dio(&lb->bus, p->id);
    }
}

static void sh_round(sh_peer_t *p, line_bus_t *lb) {
    gpib_sh_in_t in;
    gpib_sh_out_t out;
    bool atn = atn_true(lb);
    memset(&in, 0, sizeof(in));
    in.atn = atn;
    if (p->data_mode) {
        in.tacs = !atn && !p->role_holdoff;
    } else {
        in.cacs = atn && !p->role_holdoff;
    }
    /* Producer protocol (§2.3.5): nba rises only while the source sits in
     * SGNS offering the byte, and drops once the source latched it into
     * SDYS; SWNS -> SGNS then requires !nba, which the model provides. */
    in.nba = (p->sh.state == GPIB_SH_SGNS) && (p->pos < p->len);
    in.rfd = rfd_true(lb);
    in.dac = dac_true(lb);
    in.now_us = lb->now_us;
    in.t1_us = p->t1_us;
    gpib_sh_step(&p->sh, &in, &out);
    p->dav_requested = out.dav_requested;
    if (out.byte_accepted) {
        p->pos++;
        p->sent++;
    }
    sh_apply(p, lb);
}

static void ah_apply(ah_peer_t *p, line_bus_t *lb) {
    /* NRFD: RFD=1 -> release (high); RFD=0 -> assert (low). */
    bool rfd = (p->ah.state == GPIB_AH_AIDS || p->ah.state == GPIB_AH_ACRS);
    bool dac = (p->ah.state == GPIB_AH_AIDS || p->ah.state == GPIB_AH_AWNS);
    if (rfd) {
        vbus_release(&lb->bus, p->id, VBL_NRFD);
    } else {
        vbus_assert(&lb->bus, p->id, VBL_NRFD);
    }
    if (dac) {
        vbus_release(&lb->bus, p->id, VBL_NDAC);
    } else {
        vbus_assert(&lb->bus, p->id, VBL_NDAC);
    }
}

static void ah_round(ah_peer_t *p, line_bus_t *lb) {
    gpib_ah_in_t in;
    gpib_ah_out_t out;
    memset(&in, 0, sizeof(in));
    in.atn = atn_true(lb);
    in.dav = dav_true(lb);
    in.lads = p->listen;
    in.lacs = p->listen;
    /* Consumer readiness protocol: after capturing a byte the consumer is
     * NOT ready for at least one round (the ACDS -> AWNS acceptance gate),
     * then stays unready for its processing delay, then becomes ready. */
    in.rdy = (lb->now_us >= p->next_rdy_us);
    in.now_us = lb->now_us;
    in.t3_us = p->t3_us;
    gpib_ah_step(&p->ah, &in, &out);
    if (out.byte_valid) {
        uint8_t dio = 0;
        if (vbus_dio_level(&lb->bus, &dio) == 0 && p->len < p->cap) {
            p->rx[p->len++] = dio;
        }
        p->next_rdy_us = lb->now_us + 2u + (uint64_t)p->ready_delay_us;
    }
    if (out.accepted_event) {
        p->accepted++;
    }
    ah_apply(p, lb);
}

bool line_bus_round(line_bus_t *lb, sh_peer_t *src, ah_peer_t *acc,
    size_t n_acc) {
    /* Lazily attach every participant with its fixed id: an unattached
     * participant's assert/release calls are silently ignored by the bus. */
    vbus_attach_as(&lb->bus, src->id);
    for (size_t i = 0; i < n_acc; i++) {
        vbus_attach_as(&lb->bus, acc[i].id);
    }
    /* Phase 1: everyone reads the settled levels of the previous round and
     * advances its FSM. Phase 2: everyone applies its new line requests. */
    sh_round(src, lb);
    for (size_t i = 0; i < n_acc; i++) {
        ah_round(&acc[i], lb);
    }
    vbus_tick(&lb->bus);
    lb->now_us++;
    if (lb->bus.conflicts != 0) {
        return false; /* contention on the simulated wires */
    }
    return true;
}
