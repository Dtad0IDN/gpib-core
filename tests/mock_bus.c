#include <string.h>

#include "mock_bus.h"

static bool b_arm(void *user) {
    (void)user;
    return true;
}

static bool b_detached(void *user) {
    bus_port_t *p = (bus_port_t *)user;
    /* T8-01: honest evidence on the shared wire. */
    return vbus_participant_clean(&p->shared->bus, p->id);
}

static bool b_quiesced(void *user) {
    (void)user;
    return true;
}

static void b_inhibit(void *user) {
    (void)user;
}

static uint64_t b_now(void *user) {
    return ((bus_port_t *)user)->shared->now_us;
}

static size_t b_transfer(void *user, gpib_dir_t dir, uint8_t *buf,
    size_t cap) {
    (void)user;
    (void)dir;
    (void)buf;
    (void)cap;
    return 0;
}

static bool b_poll(void *user, gpib_port_event_t *out) {
    (void)user;
    (void)out;
    return false;
}

static bool b_read_lines(void *user, gpib_line_snap_t *out) {
    bus_port_t *p = (bus_port_t *)user;
    shared_bus_t *b = p->shared;
    memset(out, 0, sizeof(*out));
    out->dav = vbus_level(&b->bus, VBL_DAV);
    out->nrfd = vbus_level(&b->bus, VBL_NRFD);
    out->ndac = vbus_level(&b->bus, VBL_NDAC);
    out->atn = vbus_level(&b->bus, VBL_ATN);
    out->eoi = vbus_level(&b->bus, VBL_EOI);
    out->srq = vbus_level(&b->bus, VBL_SRQ);
    out->ifc = vbus_level(&b->bus, VBL_IFC);
    out->ren = vbus_level(&b->bus, VBL_REN);
    uint8_t dio = 0;
    if (vbus_dio_level(&b->bus, &dio) == 0) {
        out->dio = dio;
    }
    vbus_tick(&b->bus);
    b->now_us++;
    out->now_us = b->now_us;
    return true;
}

static void b_drive(void *user, const gpib_line_drive_t *req) {
    bus_port_t *p = (bus_port_t *)user;
    vbus_t *bus = &p->shared->bus;
    if (req->dav) {
        vbus_assert(bus, p->id, VBL_DAV);
    } else {
        vbus_release(bus, p->id, VBL_DAV);
    }
    if (req->nrfd) {
        vbus_assert(bus, p->id, VBL_NRFD);
    } else {
        vbus_release(bus, p->id, VBL_NRFD);
    }
    if (req->ndac) {
        vbus_assert(bus, p->id, VBL_NDAC);
    } else {
        vbus_release(bus, p->id, VBL_NDAC);
    }
    if (req->atn) {
        vbus_assert(bus, p->id, VBL_ATN);
    } else {
        vbus_release(bus, p->id, VBL_ATN);
    }
    if (req->ifc) {
        vbus_assert(bus, p->id, VBL_IFC);
    } else {
        vbus_release(bus, p->id, VBL_IFC);
    }
    if (req->ren) {
        vbus_assert(bus, p->id, VBL_REN);
    } else {
        vbus_release(bus, p->id, VBL_REN);
    }
    if (req->srq) {
        vbus_assert(bus, p->id, VBL_SRQ);
    } else {
        vbus_release(bus, p->id, VBL_SRQ);
    }
    if (req->eoi) {
        vbus_assert(bus, p->id, VBL_EOI);
    } else {
        vbus_release(bus, p->id, VBL_EOI);
    }
}

static void b_drive_data(void *user, const uint8_t *dio) {
    bus_port_t *p = (bus_port_t *)user;
    if (dio != NULL) {
        vbus_drive_dio(&p->shared->bus, p->id, *dio);
    } else {
        vbus_release_dio(&p->shared->bus, p->id);
    }
}

void shared_bus_init(shared_bus_t *b) {
    vbus_init(&b->bus);
    b->now_us = 0;
}

void bus_port_init(bus_port_t *p, shared_bus_t *b, int id) {
    memset(p, 0, sizeof(*p));
    p->base.name = "shared-bus";
    p->base.arm = b_arm;
    p->base.safe_detached = b_detached;
    p->base.executor_quiesced = b_quiesced;
    p->base.emergency_inhibit = b_inhibit;
    p->base.now_us = b_now;
    p->base.transfer = b_transfer;
    p->base.poll_event = b_poll;
    p->base.read_lines = b_read_lines;
    p->base.drive = b_drive;
    p->base.drive_data = b_drive_data;
    p->base.user = p;
    p->shared = b;
    p->id = id;
    vbus_attach_as(&b->bus, id);
}
