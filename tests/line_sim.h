#ifndef LINE_SIM_H
#define LINE_SIM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib_proto.h"
#include "vbus.h"

/* Deterministic line-level simulation: one SH peer and several AH peers
 * exchange bytes over the wired-OR vbus lines (NRFD/NDAC/DAV/ATN/EOI/DIO),
 * exactly the Annex B byte cycle. One scheduler round = 1 virtual us:
 * read snapshot -> step every participant -> apply outputs. */

typedef struct line_bus {
    vbus_t bus;
    uint64_t now_us;
} line_bus_t;

typedef struct sh_peer {
    int id;
    gpib_sh_t sh;
    const uint8_t *data;
    size_t len;
    size_t pos;          /* next byte to present */
    uint32_t t1_us;
    bool data_mode;      /* data mode: TACS; command mode: CACS + ATN */
    size_t sent;         /* bytes with DAC confirmed by all acceptors */
    bool role_holdoff;   /* test hook: force SH_LOSES_ROLE next round */
    bool dav_requested;  /* last FSM output applied to the wire */
} sh_peer_t;

typedef struct ah_peer {
    int id;
    gpib_ah_t ah;
    uint8_t *rx;
    size_t cap;
    size_t len;          /* captured bytes (at ACDS entry) */
    size_t accepted;     /* bytes completed (ACDS -> AWNS) */
    uint32_t t3_us;
    uint32_t ready_delay_us; /* consumer slowness between bytes */
    uint64_t next_rdy_us;
    bool listen;         /* lads/lacs input for the scenario */
} ah_peer_t;

void line_bus_init(line_bus_t *lb);
void sh_peer_init(sh_peer_t *p, int id, const uint8_t *data, size_t len,
    uint32_t t1_us, bool data_mode);
void ah_peer_init(ah_peer_t *p, int id, uint8_t *rx, size_t cap,
    uint32_t t3_us, uint32_t ready_delay_us, bool listen);
/* One scheduler round. Returns false when a fault is detected on the bus. */
bool line_bus_round(line_bus_t *lb, sh_peer_t *src, ah_peer_t *acc,
    size_t n_acc);

#endif /* LINE_SIM_H */
