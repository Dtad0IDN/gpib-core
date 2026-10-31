#include <string.h>

#include "vbus.h"

void vbus_init(vbus_t *b) {
    memset(b, 0, sizeof(*b));
}

int vbus_attach(vbus_t *b) {
    for (int i = 0; i < VBUS_MAX_PART; i++) {
        if (!b->attached[i]) {
            b->attached[i] = true;
            b->assert_mask[i] = 0;
            b->dio_active[i] = false;
            b->dio_value[i] = 0;
            return i;
        }
    }
    return -1;
}

void vbus_attach_as(vbus_t *b, int id) {
    if (id < 0 || id >= VBUS_MAX_PART) {
        return;
    }
    if (b->attached[id]) {
        return; /* idempotent: must not wipe live drive state */
    }
    b->attached[id] = true;
    b->assert_mask[id] = 0;
    b->dio_active[id] = false;
    b->dio_value[id] = 0;
}

void vbus_detach(vbus_t *b, int id) {
    if (id < 0 || id >= VBUS_MAX_PART || !b->attached[id]) {
        return;
    }
    b->attached[id] = false;
    b->assert_mask[id] = 0;
    b->dio_active[id] = false;
}

void vbus_assert(vbus_t *b, int id, vbus_line_t line) {
    if (id < 0 || id >= VBUS_MAX_PART || !b->attached[id] ||
        (int)line < 0 || (int)line >= (int)VBL_COUNT) {
        return;
    }
    /* Handshake NRFD/NDAC and SRQ are designed for shared assertion; DAV
     * and ATN are single-driver lines: simultaneous assertion of DAV by
     * two participants is a protocol contention, reported not ignored. */
    if (line == VBL_DAV) {
        for (int i = 0; i < VBUS_MAX_PART; i++) {
            if (i != id && b->attached[i] &&
                (b->assert_mask[i] & (1u << (int)VBL_DAV)) != 0) {
                b->conflicts++;
            }
        }
    }
    b->assert_mask[id] |= (uint16_t)(1u << (int)line);
}

void vbus_release(vbus_t *b, int id, vbus_line_t line) {
    if (id < 0 || id >= VBUS_MAX_PART || !b->attached[id] ||
        (int)line < 0 || (int)line >= (int)VBL_COUNT) {
        return;
    }
    b->assert_mask[id] &= (uint16_t)~(1u << (int)line);
}

bool vbus_level(const vbus_t *b, vbus_line_t line) {
    /* Wired-OR: the line is asserted (electrically low) when at least one
     * attached participant asserts it; release never forces the level. */
    for (int i = 0; i < VBUS_MAX_PART; i++) {
        if (b->attached[i] && (b->assert_mask[i] & (1u << (int)line)) != 0) {
            return true;
        }
    }
    return false;
}

void vbus_drive_dio(vbus_t *b, int id, uint8_t value) {
    if (id < 0 || id >= VBUS_MAX_PART || !b->attached[id]) {
        return;
    }
    for (int i = 0; i < VBUS_MAX_PART; i++) {
        if (i == id || !b->attached[i]) {
            continue;
        }
        if (b->dio_active[i]) {
            /* T07: TWO talkers is a contention even when the bytes
             * happen to match -- identical data must not pass as legal. */
            b->conflicts++;
        }
        if (b->pp_active[i]) {
            b->conflicts++; /* domain crossing: talker vs parallel poll */
        }
    }
    b->dio_active[id] = true;
    b->dio_value[id] = value;
}

void vbus_release_dio(vbus_t *b, int id) {
    if (id < 0 || id >= VBUS_MAX_PART) {
        return;
    }
    b->dio_active[id] = false;
}

void vbus_pp_drive(vbus_t *b, int id, uint8_t value) {
    if (id < 0 || id >= VBUS_MAX_PART || !b->attached[id]) {
        return;
    }
    for (int i = 0; i < VBUS_MAX_PART; i++) {
        if (i != id && b->attached[i] && b->dio_active[i]) {
            b->conflicts++; /* domain crossing: parallel poll vs talker */
        }
    }
    b->pp_active[id] = true;
    b->pp_value[id] = value;
}

void vbus_pp_release(vbus_t *b, int id) {
    if (id < 0 || id >= VBUS_MAX_PART) {
        return;
    }
    b->pp_active[id] = false;
}

int vbus_dio_level(const vbus_t *b, uint8_t *out) {
    int talkers = 0;
    uint8_t tv = 0;
    uint8_t pp = 0;
    bool any_pp = false;
    for (int i = 0; i < VBUS_MAX_PART; i++) {
        if (!b->attached[i]) {
            continue;
        }
        if (b->dio_active[i]) {
            talkers++;
            tv = b->dio_value[i]; /* >1 talker -> error below */
        }
        if (b->pp_active[i]) {
            any_pp = true;
            pp = (uint8_t)(pp | b->pp_value[i]); /* wired-OR PPR bits */
        }
    }
    if (talkers > 1) {
        return -1; /* unresolved contention: explicit error */
    }
    if (talkers == 0 && !any_pp) {
        return -1; /* nobody drives: no defined value */
    }
    *out = (uint8_t)(tv | pp);
    return 0;
}

void vbus_tick(vbus_t *b) {
    b->ticks++;
}

bool vbus_participant_clean(const vbus_t *b, int id) {
    if (id < 0 || id >= VBUS_MAX_PART || !b->attached[id]) {
        return false; /* unknown participant: no evidence either way */
    }
    if (b->assert_mask[id] != 0 || b->dio_active[id] || b->pp_active[id]) {
        return false; /* something is still driven: NOT safely detached */
    }
    return true;
}
