#include <stddef.h>

#include "gpib/gpib_port.h"

static bool null_arm(void *user) {
    (void)user;
    return true;
}

static bool null_detached(void *user) {
    (void)user;
    return true;
}

static bool null_quiesced(void *user) {
    (void)user;
    return true;
}

static void null_inhibit(void *user) {
    (void)user;
}

static uint64_t null_now(void *user) {
    (void)user;
    return 0;
}

static size_t null_transfer(void *user, gpib_dir_t dir, uint8_t *buf,
    size_t cap) {
    (void)user;
    (void)dir;
    (void)buf;
    (void)cap;
    return 0; /* no bytes move without hardware */
}

static bool null_poll(void *user, gpib_port_event_t *out) {
    (void)user;
    (void)out;
    return false;
}

/* Host null port: no hardware behind the contract, therefore provably
 * detached and quiesced at all times; time stands still and no bytes move;
 * not line-capable (read_lines == NULL: byte-pump executor path). */
const gpib_port_t gpib_null_port = {
    "null",
    null_arm,
    null_detached,
    null_quiesced,
    null_inhibit,
    null_now,
    null_transfer,
    null_poll,
    NULL,
    NULL,
    NULL,
    0
};
