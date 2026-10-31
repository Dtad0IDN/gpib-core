#ifndef MOCK_PORT_H
#define MOCK_PORT_H

#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib_port.h"

/* Scripted mock port for S1 mailbox tests: FIFO of injected events, a
 * byte-capped transfer model and a controllable quiesce latch. */

#define MOCK_MAX_EVENTS 16

typedef struct mock_port {
    gpib_port_t base;
    gpib_port_event_t events[MOCK_MAX_EVENTS];
    size_t ev_head;
    size_t ev_tail;
    unsigned events_dropped; /* T12: overflow counter, must stay 0 */
    bool arm_fail; /* injectable bounded-arm failure for LIF-02 */
    size_t bytes_per_transfer;
    size_t tx_accepted;
    const uint8_t *rx_src;
    size_t rx_len;
    size_t rx_pos;
    /* RX generator mode: deterministic xorshift stream (ignores rx_src). */
    bool rx_generate;
    uint32_t rx_gen_state;
    uint64_t rx_generated;
    /* TX integrity accumulator: sum of every accepted TX byte. */
    uint64_t tx_sum;
    bool stuck_quiesce;
    bool inhibit_called;
    uint64_t now_us;
} mock_port_t;

void mock_init(mock_port_t *m);
void mock_push_event(mock_port_t *m, gpib_port_event_t ev);
void mock_advance_us(mock_port_t *m, uint64_t delta);

#endif /* MOCK_PORT_H */
