#include <stdio.h>
#include <string.h>

#include "mock_port.h"

static mock_port_t *self(void *user) {
    return (mock_port_t *)user;
}

static bool m_arm(void *user) {
    return !self(user)->arm_fail;
}

static bool m_detached(void *user) {
    (void)user;
    return true;
}

static bool m_quiesced(void *user) {
    return !self(user)->stuck_quiesce;
}

static void m_inhibit(void *user) {
    self(user)->inhibit_called = true;
}

static uint64_t m_now(void *user) {
    return self(user)->now_us;
}

static uint8_t gen_byte(mock_port_t *m) {
    /* xorshift32: deterministic, identical on both sides of the test. */
    uint32_t x = m->rx_gen_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    m->rx_gen_state = x;
    return (uint8_t)(x & 0xFFu);
}

static size_t m_transfer(void *user, gpib_dir_t dir, uint8_t *buf,
    size_t cap) {
    mock_port_t *m = self(user);
    size_t n = m->bytes_per_transfer;
    if (n > cap) {
        n = cap;
    }
    if (dir == GPIB_DIR_TX) {
        for (size_t i = 0; i < n; i++) {
            m->tx_sum += buf[i];
            if (m->journal_len < sizeof(m->journal)) {
                m->journal[m->journal_len++] = buf[i]; /* R17-01 */
            }
        }
        m->tx_accepted += n;
        return n;
    }
    if (m->rx_generate) {
        for (size_t i = 0; i < n; i++) {
            buf[i] = gen_byte(m);
        }
        m->rx_generated += n;
        return n;
    }
    if (m->rx_src == NULL || m->rx_pos >= m->rx_len) {
        return 0;
    }
    if (n > m->rx_len - m->rx_pos) {
        n = m->rx_len - m->rx_pos;
    }
    memcpy(buf, &m->rx_src[m->rx_pos], n);
    m->rx_pos += n;
    return n;
}

static bool m_poll(void *user, gpib_port_event_t *out) {
    mock_port_t *m = self(user);
    if (m->ev_head == m->ev_tail) {
        return false;
    }
    *out = m->events[m->ev_head];
    m->ev_head = (m->ev_head + 1u) % MOCK_MAX_EVENTS;
    return true;
}

void mock_init(mock_port_t *m) {
    memset(m, 0, sizeof(*m));
    m->base.name = "mock";
    m->base.arm = m_arm;
    m->base.safe_detached = m_detached;
    m->base.executor_quiesced = m_quiesced;
    m->base.emergency_inhibit = m_inhibit;
    m->base.now_us = m_now;
    m->base.transfer = m_transfer;
    m->base.poll_event = m_poll;
    m->base.user = m;
    m->bytes_per_transfer = 3;
    m->rx_gen_state = 0x12345678u;
}

void mock_push_event(mock_port_t *m, gpib_port_event_t ev) {
    size_t next = (m->ev_tail + 1u) % MOCK_MAX_EVENTS;
    if (next == m->ev_head) {
        /* T12: a scripted harness overflowing its own queue is a TEST
         * BUG -- silent loss would fabricate a green run. Count it and
         * say it loudly on stderr. */
        m->events_dropped++;
        fprintf(stderr,
            "mock_port: event queue overflow, dropped type=%u tag=%u "
            "(total dropped=%u)\n",
            (unsigned)ev.type, (unsigned)ev.tag,
            (unsigned)m->events_dropped);
        return;
    }
    m->events[m->ev_tail] = ev;
    m->ev_tail = next;
}

void mock_advance_us(mock_port_t *m, uint64_t delta) {
    m->now_us += delta;
}
