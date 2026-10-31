#ifndef MOCK_LINES_H
#define MOCK_LINES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib_port.h"
#include "gpib/gpib_proto.h"
#include "vbus.h"

/* Mock line port: a vbus with one simulated instrument. The core is bus
 * participant 0, the instrument participant 1. Every read_lines call runs
 * one instrument exchange round, so the simulated device progresses in
 * parallel with the core, exactly like real hardware would. */

#define MINS_RX_LOG 512
#define MINS_CMD_LOG 32

typedef struct mock_instrument {
    /* bus side: pointer to the SHARED bus (one vbus per port) */
    vbus_t *bus;
    int id; /* bus participant id */
    uint64_t now_us;
    /* protocol functions of the instrument */
    gpib_sh_t sh;
    gpib_ah_t ah;
    gpib_addr_engine_t addr;
    gpib_talker_t talker; /* serial-poll-capable talker (T2-style) */
    gpib_sr_t sr;
    bool dav;
    /* observed command history (decoded) */
    gpib_ifcmd_t cmdlog[MINS_CMD_LOG];
    size_t n_cmdlog;
    /* data received while addressed as listener */
    uint8_t rx[MINS_RX_LOG];
    size_t rxlen;
    size_t accepted_total;
    /* scripted reply while addressed as talker */
    const uint8_t *reply;
    size_t reply_len;
    size_t txpos;
    bool reply_armed;
    /* serial poll: status byte and service request state */
    uint8_t stb;
    bool rsv;
    bool stb_pending;
    uint8_t stb_byte; /* the byte actually sent (with RQS) */
    /* Parallel Poll (PP1 draft): PP FSM, IST bit, configured response */
    gpib_pp_t pp;
    bool pp_ist;
    bool pp_driving_dio; /* PPAS: we are driving our DIO line */
    /* device function events observed (DT triggers, DC clears) */
    size_t dt_triggers;
    size_t dc_clears;
    /* unified talker source for this round */
    bool tx_active;
    uint8_t tx_byte;
    bool tx_with_eoi;
    uint8_t holdoff; /* consumer not-ready window counter */
} mock_instrument_t;

typedef struct mock_line_port {
    gpib_port_t base;
    /* T16: the bus is PER FIXTURE, not a hidden file-level singleton --
     * two mock_line_port_t instances in one binary must never share a
     * wire. ins.bus / ins2.bus point here. */
    vbus_t own_bus;
    mock_instrument_t ins;
    /* Optional second instrument on the SAME bus (ALLSPOLL tests). */
    bool has_second;
    mock_instrument_t ins2;
} mock_line_port_t;

void mock_lines_init(mock_line_port_t *p);
/* Add a second instrument to the same bus at `addr` (participant id 3). */
void mock_lines_add_second(mock_line_port_t *p, uint8_t addr);
/* Script the instrument's reply (sent with EOI on the last byte once the
 * controller addresses it as talker and releases ATN). */
void mock_lines_arm_reply(mock_line_port_t *p, const uint8_t *data,
    size_t len);
/* Arm the serial-poll answer: STB value and the service-request state. */
void mock_lines_set_stb(mock_line_port_t *p, uint8_t stb, bool rsv);
void mock_lines_set_stb_second(mock_line_port_t *p, uint8_t stb, bool rsv);
/* External controller stealing ATN (test hook, participant id 2). */
void mock_lines_steal_atn(mock_line_port_t *p, bool steal);

#endif /* MOCK_LINES_H */
