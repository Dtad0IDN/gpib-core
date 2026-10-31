#ifndef MOCK_CTRL_H
#define MOCK_CTRL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib_port.h"
#include "gpib/gpib_proto.h"
#include "vbus.h"

/* Mock CONTROLLER for device-role tests: a scripted external controller
 * that drives the wired-OR bus (participant 1; the device core is 0).
 * Every read_lines call advances it by one round, symmetric to the
 * instrument mock. */

#define MCTRL_RX_MAX 256

typedef enum mc_state {
    MCS_IDLE = 0,
    MCS_CMD,
    MCS_TX,
    MCS_RX,
    MCS_DONE
} mc_state_t;

typedef struct mock_ctrl {
    vbus_t bus;
    uint64_t now_us;
    gpib_sh_t sh;
    gpib_ah_t ah;
    gpib_addr_engine_t addr;
    mc_state_t st;
    uint8_t cmds[8];
    size_t n_cmds;
    size_t cmd_pos;
    const uint8_t *tx;
    size_t tx_len;
    size_t tx_pos;
    uint8_t rx[MCTRL_RX_MAX];
    size_t rx_len;
    size_t rx_expect;
    bool done;
    bool dav;
    uint8_t tx_byte;
} mock_ctrl_t;

typedef struct mock_ctrl_port {
    gpib_port_t base;
    mock_ctrl_t c;
} mock_ctrl_port_t;

void mock_ctrl_init(mock_ctrl_port_t *p, uint8_t ctrl_addr);
/* Controller writes `len` bytes to the device (UNL, MLA(dev), MTA(ctrl)). */
void mock_ctrl_write(mock_ctrl_port_t *p, uint8_t dev_addr,
    const uint8_t *data, size_t len);
/* Controller reads up to `expect` bytes from the device (EOI ends). */
void mock_ctrl_read(mock_ctrl_port_t *p, uint8_t dev_addr, size_t expect);
/* Controller sends raw ATN command bytes (already-addressed scenarios). */
void mock_ctrl_cmds(mock_ctrl_port_t *p, const uint8_t *bytes, size_t n);

#endif /* MOCK_CTRL_H */
