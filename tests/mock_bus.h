#ifndef MOCK_BUS_H
#define MOCK_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gpib/gpib_port.h"
#include "vbus.h"

/* Shared bus for CORE-TO-CORE tests: several core instances (real
 * gpib_context_t, not simulators) sit on one wired-OR vbus. The harness
 * interleaves gpib_process() calls; the port adapter is passive — every
 * read_lines only samples the bus and advances virtual time. */

typedef struct shared_bus {
    vbus_t bus;
    uint64_t now_us;
} shared_bus_t;

typedef struct bus_port {
    gpib_port_t base;
    shared_bus_t *shared;
    int id;
} bus_port_t;

void shared_bus_init(shared_bus_t *b);
void bus_port_init(bus_port_t *p, shared_bus_t *b, int id);

#endif /* MOCK_BUS_H */
