/**
 * @file gpib_4882.h
 * @brief IEEE 488.2 Module E helper: common-command status model.
 *
 * Implements the ESR/ESE/SRE/STB/MSS status reporting, the 12
 * standard common commands (*CLS *ESE *ESR? *IDN? *OPC *RST *SRE
 * *STB? *TST? *WAI), and the Response Message output queue with
 * QYE semantics. Designed as an integration helper for the device
 * role; NOT a full 488.2 parser.
 */
#ifndef GPIB_4882_H
#define GPIB_4882_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IEEE 488.2 module E: status reporting model and common commands.
 * Pure logic, no core dependency: the integration layer feeds received
 * device-message bytes in, polls the computed STB/rsv, and supplies the
 * pending response buffer back to the bus. Numeric values follow
 * IEEE 488.2-1992; response queue depth is 1 ([P]).
 *
 * STB bit assignment (488.2 --11): B7-B5 device-defined (unused here),
 * ESB = B5? No: ESB is B5 = 0x20, MAV = B4 = 0x10; B6 (0x40) is the
 * serial-poll RQS/MSS bit --- NOT stored in the device STB here: the core
 * ORs RQS in during SPAS from the SR function state, and MSS drives rsv.
 * ESR bits: PON 0x80, URQ 0x40, CME 0x20, EXE 0x10, DDE 0x08, QYE 0x04,
 * RQC 0x02, OPC 0x01. (488.2 --10.) */

typedef struct gpib_4882 {
    char idn[72];
    uint8_t esr;
    uint8_t ese;
    uint8_t sre;
    char line_buf[96];
    size_t line_len;
    bool line_suppressed; /* overflow: the WHOLE line incl. tail drops */
    char resp[96];
    size_t resp_len;
    bool resp_overflow; /* QYE latched: the message was dropped */
    uint8_t stb;   /* without B6 */
    bool rsv;      /* MSS: OR of enabled status summary bits */
    bool stb_dirty;
} gpib_4882_t;

void gpib_4882_init(gpib_4882_t *m, const char *idn);

/* Feed received device-message bytes; lines split on '\n' (the universal
 * <PMT>; CR is white space, never a terminator -- 488.2 7.5). Returns
 * true when a complete line was executed. The integration layer should
 * push stb/rsv to the core after every call. */
bool gpib_4882_feed(gpib_4882_t *m, const char *bytes, size_t n);

/* ^END termination (488.2 7.5): call when EOI is observed after the
 * final DAB of a program message that carried no <NL> -- executes the
 * buffered line. Returns true when a message was executed. */
bool gpib_4882_feed_end(gpib_4882_t *m);

/* Pending query response (without the trailing '\n'); supply it to the
 * bus and call consumed() after the transfer completes. */
const char *gpib_4882_response(const gpib_4882_t *m, size_t *len);
void gpib_4882_response_consumed(gpib_4882_t *m);

static inline uint8_t gpib_4882_stb(const gpib_4882_t *m) {
    return m->stb;
}
static inline bool gpib_4882_rsv(const gpib_4882_t *m) {
    return m->rsv;
}
static inline bool gpib_4882_stb_dirty(gpib_4882_t *m) {
    bool d = m->stb_dirty;
    m->stb_dirty = false;
    return d;
}

#ifdef __cplusplus
}
#endif

#endif /* GPIB_4882_H */
