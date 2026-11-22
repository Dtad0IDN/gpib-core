/**
 * @file mod4882.c
 * @brief IEEE 488.2 Module E helper: common-command execution, status model, response queue.
 */
#include <string.h>

#include "gpib/gpib_4882.h"

/* ESR bits (488.2 --10). */
#define ESR_PON 0x80u
#define ESR_URQ 0x40u
#define ESR_CME 0x20u
#define ESR_EXE 0x10u
#define ESR_DDE 0x08u
#define ESR_QYE 0x04u
#define ESR_RQC 0x02u
#define ESR_OPC 0x01u

/* STB bits: ESB 0x20, MAV 0x10 (B6/RQS handled by the core SPAS path). */
#define STB_ESB 0x20u
#define STB_MAV 0x10u

static void stb_recompute(gpib_4882_t *m) {
    /* 488.2 --11: ESB summarizes ESR&ESE (STB bit 5), MAV summarizes the
     * output queue (bit 4); MSS (and therefore rsv) is the OR of the
     * summary bits ANDed with their Service Request Enable bits (SRE.5
     * for ESB, SRE.4 for MAV). Without *SRE the device never requests
     * service, however loud its status registers are. */
    bool esb = (m->esr & m->ese & 0xFFu) != 0u;
    bool mav = m->resp_len > 0u;
    bool mss = (esb && (m->sre & 0x20u) != 0u) ||
               (mav && (m->sre & 0x10u) != 0u);
    m->stb = (uint8_t)((esb ? STB_ESB : 0u) | (mav ? STB_MAV : 0u));
    m->rsv = mss;
    m->stb_dirty = true;
}

/* Response-message assembly per 488.2 --8: multiple queries inside one
 * <PROGRAM MESSAGE> produce the corresponding <RESPONSE MESSAGE UNITS>
 * separated by ';'. resp_begin() resets the message ONCE per program
 * line (exec_line), each query unit APPENDS with a ';' separator. */
static void resp_begin(gpib_4882_t *m) {
    if (m->resp_len > 0u) {
        /* F24 (488.2 6.3): a previous query response was never read --
         * starting a new program message destroys it: Query Error. */
        m->esr |= ESR_QYE;
    }
    m->resp_len = 0;
    /* A14: MAV must reflect the queue AFTER the clear -- the old code
     * recomputed before wiping resp_len, leaving MAV/RSV asserted about
     * a response that no longer exists. */
    stb_recompute(m);
}

static void resp_sep(gpib_4882_t *m) {
    if (m->resp_len > 0u && m->resp_len + 1u < sizeof(m->resp)) {
        m->resp[m->resp_len++] = ';';
    }
}

/* 488.2 6.3.2 / 8.7.1: a query whose <RESPONSE DATA> does not fit the
 * output queue area is a Query Error: set QYE, drop the message, do not
 * append a partial unit. */
static void resp_unit_begin(gpib_4882_t *m) {
    if (m->resp_len + 2u >= sizeof(m->resp)) { /* room for >=1 char */
        m->esr |= ESR_QYE;
        m->resp_overflow = true;
        stb_recompute(m);
        return;
    }
    if (m->resp_overflow) {
        return; /* message already dropped by an earlier unit */
    }
    resp_sep(m);
}

static void resp_str(gpib_4882_t *m, const char *s) {
    if (m->resp_overflow) {
        return;
    }
    while (*s != '\0' && m->resp_len + 1u < sizeof(m->resp)) {
        m->resp[m->resp_len++] = *s++;
    }
    if (*s != '\0') { /* truncated by capacity: Query Error */
        /* F24: the message is DROPPED -- a truncated prefix must not be
         * readable back (488.2 6.3.2 "drop the message"). */
        m->esr |= ESR_QYE;
        m->resp_len = 0;
        m->resp_overflow = true;
        stb_recompute(m);
    }
}

static void resp_u8(gpib_4882_t *m, uint8_t v) {
    if (m->resp_overflow) {
        return;
    }
    char tmp[4];
    size_t n = 0;
    if (v == 0u) {
        tmp[n++] = '0';
    }
    while (v > 0u && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10u));
        v = (uint8_t)(v / 10u);
    }
    while (n > 0u) {
        if (m->resp_len + 1u >= sizeof(m->resp)) { /* no room: QYE */
            /* F24: drop the message -- no truncated prefix. */
            m->esr |= ESR_QYE;
            m->resp_len = 0;
            m->resp_overflow = true;
            stb_recompute(m);
            return;
        }
        m->resp[m->resp_len++] = tmp[--n];
    }
}

static bool parse_u8(const char *s, size_t len, uint8_t *out) {
    if (len == 0u) {
        return false;
    }
    unsigned v = 0;
    for (size_t i = 0; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
        v = v * 10u + (unsigned)(s[i] - '0');
        if (v > 255u) {
            return false;
        }
    }
    *out = (uint8_t)v;
    return true;
}

/* Execute one complete program message line (without the <PMT>): per
 * 488.2 --7 a <PROGRAM MESSAGE> is a ';'-separated list of <PROGRAM
 * MESSAGE UNIT>s, so "*ESE 1;*OPC<NL>" is TWO units. The caller splits
 * on NL; we split each line on ';' and execute every unit in order. */
static void exec_unit(gpib_4882_t *m, const char *unit, size_t len);

/* 488.2 7.6: common-command mnemonics are case-INsensitive
 * ("*idn?" == "*IDN?"). Four ASCII letters, upper-cased pairwise. */
static bool ci4(const char *a, const char *b) {
    for (int i = 0; i < 4; i++) {
        char x = a[i];
        char y = b[i];
        if (x >= 'a' && x <= 'z') {
            x = (char)(x - 'a' + 'A');
        }
        if (y >= 'a' && y <= 'z') {
            y = (char)(y - 'a' + 'A');
        }
        if (x != y) {
            return false;
        }
    }
    return true;
}

static void exec_line(gpib_4882_t *m, const char *line, size_t len) {
    resp_begin(m); /* one response message per program message line */
    m->resp_overflow = false; /* QYE is per-message (488.2 6.3.2) */
    size_t start = 0;
    while (start <= len) {
        size_t end = start;
        while (end < len && line[end] != ';') {
            end++;
        }
        exec_unit(m, &line[start], end - start);
        if (end >= len) {
            break;
        }
        start = end + 1u; /* skip the ';' separator */
    }
}

static void exec_unit(gpib_4882_t *m, const char *unit, size_t len) {
    /* Skip leading whitespace, separate mnemonic and argument. */
    size_t i = 0;
    while (i < len && (unit[i] == ' ' || unit[i] == '\t')) {
        i++;
    }
    size_t start = i;
    while (i < len && unit[i] != ' ' && unit[i] != '\t') {
        i++;
    }
    size_t mlen = i - start;
    size_t alen = 0;
    size_t astart = i;
    while (astart < len && (unit[astart] == ' ' || unit[astart] == '\t')) {
        astart++;
    }
    alen = len - astart;

    uint8_t arg = 0;
    bool is_query = (mlen == 5u && unit[start + 4u] == '?');
    if (alen > 0u && !parse_u8(&unit[astart], alen, &arg)) {
        /* 488.2: syntax error -> CME; keep simple: ignore with CME. */
        m->esr |= ESR_CME;
        stb_recompute(m);
        return;
    }
    if (mlen < 4u) {
        m->esr |= ESR_CME;
        stb_recompute(m);
        return;
    }
    /* Match the FULL mnemonic, never a prefix (F25): "*ESEEVIL" or
     * "*ES" must NOT alias "*ESE". Every header is exactly 4 chars; the
     * query form adds '?' as the 5th (and last) character. */
    const char *mn = &unit[start];
    bool form_ok = (mlen == 4u) || (is_query && mlen == 5u);
    if (!form_ok) {
        m->esr |= ESR_CME;
        stb_recompute(m);
        return;
    }
    if (ci4(mn, "*CLS") && !is_query && alen == 0u) {
        m->esr = 0;
        m->resp_len = 0;
        stb_recompute(m);
        return;
    }
    if (ci4(mn, "*ESE")) {
        /* AUD-19: an argument is MANDATORY for the program form: a bare
         * "*ESE" used to assign zero and silently clear the mask. */
        if (is_query) {
            if (alen != 0u) {
                m->esr |= ESR_CME; /* query takes no argument */
                stb_recompute(m);
                return;
            }
            resp_unit_begin(m);
            resp_u8(m, m->ese);
            stb_recompute(m);
        } else {
            if (alen == 0u) {
                m->esr |= ESR_CME;
                stb_recompute(m);
                return;
            }
            m->ese = arg;
            stb_recompute(m); /* enabling bits can raise MSS */
        }
        return;
    }
    if (ci4(mn, "*ESR") && is_query) {
        if (alen != 0u) {
            m->esr |= ESR_CME; /* D06 */
            stb_recompute(m);
            return;
        }
        resp_unit_begin(m);
        resp_u8(m, m->esr);
        m->esr = 0; /* read-and-clear */
        stb_recompute(m);
        return;
    }
    if (ci4(mn, "*IDN") && is_query) {
        if (alen != 0u) {
            m->esr |= ESR_CME; /* D06 */
            stb_recompute(m);
            return;
        }
        resp_unit_begin(m);
        resp_str(m, m->idn);
        stb_recompute(m);
        return;
    }
    if (ci4(mn, "*OPC")) {
        if (is_query) {
            if (alen != 0u) {
                m->esr |= ESR_CME; /* D06 */
                stb_recompute(m);
                return;
            }
            /* *OPC?: with no pending operations the answer is ready ---
             * place '1' in the output queue immediately (--12.5.3). */
            resp_unit_begin(m);
            resp_str(m, "1");
            stb_recompute(m);
        } else {
            m->esr |= ESR_OPC;
            stb_recompute(m);
        }
        return;
    }
    if (ci4(mn, "*SRE")) {
        if (is_query) {
            if (alen != 0u) {
                m->esr |= ESR_CME; /* D06 */
                stb_recompute(m);
                return;
            }
            resp_unit_begin(m);
            resp_u8(m, m->sre);
            stb_recompute(m);
        } else {
            if (alen == 0u) {
                m->esr |= ESR_CME;
                stb_recompute(m);
                return;
            }
            m->sre = (uint8_t)(arg & 0xBFu); /* SRE.6 is unused: 0 */
            stb_recompute(m); /* enabling/disabling bits changes MSS */
        }
        return;
    }
    if (ci4(mn, "*STB") && is_query) {
        if (alen != 0u) {
            m->esr |= ESR_CME; /* D06 */
            stb_recompute(m);
            return;
        }
        resp_unit_begin(m);
        resp_u8(m, (uint8_t)(m->stb | (m->rsv ? 0x40u : 0x00u)));
        stb_recompute(m);
        return;
    }
    if (ci4(mn, "*TST") && is_query) {
        if (alen != 0u) {
            m->esr |= ESR_CME; /* D06 */
            stb_recompute(m);
            return;
        }
        resp_unit_begin(m);
        resp_str(m, "0");
        stb_recompute(m);
        return;
    }
    if (ci4(mn, "*WAI") && !is_query) {
        return; /* single-threaded execution model: nothing to wait for */
    }
    if (ci4(mn, "*RST") && !is_query) {
        /* F22 (488.2 10.14): *RST returns the DEVICE-DEPENDENT functions
         * to a known state -- it is NOT *CLS: the event register and the
         * pending response message are untouched. This helper has no
         * device functions; the integrator hooks its own reset here. */
        return;
    }
    /* Unknown command or wrong form: command error. */
    m->esr |= ESR_CME;
    stb_recompute(m);
}

bool gpib_4882_feed(gpib_4882_t *m, const char *bytes, size_t n) {
    bool executed = false;
    for (size_t k = 0; k < n; k++) {
        if (bytes[k] == '\n') {
            /* F23 (488.2 7.5): <NL> is THE universal <PMT>. */
            if (m->line_len > 0u && !m->line_suppressed) {
                exec_line(m, m->line_buf, m->line_len);
                m->line_len = 0;
                executed = true;
            }
            m->line_len = 0;
            m->line_suppressed = false; /* a new line starts clean */
            continue;
        }
        if (bytes[k] == '\r') {
            /* F23: CR is white space INSIDE a message (7.5) -- like a
             * blank it never becomes part of a token. */
            continue;
        }
        if (m->line_suppressed) {
            continue; /* the tail of an overflowed line is DISCARDED,
                       * never executed as a fresh command (final audit:
                       * "100x'A';*ESE 32;*OPC" used to run its tail) */
        }
        if (m->line_len + 1u < sizeof(m->line_buf)) {
            m->line_buf[m->line_len++] = bytes[k];
        } else {
            m->line_len = 0;
            m->line_suppressed = true; /* drop the WHOLE line incl. tail */
            m->esr |= ESR_CME;
            stb_recompute(m);
        }
    }
    return executed;
}

bool gpib_4882_feed_end(gpib_4882_t *m) {
    /* F23 (488.2 7.5): ^END received after the last <DAB> with no <NL>
     * -- a legal <PMT> variant ("NL^END" is handled by feed itself).
     * The integration layer calls this when EOI is observed on the
     * final byte of the program message. AUD-18: END terminates the
     * message exactly like NL -- a suppressed (overflowed) line is
     * dropped and the suppression CLEARED, so the NEXT EOI-only
     * message is parsed (a device answering NL-less clients used to
     * go permanently silent after one overflow). */
    if (m->line_len > 0u && !m->line_suppressed) {
        exec_line(m, m->line_buf, m->line_len);
        m->line_len = 0;
        m->line_suppressed = false;
        return true;
    }
    m->line_len = 0;
    m->line_suppressed = false;
    return false;
}

void gpib_4882_init(gpib_4882_t *m, const char *idn) {
    memset(m, 0, sizeof(*m));
    size_t i = 0;
    while (idn != NULL && idn[i] != '\0' && i + 1u < sizeof(m->idn)) {
        m->idn[i] = idn[i];
        i++;
    }
    m->idn[i] = '\0';
    /* 488.2 12.2: Power-On sets the PON event bit -- the status model
     * starts with a reportable event (final audit: PON was missing). */
    m->esr |= ESR_PON;
    stb_recompute(m);
}

const char *gpib_4882_response(const gpib_4882_t *m, size_t *len) {
    *len = m->resp_len;
    return m->resp_len > 0u ? m->resp : NULL;
}

void gpib_4882_response_consumed(gpib_4882_t *m) {
    m->resp_len = 0;
    stb_recompute(m);
}
