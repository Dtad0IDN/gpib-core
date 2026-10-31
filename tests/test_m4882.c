#include <string.h>

#include "gpib/gpib_4882.h"
#include "test_util.h"

/* Module E unit surface: status model + common commands (488.2 §10-11). */

static void m4882_status_model(void) {
    gpib_4882_t m;
    gpib_4882_init(&m, "KILO,GPIB-CORE,0,1.0");
    CHECK(m.stb == 0);
    CHECK(!m.rsv);
    /* *OPC sets ESR.OPC; without *ESE enable it does not raise ESB. */
    CHECK(gpib_4882_feed(&m, "*OPC\n", 5));
    CHECK(m.esr == 0x01u);
    CHECK(m.stb == 0);
    CHECK(!m.rsv);
    /* Enable OPC bit: ESB appears. */
    CHECK(gpib_4882_feed(&m, "*ESE 1\n", 7));
    CHECK(m.ese == 0x01u);
    CHECK(m.esr == 0x01u);
    CHECK(m.stb == 0x20u);
    /* SRE gate (488.2 §11): ESB alone does NOT request service. */
    CHECK(!m.rsv);
    CHECK(gpib_4882_feed(&m, "*SRE 32\n", 8)); /* enable ESB summary */
    CHECK(m.sre == 0x20u);
    CHECK(m.rsv); /* ESB AND SRE.5 -> MSS */
    CHECK(gpib_4882_stb_dirty(&m));
    CHECK(!gpib_4882_stb_dirty(&m)); /* one-shot */
    /* Dropping the enable drops the request even though ESB stays. */
    CHECK(gpib_4882_feed(&m, "*SRE 0\n", 7));
    CHECK(m.stb == 0x20u);
    CHECK(!m.rsv);
}

static void m4882_queries(void) {
    gpib_4882_t m;
    size_t len = 0;
    const char *r;
    gpib_4882_init(&m, "KILO,GPIB-CORE,0,1.0");
    /* *IDN? queues the identity; MAV rises. */
    CHECK(gpib_4882_feed(&m, "*IDN?\n", 6));
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 20);
    CHECK(memcmp(r, "KILO,GPIB-CORE,0,1.0", 20) == 0);
    CHECK((m.stb & 0x10u) != 0); /* MAV */
    /* Consuming the response clears MAV. */
    gpib_4882_response_consumed(&m);
    CHECK((m.stb & 0x10u) == 0);
    /* *ESR? reads and clears; ESB falls with it. */
    CHECK(gpib_4882_feed(&m, "*OPC\n*ESE 1\n", 12));
    CHECK(m.esr == 0x01u);
    CHECK(gpib_4882_feed(&m, "*ESR?\n", 6));
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 1 && r[0] == '1');
    CHECK(m.esr == 0);      /* read-and-clear */
    CHECK((m.stb & 0x20u) == 0); /* ESB fell */
    /* *ESE? and *SRE round-trip. */
    CHECK(gpib_4882_feed(&m, "*SRE 16\n*SRE?\n", 14));
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 2 && r[0] == '1' && r[1] == '6');
    CHECK(m.sre == 16u);
    /* *STB? reports with MSS in B6 (read-only view, does not clear).
     * The *OPC line reset the response message (one message per line,
     * depth-1 queue [P]) -> MAV down; ESB is up but SRE.5 is not
     * enabled -> MSS false: 0x20 | 0 = 32. */
    CHECK(gpib_4882_feed(&m, "*OPC\n", 5));
    CHECK(gpib_4882_feed(&m, "*STB?\n", 6));
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 2);
    CHECK(r[0] == '3' && r[1] == '2');
    CHECK(m.rsv); /* the *STB? response itself raises MAV∧SRE.4 -> MSS */
    /* *CLS clears everything including the response queue. */
    CHECK(gpib_4882_feed(&m, "*CLS\n", 5));
    CHECK(m.esr == 0);
    gpib_4882_response(&m, &len);
    CHECK(len == 0);
    CHECK(m.stb == 0 && !m.rsv);
    /* Unknown command -> CME. */
    CHECK(gpib_4882_feed(&m, "*ZZZ\n", 5));
    CHECK(m.esr == 0x20u);
    /* *WAI and *TST? behave. */
    CHECK(gpib_4882_feed(&m, "*WAI\n", 5));
    CHECK(gpib_4882_feed(&m, "*TST?\n", 6));
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 1 && r[0] == '0');
    /* Compound message units (488.2 §7): ';'-separated, spaces allowed;
     * compound queries produce ';'-separated response units; SRE.6 is
     * never storable (reads back as 0). */
    CHECK(gpib_4882_feed(&m, "*CLS\n", 5));
    CHECK(gpib_4882_feed(&m, "*ESE 1; *SRE 32;*OPC\n", sizeof("*ESE 1; *SRE 32;*OPC\n") - 1u));
    CHECK(m.ese == 0x01u); /* leading space after ; accepted */
    CHECK(m.sre == 0x20u);
    CHECK(m.esr == 0x01u);
    CHECK(gpib_4882_feed(&m, "*SRE 64\n", sizeof("*SRE 64\n") - 1u)); /* bit 6 is not storable */
    CHECK(m.sre == 0x00u);
    gpib_4882_response_consumed(&m);
    CHECK(gpib_4882_feed(&m, "*ESE 32;*SRE 32;*OPC\n", sizeof("*ESE 32;*SRE 32;*OPC\n") - 1u)); /* ESB|CME off */
    CHECK(m.esr == 0x01u);
    CHECK(m.ese == 0x20u); /* ESE last-write-wins within the message */
    CHECK(gpib_4882_feed(&m, "*ESE 1\n", sizeof("*ESE 1\n") - 1u));
    gpib_4882_response_consumed(&m);
    CHECK(gpib_4882_feed(&m, "*ESE?;*SRE?\n", sizeof("*ESE?;*SRE?\n") - 1u)); /* compound query */
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 4); /* "1;32": ESE=1, SRE=0x20=32 */
    CHECK(r[0] == '1' && r[1] == ';' && r[2] == '3' && r[3] == '2');
    /* *OPC? answers '1' when no operation is pending (§12.5.3). */
    CHECK(gpib_4882_feed(&m, "*OPC?\n", sizeof("*OPC?\n") - 1u));
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 1 && r[0] == '1');
    /* Response-area overflow -> Query Error (488.2 6.3.2): the message
     * is dropped, QYE set, no partial units appended, the flag clears
     * with the next message. */
    {
        char big[160];
        int w = 0;
        for (int i = 0; i < 8; i++) {
            w += sprintf(&big[w], "*IDN?;");
        }
        w += sprintf(&big[w], "*OPC?\n");
        CHECK(gpib_4882_feed(&m, big, (size_t)w));
        CHECK((m.esr & 0x04u) != 0u); /* QYE fired on overflow */
        CHECK((m.esr & ~0x04u) == 0x01u); /* prior OPC only, no new bits */
        /* *CLS clears QYE; the *OPC? QUERY does not set the OPC bit
         * (only the *OPC command does, 488.2 10.18). */
        CHECK(gpib_4882_feed(&m, "*CLS\n*OPC?\n",
            sizeof("*CLS\n*OPC?\n") - 1u));
        CHECK(m.esr == 0x00u);
        r = gpib_4882_response(&m, &len);
        CHECK(r != NULL && len == 1 && r[0] == '1');
    }
}

/* Audit F22-F24 regressions: *RST semantics, PMT variants (NL / ^END /
 * CR-as-whitespace), query-interrupted QYE, dropped (never partial)
 * overflow responses. */
static void m4882_audit_regressions(void) {
    gpib_4882_t m;
    size_t len = 0;
    const char *r;
    gpib_4882_init(&m, "KILO,GPIB-CORE,0,1.0");

    /* F22: *RST is NOT *CLS -- ESR and a pending response survive.
     * (Consume the response first: any NEW program message -- *RST
     * included -- destroys an unread response with QYE, see F24.) */
    CHECK(gpib_4882_feed(&m, "*OPC\n", 5));
    CHECK(m.esr == 0x01u);
    CHECK(gpib_4882_feed(&m, "*IDN?\n", 6));
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 20);
    gpib_4882_response_consumed(&m);
    CHECK(gpib_4882_feed(&m, "*RST\n", 5));
    CHECK(m.esr == 0x01u); /* untouched (488.2 10.14) */
    r = gpib_4882_response(&m, &len);
    CHECK(r == NULL && len == 0); /* nothing was pending to survive */

    /* F23: CR is white space, never a terminator -- "*OPC\r\n" and
     * "*OPC\r \n" execute exactly once on the NL. */
    CHECK(gpib_4882_feed(&m, "*CLS\r\n", 6));
    CHECK(m.esr == 0u);
    CHECK(gpib_4882_feed(&m, "*OPC\r\n", 6));
    CHECK(m.esr == 0x01u);
    /* F23: ^END after the last DAB with no NL is a legal PMT. */
    CHECK(!gpib_4882_feed(&m, "*OPC", 4)); /* buffered, not executed */
    CHECK(m.esr == 0x01u);                  /* nothing new yet */
    CHECK(gpib_4882_feed_end(&m));
    CHECK(m.esr == 0x01u); /* *OPC again: bit already set */
    /* An empty ^END is a no-op. */
    CHECK(!gpib_4882_feed_end(&m));

    /* F24: unread response destroyed by the next program message -> QYE. */
    CHECK(gpib_4882_feed(&m, "*CLS\n", 5));
    CHECK(m.esr == 0u);
    CHECK(gpib_4882_feed(&m, "*IDN?\n", 6)); /* response queued, unread */
    CHECK(m.esr == 0u);
    CHECK(gpib_4882_feed(&m, "*IDN?\n", 6)); /* interrupts the previous */
    CHECK((m.esr & 0x04u) != 0u);             /* QYE latched */
    r = gpib_4882_response(&m, &len);
    CHECK(r != NULL && len == 20); /* the NEW response is intact */

    /* F24: an overflowed response is dropped COMPLETELY -- a truncated
     * prefix must not be readable back. */
    CHECK(gpib_4882_feed(&m, "*CLS\n", 5));
    {
        char big[160];
        int w = 0;
        for (int i = 0; i < 8; i++) {
            w += sprintf(&big[w], "*IDN?;");
        }
        w += sprintf(&big[w], "\n");
        CHECK(gpib_4882_feed(&m, big, (size_t)w));
        CHECK((m.esr & 0x04u) != 0u); /* QYE */
        r = gpib_4882_response(&m, &len);
        CHECK(r == NULL && len == 0); /* dropped, not partial */
    }

    /* F25: FULL-token matching -- near-miss headers are CME, not
     * aliases. "*ESEEVIL" shares the "*ESE" prefix; "*IDN?X" adds
     * garbage after the '?'; "*CLS?" queries a non-query. */
    CHECK(gpib_4882_feed(&m, "*CLS\n", 5));
    CHECK(m.esr == 0u);
    CHECK(gpib_4882_feed(&m, "*ESEEVIL\n", sizeof("*ESEEVIL\n") - 1u));
    CHECK((m.esr & 0x20u) != 0u); /* CME, NOT an *ESE execution */
    CHECK(m.ese == 0u);           /* no side effect */
    CHECK(gpib_4882_feed(&m, "*IDN?X\n", sizeof("*IDN?X\n") - 1u));
    CHECK((m.esr & 0x20u) != 0u);
    r = gpib_4882_response(&m, &len); /* no response queued */
    CHECK(r == NULL && len == 0);
    CHECK(gpib_4882_feed(&m, "*CLS?\n", sizeof("*CLS?\n") - 1u));
    CHECK((m.esr & 0x20u) != 0u); /* *CLS has no query form */
}

void suite_m4882(void) {
    m4882_status_model();
    m4882_queries();
    m4882_audit_regressions();
}
