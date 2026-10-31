#include <stdio.h>
#include <stdlib.h>

#include "test_util.h"
void suite_addr(void);
void suite_lifecycle(void);
void suite_ledger(void);
void suite_txn(void);
void suite_mbx(void);
void suite_emergency(void);
void suite_stream(void);
void suite_vbus(void);
void suite_trace(void);
void suite_deadline(void);
void suite_interleave(void);
void suite_litmus(void);
void suite_proto(void);
void suite_linesim(void);
void suite_lineexec(void);
void suite_tlc(void);
void suite_spoll(void);
void suite_devneg(void);
void suite_dualcore(void);
void suite_rldcdt(void);
void suite_m4882(void);
void suite_s2oracle(void);
void suite_optfeatures(void);
void suite_oracle(void);

typedef void (*suite_fn_t)(void);

static const suite_fn_t k_suites[] = {
    suite_addr,      suite_lifecycle, suite_ledger,   suite_txn,
    suite_mbx,       suite_emergency, suite_stream,   suite_vbus,
    suite_trace,     suite_deadline,  suite_interleave, suite_litmus,
    suite_proto,     suite_linesim,   suite_lineexec, suite_tlc,
    suite_spoll,     suite_devneg,    suite_dualcore, suite_rldcdt,
    suite_m4882,     suite_s2oracle,  suite_optfeatures, suite_oracle
};

static const char *k_names[] = {
    "addr", "lifecycle", "ledger", "txn", "mbx", "emergency", "stream",
    "vbus", "trace", "deadline", "interleave", "litmus", "proto",
    "linesim", "lineexec", "tlc", "spoll", "devneg", "dualcore", "rldcdt",
    "m4882", "s2oracle", "optfeatures", "oracle"
};

unsigned g_checks;
unsigned g_failures;

int main(int argc, char **argv) {
    /* Bisect mode: run only the first N suites (argv[1]); default all.
     * T08: the count is validated -- "0", negative and garbage values
     * used to produce a green run with ZERO checks executed. */
    int total = (int)(sizeof(k_suites) / sizeof(k_suites[0]));
    int limit = total;
    if (argc > 1) {
        char *end = NULL;
        long v = strtol(argv[1], &end, 10);
        if (end == argv[1] || *end != '\0' || v <= 0 || v > (long)total) {
            fprintf(stderr, "usage: %s [1..%d]\n", argv[0], total);
            return 2;
        }
        limit = (int)v;
    }
    for (int i = 0; i < limit; i++) {
        fprintf(stderr, "[start] %s\n", k_names[i]);
        fflush(stderr);
        k_suites[i]();
    }
    printf("gpib-core S1/S2: checks=%u failures=%u\n", g_checks, g_failures);
    return (g_failures == 0u && g_checks > 0u) ? 0 : 1;
}
