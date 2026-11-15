#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
void suite_stress(void);

typedef void (*suite_fn_t)(void);

static const suite_fn_t k_suites[] = {
    suite_addr,      suite_lifecycle, suite_ledger,   suite_txn,
    suite_mbx,       suite_emergency, suite_stream,   suite_vbus,
    suite_trace,     suite_deadline,  suite_interleave, suite_litmus,
    suite_proto,     suite_linesim,   suite_lineexec, suite_tlc,
    suite_spoll,     suite_devneg,    suite_dualcore, suite_rldcdt,
    suite_m4882,     suite_s2oracle,  suite_optfeatures, suite_oracle,
    suite_stress
};

static const char *k_names[] = {
    "addr", "lifecycle", "ledger", "txn", "mbx", "emergency", "stream",
    "vbus", "trace", "deadline", "interleave", "litmus", "proto",
    "linesim", "lineexec", "tlc", "spoll", "devneg", "dualcore", "rldcdt",
    "m4882", "s2oracle", "optfeatures", "oracle", "stress"
};

unsigned g_checks;
unsigned g_failures;

int main(int argc, char **argv) {
    /* Two modes (T08/Rev10 §5): `--suite NAME` runs EXACTLY one suite
     * (the isolated CTest targets depend on this -- a numeric argument
     * would run the first N suites and reintroduce masking); a bare
     * number N runs the first N suites for bisection. The count is
     * validated -- "0", negative and garbage used to produce a green
     * run with ZERO checks executed. */
    int total = (int)(sizeof(k_suites) / sizeof(k_suites[0]));
    int limit = total;
    int only = -1;
    if (argc > 1 && strcmp(argv[1], "--suite") == 0) {
        if (argc <= 2) {
            fprintf(stderr, "usage: %s --suite NAME\n", argv[0]);
            return 2;
        }
        for (int i = 0; i < total; i++) {
            if (strcmp(argv[2], k_names[i]) == 0) {
                only = i;
                break;
            }
        }
        if (only < 0) {
            fprintf(stderr, "unknown suite '%s'\n", argv[2]);
            return 2;
        }
    } else if (argc > 1) {
        char *end = NULL;
        long v = strtol(argv[1], &end, 10);
        if (end == argv[1] || *end != '\0' || v <= 0 || v > (long)total) {
            fprintf(stderr, "usage: %s [1..%d] | --suite NAME\n",
                argv[0], total);
            return 2;
        }
        limit = (int)v;
    }
    if (only >= 0) {
        fprintf(stderr, "[start] %s\n", k_names[only]);
        fflush(stderr);
        k_suites[only]();
    } else {
        for (int i = 0; i < limit; i++) {
            fprintf(stderr, "[start] %s\n", k_names[i]);
            fflush(stderr);
            k_suites[i]();
        }
    }
    printf("gpib-core S1/S2: checks=%u failures=%u\n", g_checks, g_failures);
    return (g_failures == 0u && g_checks > 0u) ? 0 : 1;
}
