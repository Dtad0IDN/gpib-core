#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdio.h>
#include <stdlib.h>

extern unsigned g_checks;
extern unsigned g_failures;

#define CHECK(cond)                                                     \
    do {                                                                \
        ++g_checks;                                                     \
        if (!(cond)) {                                                  \
            ++g_failures;                                               \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,     \
                #cond);                                                 \
        }                                                               \
    } while (0)

/* T09: REQUIRE is the hard precondition gate -- a failed REQUIRE means
 * the scenario is VOID: every later CHECK would test garbage on top of
 * a broken setup. Abort the whole binary so the failure cannot hide
 * behind a green tail (a soft CHECK would let a wedged fixture "pass"
 * hundreds of meaningless assertions afterwards). */
#define REQUIRE(cond)                                                   \
    do {                                                                \
        ++g_checks;                                                     \
        if (!(cond)) {                                                  \
            ++g_failures;                                               \
            fprintf(stderr, "REQUIRE-FAIL %s:%d: %s -- aborting\n",    \
                __FILE__, __LINE__, #cond);                             \
            fprintf(stderr, "gpib-core ABORT: checks=%u failures=%u\n", \
                g_checks, g_failures);                                  \
            exit(3);                                                    \
        }                                                               \
    } while (0)

#endif /* TEST_UTIL_H */
