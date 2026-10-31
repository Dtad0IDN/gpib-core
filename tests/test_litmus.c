/* T02: portable thread primitive -- Win32 on Windows, pthread elsewhere
 * (the Linux/Clang CI path). The tested memory-order pattern is the
 * same; only the thread spawn/join is platform-specific. */
#include <stdbool.h>
#if defined(_WIN32)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#pragma GCC diagnostic pop

typedef HANDLE litmus_thread_t;
static DWORD WINAPI producer(LPVOID param);
static bool litmus_spawn(litmus_thread_t *th) {
    *th = CreateThread(NULL, 0, producer, NULL, 0, NULL);
    return *th != NULL;
}
static bool litmus_join(litmus_thread_t th) {
    bool ok = WaitForSingleObject(th, 5000) == WAIT_OBJECT_0;
    CloseHandle(th);
    return ok;
}
#else
#include <pthread.h>

typedef pthread_t litmus_thread_t;
static void *producer(void *param);
static bool litmus_spawn(litmus_thread_t *th) {
    return pthread_create(th, NULL, producer, NULL) == 0;
}
static bool litmus_join(litmus_thread_t th) {
    return pthread_join(th, NULL) == 0;
}
#endif

#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>

#include "test_util.h"

/* Memory-order litmus for the mailbox publication pattern (MAILBOX §2/§9).
 *
 * The tested pattern is the bounded two-slot SPSC exchange: the producer
 * writes an immutable descriptor slot non-atomically, then publishes its
 * sequence number with a release store; the consumer acquires the sequence,
 * reads the slot, and publishes its consumption so the producer never
 * overwrites a slot that is still in flight. This mirrors the core rule
 * "a descriptor is immutable after publish and a slot is reused only after
 * retirement/accounting".
 *
 * A naive single-slot overwrite publication (producer rewriting the
 * descriptor while the consumer reads it, with only a seq-number recheck)
 * is NOT sound: the recheck cannot distinguish "stable" from "writer busy
 * with the next version before publishing". That pattern is deliberately
 * not used by the core; single-slot reuse is gated by accounting.
 *
 * On x86 (TSO) torn fields must never appear; on weaker cores this same
 * binary becomes a real proof obligation for the target ports (VT). */

#define LITMUS_ITERS 200000u

typedef struct {
    uint64_t a;
    uint64_t b;
    uint64_t c;
    uint64_t d;
} litmus_desc_t;

static litmus_desc_t g_slots[2];
static _Atomic uint64_t g_pub_seq; /* producer -> consumer, release */
static _Atomic uint64_t g_con_seq; /* consumer -> producer, release */
static _Atomic uint32_t g_probe32;

static uint64_t pat(uint32_t i) {
    uint32_t x = i * 2654435761u + 0x9E3779B9u;
    x ^= x >> 15;
    x *= 2246822519u;
    x ^= x >> 13;
    return ((uint64_t)x << 32) | (uint64_t)(x ^ 0x5bf03635u);
}

#if defined(_WIN32)
static DWORD WINAPI producer(LPVOID param) {
#else
static void *producer(void *param) {
#endif
    (void)param;
    for (uint64_t i = 1; i <= (uint64_t)LITMUS_ITERS; i++) {
        /* Bounded queue: never overwrite a slot still in flight. */
        while (i - atomic_load_explicit(&g_con_seq, memory_order_acquire) > 1u) {
            /* spin: mailbox full */
        }
        litmus_desc_t *d = &g_slots[i & 1u];
        d->a = pat((uint32_t)i);
        d->b = pat((uint32_t)i + 1u);
        d->c = pat((uint32_t)i + 2u);
        d->d = pat((uint32_t)i + 3u);
        atomic_store_explicit(&g_pub_seq, i, memory_order_release);
    }
    return 0;
}

static void litmus_publication(void) {
    uint64_t last = 0;
    uint64_t torn = 0;
    uint64_t observed = 0;
    litmus_thread_t th;
    atomic_store_explicit(&g_pub_seq, 0, memory_order_relaxed);
    atomic_store_explicit(&g_con_seq, 0, memory_order_relaxed);
    CHECK(litmus_spawn(&th));
    while (last < (uint64_t)LITMUS_ITERS) {
        uint64_t p = atomic_load_explicit(&g_pub_seq, memory_order_acquire);
        if (p == last) {
            continue;
        }
        /* The slot for sequence p is stable: the producer may only be
         * writing the OTHER slot (queue bound is 1 in flight + 1 next). */
        const litmus_desc_t *d = &g_slots[p & 1u];
        uint64_t a = d->a, b = d->b, c = d->c, e = d->d;
        if (a != pat((uint32_t)p) || b != pat((uint32_t)p + 1u) ||
            c != pat((uint32_t)p + 2u) || e != pat((uint32_t)p + 3u)) {
            torn++;
        }
        last = p;
        observed++;
        atomic_store_explicit(&g_con_seq, p, memory_order_release);
    }
    CHECK(litmus_join(th));
    CHECK(observed > (uint64_t)LITMUS_ITERS / 10u); /* consumer kept up */
    CHECK(torn == 0);
}

static void litmus_lockfree_probe(void) {
    /* EX-02: small gate flags/counters must be lock-free on the target;
     * 64-bit scalars are deliberately NOT a portability precondition.
     * (Rev6: also exercise load/store so strict toolchains cannot fold
     * the object away as unused.) */
    atomic_store_explicit(&g_probe32, 0x2Au, memory_order_relaxed);
    CHECK(atomic_load_explicit(&g_probe32, memory_order_relaxed) == 0x2Au);
    CHECK(atomic_is_lock_free(&g_probe32));
}

void suite_litmus(void) {
    litmus_lockfree_probe();
    litmus_publication();
}
