#ifndef VBUS_H
#define VBUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* S2 virtual shared bus (simulation_plan.md): wired-OR lines computed from
 * ALL participants, explicit direction/contention errors, deterministic
 * virtual time. Infrastructure only in this increment: the line-level core
 * handshake engine waits for the normative baseline (G-NORM, OPEN-01/02). */

#define VBUS_MAX_PART 16

typedef enum vbus_line {
    VBL_DAV = 0,
    VBL_NRFD,
    VBL_NDAC,
    VBL_ATN,
    VBL_EOI,
    VBL_IFC,
    VBL_REN,
    VBL_SRQ,
    VBL_COUNT
} vbus_line_t;

typedef struct vbus {
    uint16_t assert_mask[VBUS_MAX_PART]; /* wired-OR participation */
    bool dio_active[VBUS_MAX_PART];      /* talker domain (single owner) */
    uint8_t dio_value[VBUS_MAX_PART];
    bool pp_active[VBUS_MAX_PART];       /* parallel-poll domain (OR) */
    uint8_t pp_value[VBUS_MAX_PART];
    bool attached[VBUS_MAX_PART];
    uint32_t conflicts; /* direction/contention errors, never ignored */
    uint64_t ticks;     /* virtual time: one tick per scheduler round */
} vbus_t;

void vbus_init(vbus_t *b);
int vbus_attach(vbus_t *b);          /* returns participant id or -1 */
void vbus_attach_as(vbus_t *b, int id); /* attach with a fixed id */
void vbus_detach(vbus_t *b, int id);
void vbus_assert(vbus_t *b, int id, vbus_line_t line);
void vbus_release(vbus_t *b, int id, vbus_line_t line);
bool vbus_level(const vbus_t *b, vbus_line_t line); /* wired-OR */
/* Talker domain: at most ONE active driver. A second driver is a
 * CONTENTION even when the values match (T07: identical bytes from two
 * talkers used to pass silently). Crossing into the PP domain (or vice
 * versa) is a contention as well. */
void vbus_drive_dio(vbus_t *b, int id, uint8_t value);
void vbus_release_dio(vbus_t *b, int id);
/* Parallel-poll domain: wired-OR semantics -- every configured PPAS
 * responder may drive simultaneously; the bus value is the OR of all
 * PP drivers (488.1 2.9: PPR lines are open-collector). */
void vbus_pp_drive(vbus_t *b, int id, uint8_t value);
void vbus_pp_release(vbus_t *b, int id);
/* Returns 0 and the driven value, or -1 on data-line contention.
 * The value is the single talker byte OR-ed with every active PP bit. */
int vbus_dio_level(const vbus_t *b, uint8_t *out);
/* T8-01: honest safe_detached evidence -- the participant asserts NO
 * line, holds no talker DIO drivers and no PP drivers. */
bool vbus_participant_clean(const vbus_t *b, int id);
void vbus_tick(vbus_t *b);

#endif /* VBUS_H */
