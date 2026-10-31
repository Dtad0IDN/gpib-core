#include "test_util.h"
#include "vbus.h"

/* VB-01: wired-OR aggregation over ALL participants (simulation_plan S2). */
static void vb_01_wired_or(void) {
    vbus_t b;
    int p1, p2;
    vbus_init(&b);
    p1 = vbus_attach(&b);
    p2 = vbus_attach(&b);
    CHECK(p1 >= 0 && p2 >= 0 && p1 != p2);
    CHECK(!vbus_level(&b, VBL_NRFD));
    vbus_assert(&b, p1, VBL_NRFD);
    CHECK(vbus_level(&b, VBL_NRFD));
    vbus_assert(&b, p2, VBL_NRFD); /* shared assertion is legal */
    CHECK(vbus_level(&b, VBL_NRFD));
    vbus_release(&b, p1, VBL_NRFD);
    CHECK(vbus_level(&b, VBL_NRFD)); /* slowest participant still holds */
    vbus_release(&b, p2, VBL_NRFD);
    CHECK(!vbus_level(&b, VBL_NRFD));
    CHECK(b.conflicts == 0);
}

/* VB-02: contention on single-driver lines is an explicit model error. */
static void vb_02_contention(void) {
    vbus_t b;
    int p1, p2;
    uint8_t val;
    vbus_init(&b);
    p1 = vbus_attach(&b);
    p2 = vbus_attach(&b);
    vbus_assert(&b, p1, VBL_DAV);
    vbus_assert(&b, p2, VBL_DAV);
    CHECK(b.conflicts == 1); /* second DAV driver reported */
    vbus_release(&b, p2, VBL_DAV);
    vbus_drive_dio(&b, p1, 0x55);
    vbus_drive_dio(&b, p2, 0xAA);
    CHECK(b.conflicts == 2); /* two talkers */
    CHECK(vbus_dio_level(&b, &val) == -1);
    vbus_release_dio(&b, p2);
    CHECK(vbus_dio_level(&b, &val) == 0);
    CHECK(val == 0x55);
    /* T07: identical bytes from two talkers are STILL a contention --
     * the bus cannot know the values match without reading them, and a
     * second source is wrong regardless of the data. */
    vbus_drive_dio(&b, p2, 0x55);
    CHECK(b.conflicts == 3);
    vbus_release_dio(&b, p2);
    CHECK(b.conflicts == 3);
}

/* VB-04 (T07): parallel-poll DIO is a wired-OR domain -- several PPAS
 * responders may drive simultaneously (the OR of their PPR bits); a
 * talker crossing into the IDY window is a conflict. */
static void vb_04_pp_domain(void) {
    vbus_t b;
    int p1, p2, p3;
    uint8_t val;
    vbus_init(&b);
    p1 = vbus_attach(&b);
    p2 = vbus_attach(&b);
    p3 = vbus_attach(&b);
    vbus_pp_drive(&b, p1, 0x01); /* line 0 */
    vbus_pp_drive(&b, p2, 0x04); /* line 2 */
    CHECK(b.conflicts == 0);     /* wired-OR is the LEGAL PP case */
    CHECK(vbus_dio_level(&b, &val) == 0);
    CHECK(val == 0x05);          /* OR of both responders */
    vbus_pp_release(&b, p2);
    CHECK(vbus_dio_level(&b, &val) == 0);
    CHECK(val == 0x01);
    /* A talker during IDY crosses the domains: contention. */
    vbus_drive_dio(&b, p3, 0x80);
    CHECK(b.conflicts == 1);
    vbus_dio_level(&b, &val); /* still resolvable: one talker + pp */
    CHECK(val == 0x81);
    vbus_pp_release(&b, p1);
    vbus_release_dio(&b, p3);
    CHECK(vbus_dio_level(&b, &val) == -1); /* nobody drives */
}

/* VB-03: detach removes the participant from wired-OR aggregation; virtual
 * time is a plain deterministic counter. */
static void vb_03_detach_time(void) {
    vbus_t b;
    int p1, p2;
    vbus_init(&b);
    p1 = vbus_attach(&b);
    p2 = vbus_attach(&b);
    vbus_assert(&b, p1, VBL_NDAC);
    vbus_assert(&b, p2, VBL_NDAC);
    vbus_detach(&b, p1);
    CHECK(vbus_level(&b, VBL_NDAC)); /* p2 still holds */
    vbus_detach(&b, p2);
    CHECK(!vbus_level(&b, VBL_NDAC));
    vbus_tick(&b);
    vbus_tick(&b);
    CHECK(b.ticks == 2);
}

void suite_vbus(void) {
    vb_01_wired_or();
    vb_02_contention();
    vb_03_detach_time();
    vb_04_pp_domain();
}
