#include "board.h"
#include "protocol.h"
#include "safety.h"
#include "test.h"

static void test_not_ready(void) {
  safety_t s;
  safety_init(&s);
  CHECK_EQ(safety_therm(&s, 0, 0), F_THERM);
}

static void test_normal_and_trip(void) {
  safety_t s;
  safety_init(&s);
  CHECK_EQ(safety_therm(&s, THERM_ROOM, 1), 0);
  CHECK_EQ(safety_therm(&s, THERM_TRIP + 1, 1), 0);
  CHECK_EQ(safety_therm(&s, THERM_TRIP, 1), F_OVERTEMP);
  CHECK_EQ(safety_therm(&s, THERM_CLEAR - 1, 1), F_OVERTEMP); /* hysteresis */
  CHECK_EQ(safety_therm(&s, THERM_CLEAR, 1), 0);
}

static void test_hot_at_start(void) {
  /* A restart with the heatsink still warm, but below the trip point, is fine. */
  safety_t s;
  safety_init(&s);
  CHECK_EQ(safety_therm(&s, THERM_TRIP + 20, 1), 0);
  safety_init(&s);
  CHECK_EQ(safety_therm(&s, THERM_TRIP - 20, 1), F_OVERTEMP);
}

static void test_open_short(void) {
  safety_t s;
  safety_init(&s);
  CHECK_EQ(safety_therm(&s, THERM_OPEN_SHORT_HIGH + 1, 1), F_THERM); /* open: the pull-up wins */
  CHECK_EQ(safety_therm(&s, THERM_ROOM, 1), 0);
  CHECK_EQ(safety_therm(&s, THERM_OPEN_SHORT_LOW - 1, 1), F_THERM | F_OVERTEMP);
}

int main(void) {
  RUN(test_not_ready);
  RUN(test_normal_and_trip);
  RUN(test_hot_at_start);
  RUN(test_open_short);
  TEST_EXIT();
}
