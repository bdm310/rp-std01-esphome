#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <stdlib.h>

static int test_failures;

/* The thermistor's raw reading at room temperature, measured on one unit. */
#define THERM_ROOM 933u

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);     \
      test_failures++;                                                    \
    }                                                                     \
  } while (0)

#define CHECK_EQ(a, b)                                                                     \
  do {                                                                                     \
    long long va_ = (long long) (a), vb_ = (long long) (b);                                \
    if (va_ != vb_) {                                                                      \
      printf("%s:%d: %s == %lld, expected %s == %lld\n", __FILE__, __LINE__, #a, va_, #b,  \
             vb_);                                                                         \
      test_failures++;                                                                     \
    }                                                                                      \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                \
  do {                                                                                       \
    double va_ = (a), vb_ = (b), d_ = va_ - vb_;                                             \
    if (!((d_ < 0 ? -d_ : d_) <= (tol))) {                                                   \
      printf("%s:%d: %s == %g, expected %g +- %g\n", __FILE__, __LINE__, #a, va_, vb_,       \
             (double) (tol));                                                                \
      test_failures++;                                                                       \
    }                                                                                        \
  } while (0)

#define RUN(fn)            \
  do {                     \
    printf("  %s\n", #fn); \
    fn();                  \
  } while (0)

#define TEST_EXIT()                                  \
  do {                                               \
    if (test_failures) {                             \
      printf("%d check(s) failed\n", test_failures); \
      return EXIT_FAILURE;                           \
    }                                                \
    return EXIT_SUCCESS;                             \
  } while (0)

#endif
