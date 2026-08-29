/* minitest.h - a few macros, no dependencies. Each test file is its own
 * executable; the Makefile runs them all. */
#ifndef MINITEST_H
#define MINITEST_H

#include <math.h>
#include <stdio.h>

static int mt_checks = 0;
static int mt_fails = 0;

#define ASSERT_TRUE(cond) do { \
    mt_checks++; \
    if (!(cond)) { mt_fails++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define ASSERT_NEAR(a, b, eps) do { \
    float mt_a = (float)(a), mt_b = (float)(b), mt_e = (float)(eps); \
    mt_checks++; \
    if (!(fabsf(mt_a - mt_b) <= mt_e)) { \
        mt_fails++; \
        printf("  FAIL %s:%d: %s = %g, expected %s = %g (tol %g)\n", \
               __FILE__, __LINE__, #a, (double)mt_a, #b, (double)mt_b, (double)mt_e); \
    } \
} while (0)

#define ASSERT_EQ_INT(a, b) do { \
    long mt_a = (long)(a), mt_b = (long)(b); \
    mt_checks++; \
    if (mt_a != mt_b) { \
        mt_fails++; \
        printf("  FAIL %s:%d: %s = %ld, expected %s = %ld\n", __FILE__, __LINE__, #a, mt_a, #b, mt_b); \
    } \
} while (0)

#define RUN_TEST(fn) do { printf("- %s\n", #fn); fn(); } while (0)

#define MINITEST_MAIN_END() do { \
    printf("%s: %d checks, %d failures\n", __FILE__, mt_checks, mt_fails); \
    return mt_fails ? 1 : 0; \
} while (0)

#endif /* MINITEST_H */
