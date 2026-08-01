// Minimal test helper. Each test file is its own executable registered with
// ctest: it prints failures as file:line and exits non-zero if any check
// failed.
#ifndef BMCV_TESTKIT_H_
#define BMCV_TESTKIT_H_

#include <math.h>
#include <stdio.h>

static int testkit_checks       = 0;
static int testkit_failures     = 0;
static const char* testkit_test = "";

#define TEST_CASE(name) static void name(void)

#define RUN_TEST(name)                                                                                                                     \
  do                                                                                                                                       \
  {                                                                                                                                        \
    testkit_test = #name;                                                                                                                  \
    name();                                                                                                                                \
  } while (0)

#define CHECK(cond)                                                                                                                        \
  do                                                                                                                                       \
  {                                                                                                                                        \
    testkit_checks++;                                                                                                                      \
    if (!(cond))                                                                                                                           \
    {                                                                                                                                      \
      testkit_failures++;                                                                                                                  \
      fprintf(stderr, "FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, testkit_test, #cond);                                                    \
    }                                                                                                                                      \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                                                                                              \
  do                                                                                                                                       \
  {                                                                                                                                        \
    testkit_checks++;                                                                                                                      \
    double _a = (a), _b = (b), _eps = (eps);                                                                                               \
    if (fabs(_a - _b) > _eps)                                                                                                              \
    {                                                                                                                                      \
      testkit_failures++;                                                                                                                  \
      fprintf(stderr, "FAIL %s:%d [%s] %s ~= %s (got %f vs %f, tol %f)\n", __FILE__, __LINE__, testkit_test, #a, #b, _a, _b, _eps);         \
    }                                                                                                                                      \
  } while (0)

// Call at the end of main(): prints the summary and yields the exit code.
static inline int testkit_summary(const char* file)
{
  fprintf(stdout, "%s: %d checks, %d failed\n", file, testkit_checks, testkit_failures);
  return testkit_failures ? 1 : 0;
}

#define TESTKIT_SUMMARY() testkit_summary(__FILE__)

#endif /* BMCV_TESTKIT_H_ */
