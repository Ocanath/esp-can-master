#include "unity.h"

/* Declare your test functions */
void test_ik_angles(void);

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_ik_angles);

    return UNITY_END();
}
