#include "unity.h"

/* according to chatgippity, unity requires these even if you don't use them.
* TODO: confirm this
*/
void setUp(void) { /* nothing */ }
void tearDown(void) { /* nothing */ }


/* Declare your test functions */
void test_ik_angles(void);
void test_ik_angles_dbl(void);
void test_sqrt(void);

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_ik_angles_dbl);
    RUN_TEST(test_ik_angles);

    return UNITY_END();
}
