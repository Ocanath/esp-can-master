/* test_ik_angles.c */

#include <stdio.h>
#include <math.h>
#include "../Core/Inc/sauron-eye-closedform-ik.h"
#include "unity.h"

/* Some toolchains don’t define M_PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void test_ik_angles(void) 
{
    float th1 = 0.0f, th2 = 0.0f;
    get_ik_angles_float(-3.0f, 3.0f, 10.0f, &th1, &th2);

    float th1_deg = th1 * 180.0f / M_PI;
    float th2_deg = th2 * 180.0f / M_PI;

    /* ±0.02° tolerance */
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -16.70f, th1_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 15.39f, th2_deg);
}


void test_ik_angles_dbl(void)
{
    double th1 = 0.0f, th2 = 0.0f;
    get_ik_angles_double(-3.0f, 3.0f, 10.0f, &th1, &th2);

    double th1_deg = th1 * 180.0 / M_PI;
    double th2_deg = th2 * 180.0 / M_PI;

    /* ±0.02° tolerance */
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -16.70, (float)th1_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 15.39, (float)th2_deg);
}

