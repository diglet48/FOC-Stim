#include "signals/fourphase_math_2.h"

#include <unity.h>
#include <random>
#include <algorithm>


static void UnityAssertEqualVec4f(Vec4f expected, Vec4f actual, const UNITY_LINE_TYPE lineNumber) {
    float tolerance = 0.001f;
    if ((abs(expected.a - actual.a) > tolerance) || \
        (abs(expected.b - actual.b) > tolerance) || \
        (abs(expected.c - actual.c) > tolerance) || \
        (abs(expected.d - actual.d) > tolerance) ||
        isnan(actual.a) || isnan(actual.b) || isnan(actual.c) || isnan(actual.d))
    {
        UnityPrint(" Expected {");
        UnityPrintFloat(expected.a);
        UnityPrint(", ");
        UnityPrintFloat(expected.b);
        UnityPrint(", ");
        UnityPrintFloat(expected.c);
        UnityPrint(", ");
        UnityPrintFloat(expected.d);
        UnityPrint("} Was {");
        UnityPrintFloat(actual.a);
        UnityPrint(", ");
        UnityPrintFloat(actual.b);
        UnityPrint(", ");
        UnityPrintFloat(actual.c);
        UnityPrint(", ");
        UnityPrintFloat(actual.d);
        UnityPrint("} ");
        UNITY_TEST_FAIL(lineNumber, NULL);
    }
}

#define TEST_ASSERT_EQUAL_VEC4(expected, actual) UnityAssertEqualVec4f((expected), (actual), __LINE__);

static Vec4f calibration_none{1, 1, 1, 1};

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_constrain(void) {

    TEST_ASSERT_EQUAL_VEC4(Vec4f({1, 1/3.f, 1/3.f, 1/3.f}), fourphase_constrain_coordinates({1, 0, 0, 0}));
    // out-of-bounds
    TEST_ASSERT_EQUAL_VEC4(Vec4f({1, 1/3.f, 1/3.f, 1/3.f}), fourphase_constrain_coordinates({5, 0, 0, 0}));
    // various combinations of under-specified commands.
    TEST_ASSERT_EQUAL_VEC4(Vec4f({1, .8666666, .8666666, .8666666}), fourphase_constrain_coordinates({.2, 0, 0, 0}));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.7333333, 1, .7333333, .7333333}), fourphase_constrain_coordinates({0, .4, 0, 0}));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.6, .6, 1, .6}), fourphase_constrain_coordinates({0, 0, .6, 0}));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.4666666, .4666666, .4666666, 1}), fourphase_constrain_coordinates({0, 0, 0, .8}));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.4, .4, .4, 1}), fourphase_constrain_coordinates({0, 0, 0, .9}));
}

void test_center(void) {
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.86, .86, .86, .86}), fourphase_interpolate({0, 0, 0, 0}, calibration_none));
    // center with calibration
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.86*.8, .86, .86, .86}), fourphase_interpolate({0, 0, 0, 0}, {.8, 1, 1, 1}));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.86, .86*.7, .86, .86}), fourphase_interpolate({0, 0, 0, 0}, {1, .7, 1, 1}));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.86, .86, .86f*.6f, .86}), fourphase_interpolate({0, 0, 0, 0}, {1, 1, .6, 1}));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.86, .86, .86, .86*.5}), fourphase_interpolate({0, 0, 0, 0}, {1, 1, 1, .5}));
}

void test_a(void) {
    // must be maximum amplitude on A, rest whatever
    TEST_ASSERT_EQUAL_VEC4(Vec4f({1, .333333, .333333, .333333}), fourphase_interpolate({1, 0, 0, 0}, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.93, .596666, .596666, .596666}), fourphase_interpolate({.5, 0, 0, 0}, calibration_none));
}

void test_b(void) {
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.333333, 1, .333333, .333333}), fourphase_interpolate({0, 1, 0, 0}, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.596666, .93, .596666, .596666}), fourphase_interpolate({0, .5, 0, 0}, calibration_none));
}

void test_c(void) {
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.333333, .333333, 1, .333333}), fourphase_interpolate({0, 0, 1, 0}, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.596666, .596666, .93, .596666}), fourphase_interpolate({0, 0, .5, 0}, calibration_none));
}

void test_d(void) {
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.333333, .333333, .333333, 1}), fourphase_interpolate({0, 0, 0, 1}, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f({.596666, .596666, .596666, .93}), fourphase_interpolate({0, 0, 0, .5}, calibration_none));
}

void test_a_to_ab() {
    Vec4f A = {1, 1/3.f, 1/3.f, 1/3.f};
    Vec4f AB = {1, 1, 0, 0};

    // exact values not too critical, but A should decrease, B increase and C/D minimal
    TEST_ASSERT_EQUAL_VEC4(Vec4f(1.000, 0.333, 0.333, 0.333), fourphase_interpolate(A * 1 + AB * 0, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.998, 0.398, 0.300, 0.300), fourphase_interpolate(A * .9 + AB * .1, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.993, 0.460, 0.266, 0.266), fourphase_interpolate(A * .8 + AB * .2, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.985, 0.518, 0.233, 0.233), fourphase_interpolate(A * .7 + AB * .3, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.973, 0.573, 0.200, 0.200), fourphase_interpolate(A * .6 + AB * .4, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.958, 0.625, 0.166, 0.166), fourphase_interpolate(A * .5 + AB * .5, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.940, 0.673, 0.133, 0.133), fourphase_interpolate(A * .4 + AB * .6, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.918, 0.718, 0.100, 0.100), fourphase_interpolate(A * .3 + AB * .7, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.893, 0.760, 0.067, 0.067), fourphase_interpolate(A * .2 + AB * .8, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.865, 0.798, 0.033, 0.033), fourphase_interpolate(A * .1 + AB * .9, calibration_none));
    TEST_ASSERT_EQUAL_VEC4(Vec4f(0.833, 0.833, 0.000, 0.000), fourphase_interpolate(A * 0 + AB * 1, calibration_none));
}

void test_a_to_between_ab_and_ac() {
    Vec4f A = {1, 1/3.f, 1/3.f, 1/3.f};
    Vec4f mid = {1, .5, .5, 0}; // between AB and AC

    Vec4f previous = fourphase_interpolate(A, calibration_none);

    for (float s = 0.05f; s <= 1; s += 0.05f) {
        Vec4f pos = A * (1 - s) + mid * s;
        Vec4f powers = fourphase_interpolate(pos, calibration_none);
        // A power decreases every step
        TEST_ASSERT_LESS_THAN_FLOAT(previous.a, powers.a);
        // B and C power increases every step
        TEST_ASSERT_GREATER_THAN_FLOAT(previous.b, powers.b);
        TEST_ASSERT_GREATER_THAN_FLOAT(previous.c, powers.c);
        // B and C equal.
        TEST_ASSERT_EQUAL_FLOAT(powers.b, powers.c);
        previous = powers;
    }
}

void test_continuity() {
    // generate N random test vectors and small disturbances.
    // Given small disturbance in input position, difference in output must also be small.

    std::uniform_real_distribution<float> position_distribution(0, 1);
    std::uniform_real_distribution<float> disturbance_distribution(-0.01f, .01f);

    for (int seed = 0; seed < 1000; seed++) {
        std::minstd_rand0 gen(seed);

        Vec4f random_disturbance(disturbance_distribution(gen), disturbance_distribution(gen), disturbance_distribution(gen), disturbance_distribution(gen));

        Vec4f point1 = Vec4f(position_distribution(gen), position_distribution(gen), position_distribution(gen), position_distribution(gen));
        point1 = fourphase_constrain_coordinates(point1);
        Vec4f point2 = fourphase_constrain_coordinates(point1 + random_disturbance);

        Vec4f out1 = fourphase_interpolate(point1, calibration_none);
        Vec4f out2 = fourphase_interpolate(point2, calibration_none);

        // TEST_PRINTF("seed: %i", seed);
        // TEST_PRINTF("point1: %f %f %f %f", point1.a, point1.b, point1.c, point1.d);
        // TEST_PRINTF("point2: %f %f %f %f", point2.a, point2.b, point2.c, point2.d);
        // TEST_PRINTF("out1: %f %f %f %f", out1.a, out1.b, out1.c, out1.d);
        // TEST_PRINTF("out2: %f %f %f %f", out2.a, out2.b, out2.c, out2.d);

        float tolerance = 0.025f;
        TEST_ASSERT_FLOAT_WITHIN(tolerance, out2.a, out1.a);
        TEST_ASSERT_FLOAT_WITHIN(tolerance, out2.b, out1.b);
        TEST_ASSERT_FLOAT_WITHIN(tolerance, out2.c, out1.c);
        TEST_ASSERT_FLOAT_WITHIN(tolerance, out2.d, out1.d);
    }
}

void test_intensity() {
    Vec4f vec1 = {1, 1/3.f, 1/3.f, 1/3.f};
    TEST_ASSERT_EQUAL_FLOAT(1, 1/fourphase_intensity(vec1));

    // .86 = empirically nice value
    Vec4f vec2 = {.86, .86, .86, .86};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1, fourphase_intensity(vec2));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2, fourphase_intensity(vec2 * 2));
}

static Vec4f calculate_maximum_amplitudes2(Vec4f calibration_vector_in_db) {
    // normalize calibration values
    float calib_max = calibration_vector_in_db.max();
    calibration_vector_in_db = calibration_vector_in_db - calib_max;
    return Vec4f{
        powf(10, calibration_vector_in_db.a / 10),
        powf(10, calibration_vector_in_db.b / 10),
        powf(10, calibration_vector_in_db.c / 10),
        powf(10, calibration_vector_in_db.d / 10)
    };
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_center);
    RUN_TEST(test_constrain);
    RUN_TEST(test_a);
    RUN_TEST(test_b);
    RUN_TEST(test_c);
    RUN_TEST(test_d);
    RUN_TEST(test_a_to_ab);
    RUN_TEST(test_a_to_between_ab_and_ac);
    RUN_TEST(test_continuity);
    RUN_TEST(test_intensity);
    UNITY_END();

    return 0;
}