#include "signals/threephase_math.h"

#include <unity.h>
#include <random>
#include <algorithm>


static void UnityAssertEqualVec2f(Vec2f expected, Vec2f actual, const UNITY_LINE_TYPE lineNumber) {
    float tolerance = 0.001f;
    if ((abs(expected.a - actual.a) > tolerance) || \
        (abs(expected.b - actual.b) > tolerance) ||
        isnan(actual.a) || isnan(actual.b))
    {
        UnityPrint(" Expected {");
        UnityPrintFloat(expected.a);
        UnityPrint(", ");
        UnityPrintFloat(expected.b);
        UnityPrint("} Was {");
        UnityPrintFloat(actual.a);
        UnityPrint(", ");
        UnityPrintFloat(actual.b);
        UnityPrint("} ");
        UNITY_TEST_FAIL(lineNumber, NULL);
    }
}

static void UnityAssertEqualVec3f(Vec3f expected, Vec3f actual, const UNITY_LINE_TYPE lineNumber) {
    float tolerance = 0.001f;
    if ((abs(expected.a - actual.a) > tolerance) || \
        (abs(expected.b - actual.b) > tolerance) || \
        (abs(expected.c - actual.c) > tolerance) ||
        isnan(actual.a) || isnan(actual.b) || isnan(actual.c))
    {
        UnityPrint(" Expected {");
        UnityPrintFloat(expected.a);
        UnityPrint(", ");
        UnityPrintFloat(expected.b);
        UnityPrint(", ");
        UnityPrintFloat(expected.c);
        UnityPrint("} Was {");
        UnityPrintFloat(actual.a);
        UnityPrint(", ");
        UnityPrintFloat(actual.b);
        UnityPrint(", ");
        UnityPrintFloat(actual.c);
        UnityPrint("} ");
        UNITY_TEST_FAIL(lineNumber, NULL);
    }
}

static void UnityAssertEqualComplex(Complex expected, Complex actual, const UNITY_LINE_TYPE lineNumber) {
    float tolerance = 0.001f;
    if ((abs(expected.real() - actual.real()) > tolerance) || \
        (abs(expected.imag() - actual.imag()) > tolerance))
    {
        UnityPrint(" Expected ");
        UnityPrintFloat(expected.real());
        UnityPrint(" + ");
        UnityPrintFloat(expected.imag());
        UnityPrint("i");
        UnityPrint(" Was ");
        UnityPrintFloat(actual.real());
        UnityPrint(" + ");
        UnityPrintFloat(actual.imag());
        UnityPrint("i ");
        UNITY_TEST_FAIL(lineNumber, NULL);
    }
}



#define TEST_ASSERT_EQUAL_VEC2(expected, actual) UnityAssertEqualVec2f((expected), (actual), __LINE__);
#define TEST_ASSERT_EQUAL_VEC3(expected, actual) UnityAssertEqualVec3f((expected), (actual), __LINE__);
#define TEST_ASSERT_EQUAL_COMPLEX(expected, actual) UnityAssertEqualComplex((expected), (actual), __LINE__);

static Vec3f calibration_none{1, 1, 1};

void test_3p_center() {
    TEST_ASSERT_EQUAL_VEC3(Vec3f({1, 1, 1}), threephase_interpolate(0, 0, calibration_none));
    // center with calibration
    TEST_ASSERT_EQUAL_VEC3(Vec3f({1, .9, .8}), threephase_interpolate(0, 0, {1, .9, .8}));
    TEST_ASSERT_EQUAL_VEC3(Vec3f({.8, .9, 1}), threephase_interpolate(0, 0, {.8, .9, 1}));
}

void test_3p_a() {
    // must be maximum amplitude on A, rest whatever
    TEST_ASSERT_EQUAL_VEC3(Vec3f({1, .5, .5}), threephase_interpolate(1, 0, calibration_none));
    TEST_ASSERT_EQUAL_VEC3(Vec3f({1, .75, .75}), threephase_interpolate(.5, 0, calibration_none));
}

void test_3p_b() {
    // must be maximum amplitude on B, rest whatever
    TEST_ASSERT_EQUAL_VEC3(Vec3f({.5, 1, .5}), threephase_interpolate(-0.5, sqrtf(3)/2, calibration_none));
    TEST_ASSERT_EQUAL_VEC3(Vec3f({.75, 1, .75}), threephase_interpolate(-0.25, sqrtf(3)/4, calibration_none));
}

void test_3p_c() {
    // must be maximum amplitude on C, rest whatever
    TEST_ASSERT_EQUAL_VEC3(Vec3f({.5, .5, 1}), threephase_interpolate(-0.5, -sqrtf(3)/2, calibration_none));
    TEST_ASSERT_EQUAL_VEC3(Vec3f({.75, .75, 1}), threephase_interpolate(-0.25, -sqrtf(3)/4, calibration_none));
}

void test_3p_calibration() {
    TEST_ASSERT_EQUAL_VEC3(Vec3f(1, 1, 1), threephase_calibration_to_max_amplitude(0, 0));

    // neutral
    TEST_ASSERT_EQUAL_VEC3(Vec3f(1, .85, .85), threephase_calibration_to_max_amplitude(1, 0));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(.834, 1, 1), threephase_calibration_to_max_amplitude(-1, 0));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(1, .507, .507), threephase_calibration_to_max_amplitude(10, 0));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(.115, 1, 1), threephase_calibration_to_max_amplitude(-10, 0));

    // left
    TEST_ASSERT_EQUAL_VEC3(Vec3f(.85, 1, .85), threephase_calibration_to_max_amplitude(-.5,-sqrtf(3)/2));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(1, .834, 1), threephase_calibration_to_max_amplitude(.5, sqrtf(3)/2));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(.507, 1, .507), threephase_calibration_to_max_amplitude(-5, -10 * sqrtf(3)/2));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(1, .115, 1), threephase_calibration_to_max_amplitude(5, 10 * sqrtf(3)/2));

    // right
    TEST_ASSERT_EQUAL_VEC3(Vec3f(.85, .85, 1), threephase_calibration_to_max_amplitude(-.5, sqrtf(3)/2));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(1, 1, .834), threephase_calibration_to_max_amplitude(.5, -sqrtf(3)/2));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(.507, .507, 1), threephase_calibration_to_max_amplitude(-5, 10 * sqrtf(3)/2));
    TEST_ASSERT_EQUAL_VEC3(Vec3f(1, 1, .115), threephase_calibration_to_max_amplitude(5, -10 * sqrtf(3)/2));
}

void test_3p_intensity() {
    float default_reduction = 0.14f;

    Vec3f vec1 = {1, 1/5.f, 1/5.f};
    TEST_ASSERT_EQUAL_FLOAT(1.008857f, threephase_intensity(vec1, default_reduction));

    // .86 = empirically nice value
    Vec3f vec2 = {.86, .86, .86};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1, threephase_intensity(vec2, default_reduction));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2, threephase_intensity(vec2 * 2, default_reduction));

    TEST_ASSERT_EQUAL_FLOAT(2.017713f, threephase_intensity(vec1 * 2, default_reduction));
    TEST_ASSERT_EQUAL_FLOAT(0.50442825f, threephase_intensity(vec1 / 2, default_reduction));
}

void test_3p_project() {
    {
        ComplexThreephasePoints points = threephase_electrode_amplitude_to_complex_points({1, 1, 1});
        TEST_ASSERT_EQUAL_COMPLEX(Complex(0, 0), points.p1 + points.p2 + points.p3);
        TEST_ASSERT_EQUAL_COMPLEX(Complex(1, 0), points.p1);
        TEST_ASSERT_EQUAL_COMPLEX(Complex(-0.5, -.866), points.p2);
        TEST_ASSERT_EQUAL_COMPLEX(Complex(-0.5, .866), points.p3);
    }
    {
        ComplexThreephasePoints points = threephase_electrode_amplitude_to_complex_points({0.5, 1, 0.8});
        TEST_ASSERT_EQUAL_COMPLEX(Complex(0, 0), points.p1 + points.p2 + points.p3);
        TEST_ASSERT_EQUAL_COMPLEX(Complex(.5, 0), points.p1);
        TEST_ASSERT_EQUAL_COMPLEX(Complex(-0.61, -.792), points.p2);
        TEST_ASSERT_EQUAL_COMPLEX(Complex(0.11, .792), points.p3);
    }
}

void tests_3p() {
    UNITY_BEGIN();
    RUN_TEST(test_3p_center);
    RUN_TEST(test_3p_a);
    RUN_TEST(test_3p_b);
    RUN_TEST(test_3p_c);
    RUN_TEST(test_3p_calibration);
    RUN_TEST(test_3p_intensity);
    RUN_TEST(test_3p_project);
    UNITY_END();
}