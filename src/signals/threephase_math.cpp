#include "threephase_math.h"

#include "utils.h"
#include "foc_utils.h"

// See ThreePhaseHardwareCalibration in restim
// calibration coefs are in alpha-beta space
static void get_calibration_coefs(
    float left_right, float up_down,
    float *s11, float *s12,
    float *s21, float *s22)
{
    // generate_transform_in_ab()
    float theta = atan2f(left_right, up_down) / 2;
    float r = norm(up_down, left_right);
    float a = sinf(theta);
    float b = cosf(theta);
    float scale = 1.f / powf(10, (r / 10));

    if (r == 0)
    {
        // identity matrix
        *s11 = 1;
        *s12 = 0;
        *s21 = 0;
        *s22 = 1;
    }
    else
    {
        float s = (scale - 1);

        // normalization parameter to generate matrix with eigenvalues <= 1
        float normalization = 1;
        if (scale >= 1) {
            normalization = 1 / scale; // scale is the largest eigenvalue
        }

        // scale_in_arbitrary_direction()
        *s11 = (1 + s * a * a) * normalization;
        *s12 = (s * a * b) * normalization;
        *s21 = (s * a * b) * normalization;
        *s22 = (1 + s * b * b) * normalization;
    }
}

ComplexThreephasePoints project_threephase(
    float pulse_amplitude,
    float alpha,
    float beta,
    float center_calibration,
    float up_down_calibration,
    float left_right_calibration,
    bool flip_polarity,
    float start_angle)
{
    // constrain (alpha, beta) to unit circle
    float r = sqrtf(alpha * alpha + beta * beta);
    if (r > 1)
    {
        alpha /= r;
        beta /= r;
        r = 1;
    }

    // https://github.com/diglet48/restim/wiki/software-basics
    // base projection matrix in ab space
    float a11 = 0.5f * (2 - r + alpha);
    float a12 = 0.5f * beta;
    float a21 = 0.5f * beta;
    float a22 = 0.5f * (2 - r - alpha);

    // calibration matrix
    float t11, t12, t21, t22;
    get_calibration_coefs(left_right_calibration, up_down_calibration, &t11, &t12, &t21, &t22);

    // calibration * projection
    float b11 = t11 * a11 + t12 * a21;
    float b12 = t11 * a12 + t12 * a22;
    float b21 = t21 * a11 + t22 * a21;
    float b22 = t21 * a12 + t22 * a22;

    // ab transform
    float ab11 = 1;
    float ab12 = 0;
    float ab21 = -0.5f;
    float ab22 = -_SQRT3_2; // left
    float ab31 = -0.5f;
    float ab32 = _SQRT3_2;  // right

    // complex points
    Complex p1(ab11 * b11 + ab12 * b21, ab11 * b12 + ab12 * b22);
    Complex p2(ab21 * b11 + ab22 * b21, ab21 * b12 + ab22 * b22);
    Complex p3(ab31 * b11 + ab32 * b21, ab31 * b12 + ab32 * b22);

    // center calibration
    float ratio = powf(10, (center_calibration / 10));
    if (ratio <= 1)
    {
        pulse_amplitude *= lerp(r, ratio, 1);
    }
    else
    {
        pulse_amplitude *= lerp(r, 1, 1 / ratio);
    }

    if (flip_polarity) {
        p1 = Complex(-p1.real(), p1.imag());
        p2 = Complex(-p2.real(), p2.imag());
        p3 = Complex(-p3.real(), p3.imag());
    }

    Complex rotation(cosf(start_angle), sinf(start_angle));

    return ComplexThreephasePoints{
        .p1 = p1 * rotation * pulse_amplitude,
        .p2 = p2 * rotation * pulse_amplitude,
        .p3 = p3 * rotation * pulse_amplitude
    };
}

class Interpolator {
public:
    Interpolator(Vec3f max_amplitude) {
        this->max_amplitude = max_amplitude;
        float p;

        // pre-calculate the currents for the fixed points A, B, C
        // TODO: check p <= 1?
        p = max_amplitude.a / (max_amplitude.b + max_amplitude.c);
        A = Vec3f{max_amplitude.a * 1, max_amplitude.b * p, max_amplitude.c * p};
        p = max_amplitude.b / (max_amplitude.a + max_amplitude.c);
        B = Vec3f{max_amplitude.a * p, max_amplitude.b * 1, max_amplitude.c * p};
        p = max_amplitude.c / (max_amplitude.a + max_amplitude.b);
        C = Vec3f{max_amplitude.a * p, max_amplitude.b * p, max_amplitude.c * 1};

        AB = minimize_c({max_amplitude.a, max_amplitude.b, 0});
        AC = minimize_b({max_amplitude.a, 0, max_amplitude.c});
        BC = minimize_a({0, max_amplitude.b, max_amplitude.c});
    }

    Vec3f interpolate(float alpha, float beta) {
        float theta = atan2f(beta, alpha) * float(180 / M_PI);
        float r = norm(alpha, beta);
        if (r >= 1) {
            r = 1;
        }

        Vec3f out{1, 1, 1};
        if (theta >= 120) {        // segment between BC and B
            float t = 1 - (theta - 120) / 60;   // t = 1 near B
            float b = max_amplitude.b;
            float c = BC.c + t * t * (B.c - BC.c);
            out = minimize_a({0, b, c});
        } else if (theta >= 60) {  // segment between B and AB
            float t = 1 - (120 - theta) / 60;   // t = 1 near B
            float a = AB.a + t * t * (B.a - AB.a);
            float b = max_amplitude.b;
            out = minimize_c({a, b, 0});
        } else if (theta >= 0) {    // segment between AB and A
            float t = 1 - (theta - 0) / 60;   // t = 1 near A
            float a = max_amplitude.a;
            float b = AB.b + t * t * (A.b - AB.b);
            out = minimize_c({a, b, 0});
        } else if (theta >= -60) {   // segment between A and AC
            float t = 1 - (0 - theta) / 60;   // t = 1 near A
            float a = max_amplitude.a;
            float c = AC.c + t * t * (A.c - AC.c);
            out = minimize_b({a, 0, c});
        } else if (theta >= -120) {  // segment between AC and C
            float t = 1 - (theta + 120) / 60;   // t = 1 near C
            float a = AC.a + t * t * (C.a - AC.a);
            float c = max_amplitude.c;
            out = minimize_b({a, 0, c});
        } else { // if (theta >= -180) {  // segment between C and BC
            float t = 1 - (-120 - theta) / 60;   // t = 1 near C
            float b = BC.b + t * t * (C.b - BC.b);
            float c = max_amplitude.c;
            out = minimize_a({0, b, c});
        }

        out = out * r + max_amplitude * (1 - r);
        return out;
    }

private:
    Vec3f minimize_a(Vec3f p) {
        p.a = 0;
        p.a = std::max(p.max() * 2 - p.sum(), 0.0f);
        return p;
    }

    Vec3f minimize_b(Vec3f p) {
        p.b = 0;
        p.b = std::max(p.max() * 2 - p.sum(), 0.0f);
        return p;
    }

    Vec3f minimize_c(Vec3f p) {
        p.c = 0;
        p.c = std::max(p.max() * 2 - p.sum(), 0.0f);
        return p;
    }

    Vec3f max_amplitude;
    Vec3f A;
    Vec3f B;
    Vec3f C;
    Vec3f AB;
    Vec3f AC;
    Vec3f BC;
};

ComplexThreephasePoints project_threephase_2(
    float pulse_amplitude,
    float alpha, float beta,
    float center_calibration, float up_down_calibration, float left_right_calibration,
    bool flip_polarity, float start_angle)
{
    Vec3f max_amplitude = threephase_calibration_to_max_amplitude(up_down_calibration, left_right_calibration);

    Vec3f current_to_power{
        1 / max_amplitude.a,
        1 / max_amplitude.b,
        1 / max_amplitude.c
    };

    // BSP_PrintDebugMsg("max amp: %f %f %f", max_amplitude.a, max_amplitude.b, max_amplitude.c);

    // transform position to electrode current
    // accounting for calibration
    Interpolator interpolator(max_amplitude);
    Vec3f output_current = interpolator.interpolate(alpha, beta);

    // BSP_PrintDebugMsg("outp: %f %f %f", output_current.a, output_current.b, output_current.c);

    // normalize intensity to 1
    float reduction_in_center = 0;
    if (center_calibration <= 0) {
        reduction_in_center = 1 - std::pow(10.0, center_calibration / 10);
    }
    float intensity = threephase_intensity(output_current * current_to_power, reduction_in_center);
    // BSP_PrintDebugMsg("intensity: %f", intensity);
    output_current = output_current / intensity;

    // project points on complex plane
    ComplexThreephasePoints complex_points = threephase_electrode_amplitude_to_complex_points(output_current);
    return threephase_permute_complex_points(complex_points, flip_polarity, start_angle, pulse_amplitude);
}

// Vec3f threephase_ab_to_electrode_intensity(float alpha, float beta)
// {
//     float r = norm(alpha, beta);
//     if (r > 1) {
//         alpha /= r;
//         beta /= r;
//         r = 1;
//     }

//     // halve the angle
//     float theta = atan2f(beta, alpha) / 2;
//     Vec2f p{r * cosf(theta), r * sinf(theta)};

//     return Vec3f{
//         std::abs(dot(p, {1, 0})) + (1 - r),
//         std::abs(dot(p, {-0.5f, -_SQRT3_2})) + (1 - r),
//         std::abs(dot(p, {-0.5f, _SQRT3_2})) + (1 - r),
//     };
// }

Vec3f threephase_calibration_to_max_amplitude(float up_down_calibration, float left_right_calibration)
{
    // calibration matrix
    float t11, t12, t21, t22;
    get_calibration_coefs(left_right_calibration, up_down_calibration, &t11, &t12, &t21, &t22);

    Vec2f a{1, 0};
    Vec2f b{-0.5f, -_SQRT3_2};
    Vec2f c{-0.5f, _SQRT3_2};

    a = {t11 * a.a + t12 * a.b, t21 * a.a + t22 * a.b};
    b = {t11 * b.a + t12 * b.b, t21 * b.a + t22 * b.b};
    c = {t11 * c.a + t12 * c.b, t21 * c.a + t22 * c.b};

    Vec3f calib = Vec3f(a.norm(), b.norm(), c.norm());
    calib = calib / calib.max();

    return calib;
}

Vec3f threephase_interpolate(float alpha, float beta, Vec3f max_amplitude)
{
    Interpolator interpolator(max_amplitude);
    return interpolator.interpolate(alpha, beta);
}


float threephase_intensity(Vec3f electrode_power_in_percent, float reduction_in_center)
{
    // Desired properties:
    //
    // The following inputs return a value very close to 1,
    // as they are (empirically) close in intensity:
    // 1,   .5, .5
    // .86, .86, .86
    // .86, .86, 0
    //
    // intensity(vec * c) == intensity(vec) * c
    //
    // to get these properties, the intensity is the p-norm of the largest
    // 2 values.

    auto norm = [](float a, float b, float exponent) {
        return powf(powf(a, exponent) + powf(b, exponent), 1/exponent);
    };

    // get largest and second-largest elements. Ignore the rest.
    Vec3f order = electrode_power_in_percent.sorted();
    float rank_1 = order.c;
    float rank_2 = order.b;
    rank_2 = std::max(1/2.f * rank_1, rank_2);

    // clamp to sensible values, and to avoid division by zero.
    reduction_in_center = std::clamp<float>(reduction_in_center, .001f, .20f);
    float exponent = logf(2) / logf(1 / (1 - reduction_in_center));
    // clamp to avoid float over/underflow
    exponent = std::clamp<float>(exponent, 1, 50);

    return rank_1 * norm(1, rank_2 / rank_1, exponent); // = norm(rank1, rank2, exponent)
}

ComplexThreephasePoints threephase_electrode_amplitude_to_complex_points(Vec3f amplitudes)
{
    Complex p1{amplitudes.a, 0};
    Complex p2, p3;
    Complex mid = -(p1 * 0.5f);

    split_point(-p1, amplitudes.b, amplitudes.c, &p2, &p3);
    return ComplexThreephasePoints{p1, p2, p3};
}

ComplexThreephasePoints threephase_permute_complex_points(ComplexThreephasePoints points, bool flip_polarity, float start_angle, float amplitude_multiplicator)
{
    if (flip_polarity) {
        points.p1 = Complex(-points.p1.real(), points.p1.imag());
        points.p2 = Complex(-points.p2.real(), points.p2.imag());
        points.p3 = Complex(-points.p3.real(), points.p3.imag());
    }

    Complex rotation(cosf(start_angle), sinf(start_angle));

    points.p1 = points.p1 * rotation * amplitude_multiplicator;
    points.p2 = points.p2 * rotation * amplitude_multiplicator;
    points.p3 = points.p3 * rotation * amplitude_multiplicator;
    return points;
}
