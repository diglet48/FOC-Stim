#include "threephase_math.h"

#include "utils.h"
#include "foc_utils.h"

Vec3f threephase_calibration_to_amplitude(Vec3f calibration_vector_in_db) {
    // normalize calibration values
    float calib_max = calibration_vector_in_db.max();
    calibration_vector_in_db = calibration_vector_in_db - calib_max;
    return Vec3f{
        powf(10, calibration_vector_in_db.a / 10),
        powf(10, calibration_vector_in_db.b / 10),
        powf(10, calibration_vector_in_db.c / 10),
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
    Vec3f calibration_vector_in_db,
    float reduction_in_center,
    bool flip_polarity, float start_angle)
{
    Vec3f max_amplitude = threephase_calibration_to_amplitude(calibration_vector_in_db);

    Vec3f current_to_power{
        1 / max_amplitude.a,
        1 / max_amplitude.b,
        1 / max_amplitude.c
    };

    // transform position to electrode current
    // accounting for calibration
    Interpolator interpolator(max_amplitude);
    Vec3f output_current = interpolator.interpolate(alpha, beta);

    // normalize intensity to 1
    float intensity = threephase_intensity(output_current * current_to_power, reduction_in_center);
    output_current = output_current / intensity;

    // project points on complex plane
    ComplexThreephasePoints complex_points = threephase_electrode_amplitude_to_complex_points(output_current);
    return threephase_permute_complex_points(complex_points, flip_polarity, start_angle, pulse_amplitude);
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
