#include "fourphase_math_2.h"

#include "fourphase_math.h"
#include "vec.h"
#include "foc_utils.h"

#include <algorithm>
#include <cmath>

namespace PointCoords {
    // origin
    constexpr Vec4f O{1, 1, 1, 1};

    // extrema points, where intensity of A is maximum and B/C/D is minimum
    constexpr Vec4f A{1, 1/3.f, 1/3.f, 1/3.f};
    constexpr Vec4f B{1/3.f, 1, 1/3.f, 1/3.f};
    constexpr Vec4f C{1/3.f, 1/3.f, 1, 1/3.f};
    constexpr Vec4f D{1/3.f, 1/3.f, 1/3.f, 1};

    // inbetween points, where intensity of AB is maximum and CD is minimum
    constexpr Vec4f AB{1, 1, 0, 0};
    constexpr Vec4f AC{1, 0, 1, 0};
    constexpr Vec4f AD{1, 0, 0, 1};
    constexpr Vec4f BC{0, 1, 1, 0};
    constexpr Vec4f BD{0, 1, 0, 1};
    constexpr Vec4f CD{0, 0, 1, 1};

    // More points...
    constexpr Vec4f ABC{1, 1, 1, 0};
    constexpr Vec4f ABD{1, 1, 0, 1};
    constexpr Vec4f ACD{1, 0, 1, 1};
    constexpr Vec4f BCD{0, 1, 1, 1};

    // normals for A == 1
    constexpr Vec4f O_AB_AC_normal{0, 1, 1, -1};    // normal for the plane O AB AC
    constexpr Vec4f O_AB_AD_normal{0, 1, -1, 1};
    constexpr Vec4f O_AC_AD_normal{0, -1, 1, 1};

    // normals for B == 1
    constexpr Vec4f O_AB_BC_normal{1, 0, 1, -1};
    constexpr Vec4f O_AB_BD_normal{1, 0, -1, 1};
    constexpr Vec4f O_BC_BD_normal{-1, 0, 1, 1};

    // normals for C == 1
    constexpr Vec4f O_AC_BC_normal{1, 1, 0, -1};
    constexpr Vec4f O_AC_CD_normal{1, -1, 0, 1};
    constexpr Vec4f O_BC_CD_normal{-1, 1, 0, 1};

    // normals for D == 1
    constexpr Vec4f O_AD_BD_normal{1, 1, -1, 0};
    constexpr Vec4f O_AD_CD_normal{1, -1, 1, 0};
    constexpr Vec4f O_BD_CD_normal{-1, 1, 1, 0};
};

/**
 * input: a position vector
 * output: a valid position vector that lies on the edges of an unit hypercube,
 *      meaning at least one component is 1 (exact),
 *      and the 3 smallest components sum to at least 1.
 */
Vec4f fourphase_constrain_coordinates(Vec4f in) {
    float a = std::clamp<float>(in.a, 0.f, 1.f);
    float b = std::clamp<float>(in.b, 0.f, 1.f);
    float c = std::clamp<float>(in.c, 0.f, 1.f);
    float d = std::clamp<float>(in.d, 0.f, 1.f);

    float s_a = std::min(-a + b + c + d, 0.f) / -3;
    float s_b = std::min( a - b + c + d, 0.f) / -3;
    float s_c = std::min( a + b - c + d, 0.f) / -3;
    float s_d = std::min( a + b + c - d, 0.f) / -3;

    a += s_b + s_c + s_d;
    b += s_a + s_c + s_d;
    c += s_a + s_b + s_d;
    d += s_a + s_b + s_c;

    in = {a, b, c, d};


    float maximum = std::max({in.a, in.b, in.c, in.d});
    if (maximum < 1.f) {
        float to_add = 1 - maximum;
        if (in.a == maximum) {
            in = in + to_add;
            in.a = 1.f;     // fix rounding errors as later code tests for equality with 1.
        }
        else if (in.b == maximum) {
            in = in + to_add;
            in.b = 1.f;
        }
        else if (in.c == maximum) {
            in = in + to_add;
            in.c = 1.f;
        }
        else if (in.d == maximum) {
            in = in + to_add;
            in.d = 1.f;
        } else {
            // unreachable ????
        }
    }

    return in;
    // Vec4f{a, b, c, d};
}

Vec4f fourphase_calibration_to_amplitude(Vec4f calibration_vector_in_db) {
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

class Interpolator {
public:
    Interpolator(Vec4f max_amplitude) {
        this->max_amplitude = max_amplitude;
        float p;

        // pre-calculate the currents for the fixed points A, B, C, D
        // TODO: check p <= 1?
        p = max_amplitude.a / (max_amplitude.b + max_amplitude.c + max_amplitude.d);
        A = Vec4f{max_amplitude.a * 1, max_amplitude.b * p, max_amplitude.c * p, max_amplitude.d * p};
        p = max_amplitude.b / (max_amplitude.a + max_amplitude.c + max_amplitude.d);
        B = Vec4f{max_amplitude.a * p, max_amplitude.b * 1, max_amplitude.c * p, max_amplitude.d * p};
        p = max_amplitude.c / (max_amplitude.a + max_amplitude.b + max_amplitude.d);
        C = Vec4f{max_amplitude.a * p, max_amplitude.b * p, max_amplitude.c * 1, max_amplitude.d * p};
        p = max_amplitude.d / (max_amplitude.a + max_amplitude.b + max_amplitude.c);
        D = Vec4f{max_amplitude.a * p, max_amplitude.b * p, max_amplitude.c * p, max_amplitude.d * 1};

        ABC = minimize_d({max_amplitude.a, max_amplitude.b, max_amplitude.c, 0});
        ABD = minimize_c({max_amplitude.a, max_amplitude.b, 0, max_amplitude.d});
        ACD = minimize_b({max_amplitude.a, 0, max_amplitude.c, max_amplitude.d});
        BCD = minimize_a({0, max_amplitude.b, max_amplitude.c, max_amplitude.d});
    }

    /**
     * For a given input position, calculate the desired (calibrated) output current.
     * p must satisfy the constraints from fourphase_constrain_coordinates.
     */
    Vec4f interpolate(Vec4f p) {
        // This could have been implemented with less code by line-in-triangle testing
        // all triangles?
        if (p.a == 1) {
            if (p.b == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AC_AD_normal) <= 0) {
                    return interpolate_A_AC_AD(p);
                } else {
                    return interpolate_ACD_AC_AD(p);
                }
            }
            else if (p.c == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AB_AD_normal) <= 0) {
                    return interpolate_A_AB_AD(p);
                } else {
                    return interpolate_ABD_AB_AD(p);
                }
            }
            else if (p.d == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AB_AC_normal) <= 0) {
                    return interpolate_A_AB_AC(p);
                } else {
                    return interpolate_ABC_AB_AC(p);
                }
            }
        } else if (p.b == 1) {
            if (p.a == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_BC_BD_normal) <= 0) {
                    return interpolate_B_BC_BD(p);
                } else {
                    return interpolate_BCD_BC_BD(p);
                }
            }
            else if (p.c == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AB_BD_normal) <= 0) {
                    return interpolate_B_AB_BD(p);
                } else {
                    return interpolate_ABD_AB_BD(p);
                }
            }
            else if (p.d == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AB_BC_normal) <= 0) {
                    return interpolate_B_AB_BC(p);
                } else {
                    return interpolate_ABC_AB_BC(p);
                }
            }
        } else if (p.c == 1) {
            if (p.a == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_BC_CD_normal) <= 0) {
                    return interpolate_C_BC_CD(p);
                } else {
                    return interpolate_BCD_BC_CD(p);
                }
            }
            else if (p.b == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AC_CD_normal) <= 0) {
                    return interpolate_C_AC_CD(p);
                } else {
                    return interpolate_ACD_AC_CD(p);
                }
            }
            else if (p.d == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AC_BC_normal) <= 0) {
                    return interpolate_C_AC_BC(p);
                } else {
                    return interpolate_ABC_AC_BC(p);
                }
            }
        } else if (p.d == 1) {
            if (p.a == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_BD_CD_normal) <= 0) {
                    return interpolate_D_BD_CD(p);
                } else {
                    return interpolate_BCD_BD_CD(p);
                }
            }
            else if (p.b == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AD_CD_normal) <= 0) {
                    return interpolate_D_AD_CD(p);
                } else {
                    return interpolate_ACD_AD_CD(p);
                }
            }
            else if (p.c == p.min()) {
                if (dot(p - PointCoords::O, PointCoords::O_AD_BD_normal) <= 0) {
                    return interpolate_D_AD_BD(p);
                } else {
                    return interpolate_ABD_AD_BD(p);
                }
            }
        }

        // unreachable
        // todo: error?
        return {0, 0, 0, 0};
    }

private:
    Vec4f interpolate_A_B(float s) {
        float a = A.a + (B.a - A.a) * powf(s, 2);
        float b = B.b + (A.b - B.b) * powf(1 - s, 2);
        float remainder = std::abs(a - b);
        float c = remainder * max_amplitude.c / (max_amplitude.c + max_amplitude.d);
        float d = remainder * max_amplitude.d / (max_amplitude.c + max_amplitude.d);
        return {a, b, c, d};
    }

    Vec4f interpolate_A_C(float s) {
        float a = A.a + (C.a - A.a) * powf(s, 2);
        float c = C.c + (A.c - C.c) * powf(1 - s, 2);
        float remainder = std::abs(a - c);
        float b = remainder * max_amplitude.b / (max_amplitude.b + max_amplitude.d);
        float d = remainder * max_amplitude.d / (max_amplitude.b + max_amplitude.d);
        return {a, b, c, d};
    }

    Vec4f interpolate_A_D(float s) {
        float a = A.a + (D.a - A.a) * powf(s, 2);
        float d = D.d + (A.d - D.d) * powf(1 - s, 2);
        float remainder = std::abs(a - d);
        float b = remainder * max_amplitude.b / (max_amplitude.b + max_amplitude.c);
        float c = remainder * max_amplitude.b / (max_amplitude.b + max_amplitude.c);
        return {a, b, c, d};
    }

    Vec4f interpolate_B_C(float s) {
        float b = B.b + (C.b - B.b) * powf(s, 2);
        float c = C.c + (B.c - C.c) * powf(1 - s, 2);
        float remainder = std::abs(b - c);
        float a = remainder * max_amplitude.a / (max_amplitude.a + max_amplitude.d);
        float d = remainder * max_amplitude.d / (max_amplitude.a + max_amplitude.d);
        return {a, b, c, d};
    }

    Vec4f interpolate_B_D(float s) {
        float b = B.b + (D.b - B.b) * powf(s, 2);
        float d = D.d + (B.d - D.d) * powf(1 - s, 2);
        float remainder = std::abs(b - d);
        float a = remainder * max_amplitude.a / (max_amplitude.a + max_amplitude.c);
        float c = remainder * max_amplitude.c / (max_amplitude.a + max_amplitude.c);
        return {a, b, c, d};
    }

    Vec4f interpolate_C_D(float s) {
        float c = C.c + (D.c - C.c) * powf(s, 2);
        float d = D.d + (C.d - D.d) * powf(1 - s, 2);
        float remainder = std::abs(c - d);
        float a = remainder * max_amplitude.a / (max_amplitude.a + max_amplitude.b);
        float b = remainder * max_amplitude.b / (max_amplitude.a + max_amplitude.b);
        return {a, b, c, d};
    }

    Vec3f project_line_through_origin_on_plane(Vec3f p, Vec3f c1, Vec3f c2, Vec3f c3) {
        // https://stackoverflow.com/questions/5666222/3d-line-plane-intersection/18543221#18543221
        Vec3f origin{1, 1, 1};
        Vec3f plane_normal = cross(c2 - c1, c3 - c1);
        plane_normal = plane_normal / plane_normal.norm();

        // TODO: check behavior for p == 0
        Vec3f u = p - origin;
        float d = dot(plane_normal, u);
        if (abs(d) >= 1e-6f) {
            Vec3f w = origin - c1;
            float fac = -dot(plane_normal, w) / d;
            return origin + u * fac;
        } else {
            return c1;
        }
    }

    Vec4f project_line_through_origin_on_plane(Vec4f p, Vec4f c1, Vec4f c2, Vec4f c3, int fixed_index) {
        switch (fixed_index) {
            case 0: // a == 1 for all points
            {
                Vec3f projection = project_line_through_origin_on_plane(
                    p.bcd(), c1.bcd(), c2.bcd(), c3.bcd()
                );
                return {1, projection.a, projection.b, projection.c};
            }
            break;
            case 1: // b == 1 for all points
            {
                Vec3f projection = project_line_through_origin_on_plane(
                    p.acd(), c1.acd(), c2.acd(), c3.acd()
                );
                return {projection.a, 1, projection.b, projection.c};
            }
            break;
            case 2: // c == 1 for all points
            {
                Vec3f projection = project_line_through_origin_on_plane(
                    p.abd(), c1.abd(), c2.abd(), c3.abd()
                );
                return {projection.a, projection.b, 1, projection.c};
            }
            break;
            case 3: // d == 1 for all points
            {
                Vec3f projection = project_line_through_origin_on_plane(
                    p.abc(), c1.abc(), c2.abc(), c3.abc()
                );
                return {projection.a, projection.b, projection.c, 1};
            }
            break;
        }
        // unreachable
    }

    float distance_from_origin(Vec4f p, Vec4f projection) {
        return (p - PointCoords::O).norm() / (projection - PointCoords::O).norm();
    }

    /**
     * transform p into 2D coordinate system inside a triangle.
     * out.a goes from 0 to 1 as p goes from c1 to c2/3
     * out.b goes from 0 to 1 as p goes from c2 to c3
     */
    Vec2f uniform_coordinates(Vec4f p, Vec4f c1, Vec4f c2, Vec4f c3) {
        // depends on (c1, c2) and (c1, c3) being equal length.
        Vec4f mid = (c2 + c3) * 0.5f;
        Vec4f u_dir = mid - c1;
        float u = dot((p - c1), u_dir) / u_dir.norm_sq();
        u = std::clamp<float>(u, 0, 1); // TODO: maybe error check?

        Vec4f v_dir = c3 - mid;
        float v = dot(p - mid, v_dir) / v_dir.norm_sq();
        if (u > 1e-6f) {
            v = v / u;
            v = v / 2 + 0.5f;
        } else {
            v = 0.5f;
        }
        v = std::clamp<float>(v, 0, 1); // TODO: maybe error check?

        return {u, v};
    }

    Vec4f minimize_a(Vec4f p) {
        p.a = 0;
        p.a = std::max(p.max() * 2 - p.sum(), 0.0f);
        return p;
    }

    Vec4f minimize_b(Vec4f p) {
        p.b = 0;
        p.b = std::max(p.max() * 2 - p.sum(), 0.0f);
        return p;
    }

    Vec4f minimize_c(Vec4f p) {
        p.c = 0;
        p.c = std::max(p.max() * 2 - p.sum(), 0.0f);
        return p;
    }

    Vec4f minimize_d(Vec4f p) {
        p.d = 0;
        p.d = std::max(p.max() * 2 - p.sum(), 0.0f);
        return p;
    }

    Vec4f interpolate_A_AB_AC(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::A, PointCoords::AB, PointCoords::AC, 0);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::A, PointCoords::AB, PointCoords::AC);

        Vec4f left = interpolate_A_B(uv.a * 0.5f);
        Vec4f right = interpolate_A_C(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_d(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_A_AB_AD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::A, PointCoords::AB, PointCoords::AD, 0);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::A, PointCoords::AB, PointCoords::AD);

        Vec4f left = interpolate_A_B(uv.a * 0.5f);
        Vec4f right = interpolate_A_D(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_c(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_A_AC_AD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::A, PointCoords::AC, PointCoords::AD, 0);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::A, PointCoords::AC, PointCoords::AD);

        Vec4f left = interpolate_A_C(uv.a * 0.5f);
        Vec4f right = interpolate_A_D(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_b(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_B_AB_BC(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::B, PointCoords::AB, PointCoords::BC, 1);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::B, PointCoords::AB, PointCoords::BC);

        Vec4f left = interpolate_A_B(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_B_C(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_d(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_B_AB_BD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::B, PointCoords::AB, PointCoords::BD, 1);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::B, PointCoords::AB, PointCoords::BD);

        Vec4f left = interpolate_A_B(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_B_D(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_c(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_B_BC_BD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::B, PointCoords::BC, PointCoords::BD, 1);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::B, PointCoords::BC, PointCoords::BD);

        Vec4f left = interpolate_B_C(uv.a * 0.5f);
        Vec4f right = interpolate_B_D(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_a(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_C_AC_BC(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::C, PointCoords::AC, PointCoords::BC, 2);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::C, PointCoords::AC, PointCoords::BC);

        Vec4f left = interpolate_A_C(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_B_C(1.0f - uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_d(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_C_AC_CD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::C, PointCoords::AC, PointCoords::CD, 2);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::C, PointCoords::AC, PointCoords::CD);

        Vec4f left = interpolate_A_C(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_C_D(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_b(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_C_BC_CD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::C, PointCoords::BC, PointCoords::CD, 2);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::C, PointCoords::BC, PointCoords::CD);

        Vec4f left = interpolate_B_C(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_C_D(uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_a(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_D_AD_BD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::D, PointCoords::AD, PointCoords::BD, 3);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::D, PointCoords::AD, PointCoords::BD);

        Vec4f left = interpolate_A_D(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_B_D(1.0f - uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_c(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_D_AD_CD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::D, PointCoords::AD, PointCoords::CD, 3);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::D, PointCoords::AD, PointCoords::CD);

        Vec4f left = interpolate_A_D(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_C_D(1.0f - uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_b(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_D_BD_CD(Vec4f p) {
        Vec4f proj = project_line_through_origin_on_plane(p, PointCoords::D, PointCoords::BD, PointCoords::CD, 3);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, PointCoords::D, PointCoords::BD, PointCoords::CD);

        Vec4f left = interpolate_B_D(1.0f - uv.a * 0.5f);
        Vec4f right = interpolate_C_D(1.0f - uv.a * 0.5f);
        Vec4f edge = left * (1 - uv.b) + right * uv.b;
        edge = minimize_a(edge);
        return interpolate_center(edge, from_center);
    }

    Vec4f linear_interpolate(
        Vec4f p,
        Vec4f c1, Vec4f c2, Vec4f c3,
        Vec4f v1, Vec4f v2, Vec4f v3,
        int edge_index, int minimize_index
    ) {
        // TODO: could try barycentric

        Vec4f proj = project_line_through_origin_on_plane(p, c1, c2, c3, edge_index);
        float from_center = distance_from_origin(p, proj);
        Vec2f uv = uniform_coordinates(proj, c1, c2, c3);

        Vec4f edge = v2 * (1 - uv.b) + v3 * uv.b;
        edge = v1 * (1 - uv.a) + edge * (uv.a);
        switch (minimize_index) {
            case 0: edge = minimize_a(edge); break;
            case 1: edge = minimize_b(edge); break;
            case 2: edge = minimize_c(edge); break;
            case 3: edge = minimize_d(edge); break;
        };
        return interpolate_center(edge, from_center);
    }

    Vec4f interpolate_ABC_AB_AC(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ABC, PointCoords::AB, PointCoords::AC,
            ABC, interpolate_A_B(.5f), interpolate_A_C(.5f),
            0, 3
        );
    }
    Vec4f interpolate_ABC_AB_BC(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ABC, PointCoords::AB, PointCoords::BC,
            ABC, interpolate_A_B(.5f), interpolate_B_C(.5f),
            1, 3
        );
    }
    Vec4f interpolate_ABC_AC_BC(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ABC, PointCoords::AC, PointCoords::BC,
            ABC, interpolate_A_C(.5f), interpolate_B_C(.5f),
            2, 3
        );
    }

    Vec4f interpolate_ABD_AB_AD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ABD, PointCoords::AB, PointCoords::AD,
            ABD, interpolate_A_B(.5f), interpolate_A_D(.5f),
            0, 2
        );
    }
    Vec4f interpolate_ABD_AB_BD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ABD, PointCoords::AB, PointCoords::BD,
            ABD, interpolate_A_B(.5f), interpolate_B_D(.5f),
            1, 2
        );

    }
    Vec4f interpolate_ABD_AD_BD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ABD, PointCoords::AD, PointCoords::BD,
            ABD, interpolate_A_D(.5f), interpolate_B_D(.5f),
            3, 2
        );
    }

    Vec4f interpolate_ACD_AC_AD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ACD, PointCoords::AC, PointCoords::AD,
            ACD, interpolate_A_C(.5f), interpolate_A_D(.5f),
            0, 1
        );
    }
    Vec4f interpolate_ACD_AC_CD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ACD, PointCoords::AC, PointCoords::CD,
            ACD, interpolate_A_C(.5f), interpolate_C_D(.5f),
            2, 1
        );
    }
    Vec4f interpolate_ACD_AD_CD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::ACD, PointCoords::AD, PointCoords::CD,
            ACD, interpolate_A_D(.5f), interpolate_C_D(.5f),
            3, 1
        );
    }

    Vec4f interpolate_BCD_BC_BD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::BCD, PointCoords::BC, PointCoords::BD,
            BCD, interpolate_B_C(.5f), interpolate_B_D(.5f),
            1, 0
        );
    }
    Vec4f interpolate_BCD_BC_CD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::BCD, PointCoords::BC, PointCoords::CD,
            BCD, interpolate_B_C(.5f), interpolate_C_D(.5f),
            2, 0
        );
    }
    Vec4f interpolate_BCD_BD_CD(Vec4f p) {
        return linear_interpolate(p,
            PointCoords::BCD, PointCoords::BD, PointCoords::CD,
            BCD, interpolate_B_D(.5f), interpolate_C_D(.5f),
            3, 0
        );
    }

    Vec4f interpolate_center(Vec4f edge, float distance_from_center) {
        // TODO: not linear?
        // TODO: not sure if this .86 does anything.
        return edge * distance_from_center + max_amplitude * .86 * (1 - distance_from_center);
    }

    Vec4f max_amplitude;
    Vec4f A;
    Vec4f B;
    Vec4f C;
    Vec4f D;
    Vec4f ABC;
    Vec4f ABD;
    Vec4f ACD;
    Vec4f BCD;
};

ComplexFourphasePoints project_fourphase_2(
    float pulse_amplitude,
    Vec4f position_vector,
    Vec4f calibration_vector_in_db,
    bool flip_polarity,
    float start_angle)
{
    position_vector = fourphase_constrain_coordinates(position_vector);
    Vec4f max_amplitude = fourphase_calibration_to_amplitude(calibration_vector_in_db);

    Vec4f current_to_power{
        1 / max_amplitude.a,
        1 / max_amplitude.b,
        1 / max_amplitude.c,
        1 / max_amplitude.d,
    };

    // transform position to electrode current
    // accounting for calibration
    Interpolator interpolator(max_amplitude);
    Vec4f output_current = interpolator.interpolate(position_vector);

    // normalize intensity to 1
    float intensity = fourphase_intensity(output_current * current_to_power);
    output_current = output_current / intensity;

    // project points on complex plane
    ComplexFourphasePoints complex_points = fourphase_electrode_amplitude_to_complex_points(output_current);
    return fourphase_permute_complex_points(complex_points, flip_polarity, start_angle, pulse_amplitude);
}

Vec4f fourphase_interpolate(Vec4f p, Vec4f max_amplitude)
{
    Interpolator interpolator(max_amplitude);
    p = fourphase_constrain_coordinates(p);
    return interpolator.interpolate(p);
}

float fourphase_intensity(Vec4f electrode_power_in_percent)
{
    // empirically, these feel about the same intensity:
    // 1,   .33, .33, .33
    // .86, .86, .86, .86
    // .86, .86, 0,   0

    // find largest and second-largest component.
    float m1 = electrode_power_in_percent.max();
    float m2;
    if (m1 == electrode_power_in_percent.a) {
        m2 = electrode_power_in_percent.bcd().max();
    }
    else if (m1 == electrode_power_in_percent.b) {
        m2 = electrode_power_in_percent.acd().max();
    }
    else if (m1 == electrode_power_in_percent.c) {
        m2 = electrode_power_in_percent.abd().max();
    }
    else {
        m2 = electrode_power_in_percent.abc().max();
    }
    m2 = std::max(1/3.f, m2);

    // z is chosen such that (1, .333) and (.86, .86) results in the same value.
    // which empirically works well.
    float z = 4.5f;
    float left = 0;
    float right = 10;
    float x;
    // solve magic formula 1 = (m1 * x)**z + (m2 * x)**z
    for (int i = 0; i < 100; i++) {
        x = (left + right) / 2;
        float intensity = powf(m1 * x, z) + powf(m2 * x, z);
        if (intensity > 1) {
            right = x;
        } else {
            left = x;
        }
    }

    // normalize such that fourphase_intensity(1, 1/3, 1/3, 1/3) is 1
    x *= float(1 / 0.9984229);

    return 1 / x;
}
