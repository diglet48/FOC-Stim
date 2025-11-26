#include "complex.h"

#include "foc_utils.h"
#include <algorithm>

void Complex::constrain_in_bound(float min_magnitude, float max_magnitude, float min_angle, float max_angle)
{
    float magnitude = norm();
    float angle = atan2f(b, a);

    float desired_magnitude = std::clamp<float>(magnitude, min_magnitude, max_magnitude);
    float desired_angle = std::clamp<float>(angle, min_angle, max_angle);

    if (magnitude == desired_magnitude && angle == desired_angle) {
        return;
    }

    a = cosf(desired_angle) * desired_magnitude;
    b = sinf(desired_angle) * desired_magnitude;
}
