#pragma once
#include <array>
#include <cmath>

namespace finemote {
struct SwerveCommand { float angle_degrees, speed_dps; };
// Keep the existing POV 100-degree reversal threshold and degree/DPS convention.
// Feedback remains a callback so the driver retains its snapshot-read semantics.
template<class Feedback>
SwerveCommand swerve_command(const std::array<float,3>& velocity, float x, float y,
                            float zero, float diameter, Feedback feedback) {
    constexpr float pi = 3.14159265358979323846f;
    const float vx = velocity[0] - y * velocity[2];
    const float vy = velocity[1] + x * velocity[2];
    float speed = std::sqrt(vx * vx + vy * vy);
    float angle = std::atan2(vy, vx) * 180 / pi;
    bool negative = false;
    while (angle + zero - feedback() > 100) { angle -= 180; negative = !negative; }
    while (angle + zero - feedback() < -100) { angle += 180; negative = !negative; }
    if (negative) speed = -speed;
    return {angle + zero, speed / diameter / pi * 360};
}
} // namespace finemote
