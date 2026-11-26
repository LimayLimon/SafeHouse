#pragma once
#include <algorithm>
#include <cmath>

namespace slai_activation {
constexpr float enter_radius = 1500.f;
constexpr float leave_radius = 1800.f;
constexpr float refresh_seconds = .5f;
constexpr float refresh_motion = 200.f;
inline bool Within(float distance, float native_radius, bool active) {
    if (!std::isfinite(distance) || distance < 0 || !(native_radius > 0)) return false;
    return distance < std::min(native_radius, active ? leave_radius : enter_radius);
}
inline bool Refresh(bool was, bool now, bool new_world, float elapsed, float motion) {
    return was != now || (now && (new_world || elapsed < 0 || elapsed >= refresh_seconds || motion >= refresh_motion));
}
}
