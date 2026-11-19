#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace finemote_urdf {
class CommandLease {
public:
    void Receive(double forward, double lateral, double yaw, bool planar, uint32_t now) {
        if (!planar || !(forward * forward + lateral * lateral <= 1.0 && std::abs(yaw) <= 2.0)) {
            Clear();
            return;
        }
        if (forward == 0 && lateral == 0 && yaw == 0) valid_ = true;
        if (valid_) {
            velocity_ = {static_cast<float>(forward), static_cast<float>(lateral), static_cast<float>(yaw)};
            received_ = now;
        }
    }

    void Clear() { valid_ = false; }

    bool Read(uint32_t now, uint32_t timeout, std::array<float, 3>& velocity) {
        if (now - received_ >= timeout) Clear();
        velocity = valid_ ? velocity_ : std::array<float, 3>{};
        return valid_ && velocity != std::array<float, 3>{};
    }

private:
    std::array<float, 3> velocity_{};
    uint32_t received_ = 0;
    bool valid_ = false;
};
}
