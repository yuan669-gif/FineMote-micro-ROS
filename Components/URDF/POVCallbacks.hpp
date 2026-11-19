#pragma once
#include <array>
#include <cstdint>
#include <utility>
#include "RemoteControllers/RadioMaster_Zorro.h"
#include "MicroROS/MicroROS_MessageTypes.hpp"
#include "main.h"

// Reusable application behaviours. Robot objects and their wiring live in URDF.
namespace finemote_urdf {
template<class Remote>
auto decode(Remote& remote) {
    return [&remote](uint8_t* data, size_t length) { remote.Decode(data, length); };
}
inline auto heartbeat(int32_t& count) {
    return [&count](std_msgs__msg__Int32& msg) { msg.data = count++; };
}
inline auto led() {
    return [](const std_msgs__msg__Bool& msg) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, msg.data ? GPIO_PIN_RESET : GPIO_PIN_SET);
    };
}
template<class Chassis, class Remote>
class ZorroVelocityTask {
public:
    ZorroVelocityTask(Chassis& chassis, Remote& remote, float linear_scale, float angular_scale)
        : chassis(chassis), remote(remote), linear_scale(linear_scale), angular_scale(angular_scale) {}
    void Run() {
        if (remote.GetInfo().sC == RemoteControl::SWITCH_STATE_E::UP_POS) {
            std::array<float, 3> target{
                remote.GetInfo().rightCol * linear_scale,
                -remote.GetInfo().rightRol * linear_scale,
                -remote.GetInfo().leftRol * angular_scale};
            chassis.SetVelocity(std::move(target));
        }
    }
private:
    Chassis& chassis;
    Remote& remote;
    const float linear_scale;
    const float angular_scale;
};
} // namespace finemote_urdf
