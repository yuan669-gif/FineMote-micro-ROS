#pragma once
#include "URDF/CommandLease.hpp"
#include "URDF/JointReport.hpp"
#include "URDF/POVDevices.hpp"
#include "MicroROS/MicroROS_Manager.hpp"
#include <rmw_microxrcedds_c/config.h>

namespace finemote_urdf {
template<typename Chassis>
auto PlanarState(Chassis& chassis) {
    return [&chassis](geometry_msgs__msg__Twist& message) {
        chassis.GetRosBinder()(message);
        if (!chassis.HasModuleFeedback())
            message.linear.x = message.linear.y = message.angular.z = NAN;
    };
}

class JointPublisher {
public:
    template<size_t Length>
    JointPublisher(const char* topic, char (&name)[Length], MotorBase& motor, bool position, bool velocity)
        : report_(name, position, velocity), motor_(motor),
          publisher_(topic, [this](sensor_msgs__msg__JointState& message) { Fill(message); }) {}

private:
    void Fill(sensor_msgs__msg__JointState& message) {
        taskENTER_CRITICAL();
        const auto state = motor_.GetState();
        const bool received = motor_.HasFeedback();
        taskEXIT_CRITICAL();
        const int64_t stamp = rmw_uros_epoch_synchronized() ? rmw_uros_epoch_nanos() : 0;
        report_.Fill(message, state.position, state.speed, motor_.JointScale(), received, stamp);
    }
    JointReport report_;
    MotorBase& motor_;
    RosPublisher<sensor_msgs__msg__JointState> publisher_;
};

class PlanarSubscriber {
public:
    template<typename Chassis>
    PlanarSubscriber(const char* topic, Chassis& chassis)
        : subscriber_(topic, [this](const geometry_msgs__msg__Twist& message) {
            taskENTER_CRITICAL();
            lease_.Receive(message.linear.x, message.linear.y, message.angular.z,
                           message.linear.z == 0 && message.angular.x == 0 && message.angular.y == 0,
                           xTaskGetTickCount());
            taskEXIT_CRITICAL();
        }, [this]() {
            taskENTER_CRITICAL();
            lease_.Clear();
            taskEXIT_CRITICAL();
        }) {
        chassis.BindCommandSource([this](std::array<float, 3>& velocity) {
            taskENTER_CRITICAL();
            bool valid = lease_.Read(xTaskGetTickCount(), pdMS_TO_TICKS(600), velocity);
            taskEXIT_CRITICAL();
            return valid;
        });
    }
private:
    CommandLease lease_;
    RosSubscriber<geometry_msgs__msg__Twist> subscriber_;
};
}
