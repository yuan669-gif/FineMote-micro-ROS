#pragma once
#include <cmath>
#include <sensor_msgs/msg/detail/joint_state__struct.h>

namespace finemote_urdf {
class JointReport {
public:
    template<size_t Length>
    JointReport(char (&name)[Length], bool position, bool velocity)
        : name_{name, Length - 1, Length}, position_(position), velocity_(velocity) {}

    void Fill(sensor_msgs__msg__JointState& message, double degrees, double dps,
              double scale, bool received, int64_t stamp) {
        angle_ = received ? degrees * scale : NAN;
        speed_ = received ? dps * scale : NAN;
        static char empty[] = "";
        message.header.frame_id = {empty, 0, 1};
        message.header.stamp.sec = stamp / 1000000000;
        message.header.stamp.nanosec = stamp % 1000000000;
        message.name = {&name_, 1, 1};
        if (position_) message.position = {&angle_, 1, 1};
        if (velocity_) message.velocity = {&speed_, 1, 1};
    }
private:
    rosidl_runtime_c__String name_;
    bool position_, velocity_;
    double angle_ = 0, speed_ = 0;
};
}
