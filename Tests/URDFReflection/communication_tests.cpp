#include "URDF/CommandLease.hpp"
#include "URDF/JointReport.hpp"
#include <cassert>
#include <cstring>
#include <limits>

int main() {
    finemote_urdf::CommandLease lease;
    std::array<float, 3> velocity{};
    constexpr uint32_t start = UINT32_MAX - 10;
    lease.Receive(0.5, 0, 1, true, start);
    assert(!lease.Read(start, 600, velocity));
    lease.Receive(0, 0, 0, true, start);
    lease.Receive(0.5, -0.25, 1, true, start);
    assert(lease.Read(start + 599, 600, velocity));
    assert(velocity[0] == 0.5f && velocity[1] == -0.25f && velocity[2] == 1);
    assert(!lease.Read(start + 600, 600, velocity));
    assert((velocity == std::array<float, 3>{}));
    for (unsigned fault = 0; fault < 5; ++fault) {
        lease.Receive(0, 0, 0, true, 1000);
        lease.Receive(1, 0, 0, true, 1001);
        assert(lease.Read(1001, 600, velocity));
        if (fault == 0) lease.Clear();
        if (fault == 1) lease.Receive(NAN, 0, 0, true, 1002);
        if (fault == 2) lease.Receive(0, 0, INFINITY, true, 1002);
        if (fault == 3) lease.Receive(0, 0, 0, false, 1002);
        if (fault == 4) lease.Receive(1, 1, 0, true, 1002);
        lease.Receive(1, 0, 0, true, 1003);
        assert(!lease.Read(1003, 600, velocity));
    }
    char name[] = "wheel_joint_FR";
    finemote_urdf::JointReport report(name, true, false);
    sensor_msgs__msg__JointState msg{};
    report.Fill(msg, 180, 45, 0.01745329252, true, 2000000003);
    assert(msg.name.size == 1 && !std::strcmp(msg.name.data->data, name));
    assert(msg.name.data->capacity == sizeof(name));
    assert(msg.position.size == 1 && std::abs(msg.position.data[0] - 3.141593) < 0.000001);
    assert(msg.velocity.size == 0 && msg.effort.size == 0);
    assert(msg.header.stamp.sec == 2 && msg.header.stamp.nanosec == 3);
    auto storage = msg.position.data;
    report.Fill(msg, 0, 0, 1, false, 0);
    assert(storage == msg.position.data && std::isnan(*storage));
}
