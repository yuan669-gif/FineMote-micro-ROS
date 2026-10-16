#include "URDF/format.hpp"
#include "URDF/finemote.hpp"
#include "pov_chassis_urdf.hpp"
#include "fixture_urdf.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {
constexpr auto limits = [] {
    fineurdf::FinemoteLimits l{};
    l.model.links = 16;
    l.model.joints = 16;
    l.model.transmissions = 16;
    l.model.hardware = 2;
    l.model.hw_joints = 16;
    l.model.syntax = {512, 1024};
    l.motors = 8;
    l.modules = 4;
    return l;
}();

constexpr auto golden = fineurdf::build_config<limits>(pov_chassis_LITERAL);
static_assert(golden.ok && golden.motor_count == 8 && golden.module_count == 4);
static_assert(golden.joint_order.ok && golden.joint_order.count == 8);
static_assert(fineurdf::build_config<limits>(fixture_LITERAL).ok);

int failures = 0;
int checks = 0;
void check(bool condition, const char* label) {
    ++checks;
    if (!condition) { std::cerr << "FAIL: " << label << '\n'; ++failures; }
}

std::string replace_one(std::string text, const std::string& from, const std::string& to) {
    const auto pos = text.find(from);
    if (pos == std::string::npos) { check(false, "test replacement exists"); return text; }
    text.replace(pos, from.size(), to);
    return text;
}

std::string replace_hardware(std::string text, const std::string& from, const std::string& to) {
    const auto start = text.find("<ros2_control");
    const auto pos = text.find(from, start);
    if (pos == std::string::npos) { check(false, "hardware test replacement exists"); return text; }
    text.replace(pos, from.size(), to);
    return text;
}

void rejected(const std::string& text, const char* label) {
    const auto config = fineurdf::build_config<limits>(text);
    check(!config.ok && !config.err_text.empty(), label);
}
} // namespace

int main() {
    const std::string source{pov_chassis_LITERAL};
    check(std::abs(golden.wheel_diameter - 0.0483f) < 1e-6f, "wheel diameter");
    check(golden.modules[0].zero_position == 180.f && golden.modules[1].zero_position == 0.f, "explicit steering calibration");
    rejected(replace_one(source, "<param name=\"can_id\">0x02", "<param name=\"can_id\">1"), "numeric CAN collision");
    rejected(replace_one(source, "<param name=\"can_id\">0x02", "<param name=\"can_id\">garbage"), "invalid CAN ID");
    rejected(replace_one(source, "<param name=\"division_factor\">2", "<param name=\"division_factor\">0"), "zero division");
    rejected(replace_one(source, "<param name=\"division_factor\">2", "<param name=\"division_factor\">65536"), "division overflow");
    rejected(replace_one(source, "<param name=\"division_factor\">2", "<param name=\"division_factor\">256"), "MotorBase division range");
    rejected(replace_one(source, "<param name=\"can_id\">0x02", "<param name=\"can_id\">18446744073709551616"), "integer overflow");
    rejected(replace_one(source, "<param name=\"control_type\">position", "<param name=\"control_type\">invalid"), "invalid control type");
    rejected(replace_one(source, "xyz=\"0.12 -0.12 0\"", "xyz=\"bad -0.12 0\""), "invalid geometry number");
    rejected(replace_one(source, "xyz=\"0.12 -0.12 0\"", "xyz=\"1e99999999999999999999 0 0\""), "exponent overflow");
    rejected(replace_one(source, "xyz=\"0.12 -0.12 0\"", "xyz=\"1e 0 0\""), "missing exponent digits");
    rejected(replace_one(source, "<param name=\"pid\">1.2 0.0 0.05", "<param name=\"pid\">1.2 0.0"), "PID arity");
    rejected(replace_one(source, "<param name=\"wheel_radius\">0.02415", "<param name=\"wheel_radius\">-0.1"), "negative radius");
    rejected(replace_one(source, "<param name=\"zero_position\">180</param>", ""), "missing steering calibration");
    rejected(replace_one(source, "rpy=\"0 0 0\"", "rpy=\"0 0 1\""), "unsupported swerve rotation");
    rejected(replace_one(source, "type=\"continuous\"", "type=\"typo\""), "unknown joint type");
    rejected(replace_one(source, "<parent link=\"base_link\"/>", "<parent link=\"missing\"/>"), "dangling link");
    rejected(replace_one(source, "<parent link=\"base_link\"/>", "<parent link=\"steer_link_FR\"/>"), "self cycle");
    rejected(replace_one(source, "<link name=\"steer_link_FR\"/>", "<link name=\"base_link\"/>"), "duplicate links");
    rejected(replace_hardware(source, "<joint name=\"steer_joint_FR\">", "<joint name=\"missing\">"), "unknown hardware joint");
    rejected(replace_hardware(source, "<joint name=\"steer_joint_FL\">", "<joint name=\"steer_joint_FR\">"), "duplicate hardware claim");
    rejected(replace_one(source, "<link name=\"steer_link_FR\"/>", "<link name=\"" + std::string(65, 'x') + "\"/>"), "name capacity");
    rejected(source + "<robot name=\"extra\"/>", "multiple XML roots");
    rejected(replace_one(source, "<robot name=", "<robot invalid name="), "valueless XML attribute");

    constexpr auto small_motors = [] { auto l = limits; l.motors = 7; return l; }();
    check(!fineurdf::build_config<small_motors>(source).ok, "motor capacity");
    constexpr auto small_links = [] { auto l = limits; l.model.links = 8; return l; }();
    check(!fineurdf::build_config<small_links>(source).ok, "link capacity");
    constexpr auto small_params = [] { auto l = limits; l.model.hw_params = 2; return l; }();
    check(!fineurdf::build_config<small_params>(source).ok, "parameter capacity");
    constexpr auto small_modules = [] { auto l = limits; l.modules = 3; return l; }();
    check(!fineurdf::build_config<small_modules>(source).ok, "module capacity");
    const auto asymmetric = fineurdf::build_config<limits>(replace_one(source, "xyz=\"0.12 -0.12 0\"", "xyz=\"0.3 -0.12 0\""));
    check(asymmetric.ok && std::abs(asymmetric.wheelbase - 0.42f) < 1e-6f &&
          std::abs(asymmetric.track_width - 0.24f) < 1e-6f, "X wheelbase and Y track width");
    const auto offset = fineurdf::build_config<limits>(replace_one(source, "xyz=\"0.02415 0 0.02415\"", "xyz=\"0.04 0 -0.1\""));
    check(offset.ok && offset.modules[0].wheel_radius == golden.modules[0].wheel_radius &&
          offset.modules[0].zero_position == golden.modules[0].zero_position, "calibration independent of wheel offset");
    const std::string split = replace_hardware(source, "    <joint name=\"steer_joint_FL\">",
        "</ros2_control><ros2_control name=\"second\" type=\"system\"><hardware>"
        "<param name=\"can_bus\">can1</param></hardware><joint name=\"steer_joint_FL\">");
    check(fineurdf::build_config<limits>(split).ok, "multiple hardware blocks");
    rejected(replace_one(split, "<param name=\"can_id\">0x02", "<param name=\"can_id\">1"), "CAN collision across hardware blocks");
    const std::string other_bus = replace_one(split, "name=\"second\" type=\"system\"><hardware><param name=\"can_bus\">can1",
        "name=\"second\" type=\"system\"><hardware><param name=\"can_bus\">can2");
    check(fineurdf::build_config<limits>(replace_one(other_bus, "<param name=\"can_id\">0x02", "<param name=\"can_id\">1")).ok,
          "same CAN identifier on different buses");
    check(!fineurdf::parse_xml<8, 8>("<robot>").ok(), "direct parser unclosed root");
    check(!fineurdf::parse_auto("<robot/><?unfinished").ok(), "unterminated processing instruction");
    check(fineurdf::parse_auto("\xef\xbb\xbf<robot/>").ok(), "UTF-8 BOM");
    check(!fineurdf::parse_auto("<robot a='1'a='2'/>").ok(), "missing attribute separator");
    check(!fineurdf::parse_auto("<robot a='1' a='2'/>").ok(), "duplicate attribute");
    fineurdf::FixedString<32> formatted{};
    fineurdf::append_fixed(formatted, 1.0, 3);
    check(formatted.view() == "1.000", "fixed formatter zero padding");
    std::cout << checks << " checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}
