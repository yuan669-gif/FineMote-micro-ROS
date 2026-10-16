/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

/**
 * @file  finemote.hpp
 * @brief Projects the parsed URDF model onto the compile-time FineMote configuration.
 *
 * @details
 * Mapping applied here:
 *   continuous joint about z      -> steer motor (position control)
 *   continuous joint about y      -> drive motor (speed control)
 *   joint origin xyz              -> swerve module lx/ly
 *   explicit joint params         -> wheel radius, steering calibration
 *   transmission mechanical ratio -> Motor_Param_t::reductionRatio
 *   ros2_control joint params     -> Motor_Param_t, division factor, PID, limits
 *
 * FineMote headers are deliberately not included: MotorBase.hpp pulls in the board
 * BSP. This layer produces configuration data; constructing devices and converting
 * SI limits into driver units requires a board-specific adapter.
 */

#ifndef FINEMOTE_URDF_FINEMOTE_HPP
#define FINEMOTE_URDF_FINEMOTE_HPP

#include "urdf.hpp"
#include "format.hpp"

namespace fineurdf {

// Mirrors Motor_Ctrl_Type_e from Devices/Motors/MotorBase.hpp.
enum class CtrlType : std::uint8_t { Position = 0, Speed = 1, Torque = 2 };
enum class TargetType : std::uint8_t { Position = 0, Speed = 1, Torque = 2 };

constexpr bool ctrl_type_from(sv s, CtrlType& out) noexcept {
    if (s == "position") { out = CtrlType::Position; return true; }
    if (s == "speed" || s == "velocity") { out = CtrlType::Speed; return true; }
    if (s == "torque" || s == "effort") { out = CtrlType::Torque; return true; }
    return false;
}

constexpr bool target_type_from(sv s, TargetType& out) noexcept {
    if (s == "position") { out = TargetType::Position; return true; }
    if (s == "speed" || s == "velocity") { out = TargetType::Speed; return true; }
    if (s == "torque" || s == "effort") { out = TargetType::Torque; return true; }
    return false;
}

// 对应 FineMote 里 TORQUE_2_SPEED / DIRECT_POSITION 这组宏
struct MotorParamView {  // ~ Motor_Param_t
    CtrlType ctrl_type = CtrlType::Position;
    TargetType target_type = TargetType::Position;
    bool multi_turn_same_position = false;
    float reduction_ratio = 1.0f;
};

struct MotorSpec {
    Name joint{};
    Name link{};
    Name kind{};                   // URDF 声明的电机型号字符串，如 "4315"/"4010"
    Name can_bus{};
    std::uint32_t can_id = 0;
    bool can_id_valid = false;
    MotorParamView params{};
    std::uint16_t division_factor = 1;   // Pass to the motor constructor; no runtime setter exists.
    float pid_p = 0.f, pid_i = 0.f, pid_d = 0.f;
    bool has_pid = false;
    float limit_lower = 0.f, limit_upper = 0.f;
    float limit_effort = 0.f, limit_velocity = 0.f;
    bool is_steer = false;         // 绕 base z 轴
    bool is_drive = false;         // 绕 steer_link y 轴
    bool has_wheel_radius = false;
    bool has_zero_position = false;
    float wheel_radius = 0.f;
    float zero_position = 0.f;
};

// 舵轮模块视图 —— 与 FineMote POV_Chassis 的 Swerve_t 逐字段同形
struct SwerveModuleView {  // ~ Swerve_t
    Name steer_joint{}, drive_joint{};
    int steer_motor = -1;      // 在 motors[] 中的下标，等价于 &SFRMotor
    int drive_motor = -1;
    float lx = 0.f, ly = 0.f;  // 相对底盘中心，米
    float zero_position = 0.f; // 度
    float wheel_radius = 0.f;  // 米
};

// ---------------------------------------------------------------- 规格常量提取
namespace detail {

template <std::size_t MP, std::size_t MI, std::size_t MM>
constexpr bool has_command_interface(const HwJointInfo<MP, MI, MM>& hj, sv n) noexcept {
    for (std::uint16_t i = 0; i < hj.command_count; ++i)
        if (hj.command_interfaces[i].name.is(n)) return true;
    return false;
}

// 解析 "<param name="pid" value="1.2 0.0 0.05">" 里的三个浮点
constexpr void parse_pid(sv s, float& p, float& i, float& d) noexcept {
    float v[3] = {0.f, 0.f, 0.f};
    int k = 0;
    std::size_t pos = 0;
    while (k < 3 && pos < s.size()) {
        while (pos < s.size() && (is_space(s[pos]) || s[pos] == ',')) ++pos;
        const std::size_t b = pos;
        while (pos < s.size() && !is_space(s[pos]) && s[pos] != ',') ++pos;
        if (pos > b) v[k++] = parse_float(s.substr(b, pos - b));
    }
    p = v[0]; i = v[1]; d = v[2];
}

template <std::size_t N>
constexpr int find_motor_slot(const std::array<MotorSpec, N>& ms, std::uint16_t n, sv joint) noexcept {
    for (std::uint16_t i = 0; i < n; ++i)
        if (ms[i].joint.is(joint)) return static_cast<int>(i);
    return -1;
}

}  // namespace detail

// Callers use static_assert(config.ok) to enforce this validation at compile time.
struct RobotValidation {
    bool parsed = false;
    bool hardware_valid = false;
    sv first_failure{};

    constexpr bool ok() const noexcept {
        return parsed && hardware_valid;
    }
};

template <ModelLimits L>
constexpr RobotValidation validate(const RobotModel<L>& m) noexcept {
    RobotValidation v{};
    v.parsed = m.ok;
    if (!m.ok) { v.first_failure = m.err_text; return v; }

    auto fail = [&](sv text) constexpr { v.first_failure = text; return v; };
    for (std::uint16_t i = 0; i < m.joint_count; ++i) {
        if (m.joints[i].type == JointType::Floating || m.joints[i].type == JointType::Planar)
            return fail("multi-DOF joints are not supported by the motor adapter");
        if (!is_actuated(m.joints[i].type)) continue;
        unsigned claims = 0;
        for (std::uint16_t h = 0; h < m.hardware_count; ++h)
            for (std::uint16_t k = 0; k < m.hardware[h].joint_count; ++k)
                if (m.hardware[h].joints[k].name == m.joints[i].name) ++claims;
        if (claims != 1) return fail("actuated joint must be claimed exactly once");
    }
    for (std::uint16_t h = 0; h < m.hardware_count; ++h) {
        const auto& hw = m.hardware[h];
        if (hw.name.empty() || hw.type == HwType::Unknown) return fail("invalid hardware name/type");
        for (std::uint16_t previous = 0; previous < h; ++previous)
            if (hw.name == m.hardware[previous].name) return fail("duplicate hardware name");
        for (std::uint16_t p = 0; p < hw.param_count; ++p)
            for (std::uint16_t q = 0; q < p; ++q)
                if (hw.params[p].name == hw.params[q].name) return fail("duplicate hardware parameter");
        for (std::uint16_t a = 0; a < hw.joint_count; ++a) {
            const auto& hj = hw.joints[a];
            if (hj.mimic_count != 0) return fail("mimic hardware joints are not supported");
            const auto* joint = m.find_joint(hj.name.view());
            if (!joint || !is_actuated(joint->type)) return fail("hardware references an unknown or non-actuated joint");
            for (std::uint16_t p = 0; p < hj.param_count; ++p)
                for (std::uint16_t q = 0; q < p; ++q)
                    if (hj.params[p].name == hj.params[q].name) return fail("duplicate joint parameter");
            const auto* id = hj.find_param("can_id");
            std::uint64_t number = 0;
            if (!id || !try_parse_uint(id->value.view(), number) || number > 0x7ff)
                return fail("can_id must be an 11-bit unsigned CAN identifier");
            const auto* kind = hj.find_param("motor_kind");
            if (!kind || kind->value.empty()) return fail("motor_kind is required");
            if (const auto* p = hj.find_param("division_factor")) {
                std::uint64_t division = 0;
                if (!try_parse_uint(p->value.view(), division) || division == 0 || division > 255)
                    return fail("division_factor must be in [1, 255] for MotorBase");
            }
            CtrlType ctrl{};
            TargetType target{};
            const auto* c = hj.find_param("control_type");
            const auto* t = hj.find_param("target_type");
            if (!c || !ctrl_type_from(c->value.view(), ctrl) || !t || !target_type_from(t->value.view(), target))
                return fail("invalid or missing control_type/target_type");
            const sv command = target == TargetType::Position ? "position" : target == TargetType::Speed ? "velocity" : "effort";
            if (!detail::has_command_interface(hj, command)) return fail("target_type has no matching command interface");
            if (const auto* p = hj.find_param("pid"))
                if (!valid_vector(p->value.view())) return fail("pid requires three finite numbers");
            if (const auto* p = hj.find_param("multi_turn_same_position")) {
                const sv text = p->value.view();
                if (!(iequals(text, "true") || iequals(text, "false") || text == "1" || text == "0" ||
                      iequals(text, "yes") || iequals(text, "no") || iequals(text, "on") || iequals(text, "off")))
                    return fail("invalid multi_turn_same_position boolean");
            }
            for (sv key : {sv{"wheel_radius"}, sv{"zero_position"}})
                if (const auto* p = hj.find_param(key))
                    if (!valid_float(p->value.view())) return fail("invalid wheel radius or steering calibration");
            const auto* bus = hw.find_param("can_bus");
            const sv bus_name = bus ? bus->value.view() : sv{};
            for (std::uint16_t hh = 0; hh <= h; ++hh) {
                const auto* other_bus = m.hardware[hh].find_param("can_bus");
                if (bus_name != (other_bus ? other_bus->value.view() : sv{})) continue;
                const auto end = hh == h ? a : m.hardware[hh].joint_count;
                for (std::uint16_t b = 0; b < end; ++b) {
                    const auto* other = m.hardware[hh].joints[b].find_param("can_id");
                    std::uint64_t other_number = 0;
                    if (other && try_parse_uint(other->value.view(), other_number) && number == other_number)
                        return fail("duplicate CAN identifier on the same bus");
                }
            }
        }
    }
    v.hardware_valid = true;
    return v;
}

// ---------------------------------------------------------------- 完整配置

struct FinemoteLimits {
    ModelLimits model{};
    std::size_t motors = 32;
    std::size_t modules = 8;
};

inline constexpr FinemoteLimits kDefaultFinemoteLimits{};
// Mechanical joint order for diagnostics. The device dependency graph also includes
// controllers, buses and chassis; it cannot be inferred from mechanical ancestry.

struct OrderedJoint {
    std::uint16_t joint_index = 0;  // index into RobotModel::joints
    std::uint16_t depth = 0;         // distance from the root of the joint tree
    std::uint16_t division_factor = 1;
};

template <std::size_t MaxJoints>
struct JointOrder {
    std::array<OrderedJoint, MaxJoints> order{};
    std::uint16_t count = 0;
    bool acyclic = true;            // false when the joint graph has a cycle
    std::uint16_t max_depth = 0;
    std::uint16_t bucket_count = 0;  // number of distinct division factors
    bool ok = false;
};

namespace detail {

/**
 * @brief Depth of a joint measured from the root of the joint tree.
 */
template <ModelLimits L>
constexpr std::uint16_t joint_depth(const RobotModel<L>& m, std::uint16_t ji) noexcept {
    std::uint16_t d = 0;
    Name cur = m.joints[ji].parent;
    for (std::uint16_t guard = 0; guard < L.joints; ++guard) {
        int up = -1;
        for (std::uint16_t k = 0; k < m.joint_count; ++k) {
            if (m.joints[k].child == cur) { up = static_cast<int>(k); break; }
        }
        if (up < 0) break;
        cur = m.joints[static_cast<std::uint16_t>(up)].parent;
        ++d;
    }
    return d;
}

}  // namespace detail

/**
 * @brief Leaf-to-root topological order of the joint tree.
 * @note Disconnected graphs are accepted; a cyclic graph yields acyclic = false.
 */
template <ModelLimits L>
constexpr JointOrder<L.joints> build_joint_order(const RobotModel<L>& m) noexcept {
    JointOrder<L.joints> s{};
    if (!m.ok) return s;

    const std::uint16_t n = m.joint_count;
    std::array<bool, L.joints> emitted{};

    // Kahn 算法：一个关节只有在"它的所有子关节都已输出"之后才能输出。
    // child_exists[j] = 是否存在关节 j2 满足 j2.parent == j.child
    for (std::uint16_t pass = 0; pass < n; ++pass) {
        bool progressed = false;
        for (std::uint16_t j = 0; j < n; ++j) {
            if (emitted[j]) continue;
            // 还没输出的子关节？
            bool has_pending_child = false;
            for (std::uint16_t k = 0; k < n; ++k) {
                if (emitted[k]) continue;
                if (m.joints[k].parent == m.joints[j].child) { has_pending_child = true; break; }
            }
            if (has_pending_child) continue;

            OrderedJoint& d = s.order[s.count];
            d.joint_index = j;
            d.depth = detail::joint_depth(m, j);
            if (d.depth > s.max_depth) s.max_depth = d.depth;
            // 分频取自该关节对应的 <ros2_control> 参数；找不到就按 1
            d.division_factor = 1;
            for (std::uint16_t h = 0; h < m.hardware_count; ++h) {
                const auto* hj = m.hardware[h].find_joint(m.joints[j].name.view());
                if (hj == nullptr) continue;
                if (const ParamInfo* p = hj->find_param("division_factor"))
                    d.division_factor = static_cast<std::uint16_t>(parse_uint(p->value.view()));
                break;
            }
            emitted[j] = true;
            ++s.count;
            progressed = true;
        }
        if (!progressed) break;   // 剩下的都有未输出的子关节 ⇒ 有环
    }

    s.acyclic = (s.count == n);
    // Number of distinct division factors in this joint description.
    std::array<std::uint16_t, L.joints> seen{};
    std::uint16_t nb = 0;
    for (std::uint16_t i = 0; i < s.count; ++i) {
        bool dup = false;
        for (std::uint16_t k = 0; k < nb; ++k)
            if (seen[k] == s.order[i].division_factor) { dup = true; break; }
        if (!dup) seen[nb++] = s.order[i].division_factor;
    }
    s.bucket_count = nb;
    s.ok = s.acyclic;
    return s;
}

// 把编译期算出的期望顺序渲染成一段可读文本（用于运行期报错）。
template <std::size_t Cap, ModelLimits L>
constexpr void append_expected_order(FixedString<Cap>& out, const RobotModel<L>& m,
                                     const JointOrder<L.joints>& s) noexcept {
    for (std::uint16_t i = 0; i < s.count; ++i) {
        out.append("  [");
        append_uint(out, i);
        out.append("] d=");
        append_uint(out, s.order[i].depth);
        out.append(" div=");
        append_uint(out, s.order[i].division_factor);
        out.push(' ');
        out.append(m.joints[s.order[i].joint_index].name.view());
        out.push('\n');
    }
}

template <FinemoteLimits FL>
struct FinemoteConfig {
    static_assert(FL.motors < 65535 && FL.modules < 65535, "Configuration budget exceeds index range");
    static constexpr FinemoteLimits limits = FL;

    RobotModel<FL.model> model{};
    std::array<MotorSpec, FL.motors> motors{};
    std::uint16_t motor_count = 0;
    std::array<SwerveModuleView, FL.modules> modules{};
    std::uint16_t module_count = 0;

    // 由 URDF 几何反推出来的底盘尺度（原来是手写的 ROBOT_LENGTH / ROBOT_WIDTH / WHEEL_DIAMETER）
    float wheelbase = 0.f;      // 前后轴距 -> ROBOT_LENGTH
    float track_width = 0.f;    // 左右轮距 -> ROBOT_WIDTH
    float wheel_radius = 0.f;
    float wheel_diameter = 0.f;
    Name device_kind{};         // <hardware><param name="device_kind">

    // Leaf-to-root mechanical joint order, including fixed joints.
    JointOrder<FL.model.joints> joint_order{};

    bool ok = false;
    sv err_text{};
};

using DefaultFinemoteConfig = FinemoteConfig<kDefaultFinemoteLimits>;

/**
 * @brief Parse, project and validate a URDF document, producing the FineMote config.
 * @note The named local for the document is required: binding a temporary would make
 *       the views dangle, which constant evaluation rejects outright.
 */
template <FinemoteLimits FL>
constexpr FinemoteConfig<FL> build_config(sv urdf) noexcept {
    using Config = FinemoteConfig<FL>;
    Config cfg{};

    // 1. syntax
    const auto doc = parse_checked<FL.model.syntax>(urdf, XmlLimits{});
    if (!doc.ok()) {
        cfg.err_text = xml_err_message(doc.err);
        return cfg;
    }

    // ---- 2. 语义层：URDF ----
    cfg.model = project_urdf<FL.model>(doc);
    if (!cfg.model.ok) {
        cfg.err_text = cfg.model.err_text;
        return cfg;
    }

    using Model = RobotModel<FL.model>;
    const Model& m = cfg.model;
    const auto validation = validate(m);
    if (!validation.ok()) { cfg.err_text = validation.first_failure; return cfg; }

    // ---- 3. 每个 <ros2_control>/<joint> -> 一个 MotorSpec ----
    for (std::uint16_t h = 0; h < m.hardware_count; ++h) {
        const auto& hw = m.hardware[h];
        if (h == 0) {
            if (const ParamInfo* dk = hw.find_param("device_kind")) cfg.device_kind = dk->value;
        }
        for (std::uint16_t k = 0; k < hw.joint_count; ++k) {
            if (cfg.motor_count >= FL.motors) { cfg.err_text = "motor capacity exceeded"; return cfg; }
            const auto& hj = hw.joints[k];
            const JointInfo* j = m.find_joint(hj.name.view());
            if (j == nullptr) { cfg.err_text = "unknown hardware joint"; return cfg; }

            MotorSpec& spec = cfg.motors[cfg.motor_count++];
            spec.joint = j->name;
            spec.link = j->child;
            if (const auto* bus = hw.find_param("can_bus")) spec.can_bus = bus->value;

            // 关节限位（编译期常量，可直接喂给 Clamp / PID 限幅）
            spec.limit_lower = j->limit.lower;
            spec.limit_upper = j->limit.upper;
            spec.limit_effort = j->limit.effort;
            spec.limit_velocity = j->limit.velocity;

            // 辨认舵向 / 轮向：舵向绕 base 的 z 轴，轮向绕 steer_link 的 y 轴。
            // Only direct base-Z / steer-Y pairs are recognized as swerve modules.
            spec.is_steer = (j->axis.z != 0.f && j->axis.x == 0.f && j->axis.y == 0.f);
            spec.is_drive = (j->axis.y != 0.f && j->axis.x == 0.f && j->axis.z == 0.f);

            // 减速比：优先 <transmission>，缺省 1.0
            if (const TransmissionInfo* t = m.find_transmission_by_joint(j->name.view())) {
                spec.params.reduction_ratio = t->mechanical_reduction;
            }

            // ros2_control 的 <param>
            if (const ParamInfo* p = hj.find_param("can_id")) {
                spec.can_id = static_cast<std::uint32_t>(parse_uint(p->value.view()));
                spec.can_id_valid = true;
            }
            if (const ParamInfo* p = hj.find_param("motor_kind")) spec.kind = p->value;
            else if (const ParamInfo* p = hw.find_param("device_kind")) spec.kind = p->value;
            if (const ParamInfo* p = hj.find_param("control_type")) ctrl_type_from(p->value.view(), spec.params.ctrl_type);
            if (const ParamInfo* p = hj.find_param("target_type")) target_type_from(p->value.view(), spec.params.target_type);
            if (const ParamInfo* p = hj.find_param("multi_turn_same_position"))
                spec.params.multi_turn_same_position = parse_bool(p->value.view());
            if (const ParamInfo* p = hj.find_param("division_factor"))
                spec.division_factor = static_cast<std::uint16_t>(parse_uint(p->value.view()));
            if (const ParamInfo* p = hj.find_param("pid")) {
                detail::parse_pid(p->value.view(), spec.pid_p, spec.pid_i, spec.pid_d);
                spec.has_pid = true;
            }
            if (const ParamInfo* p = hj.find_param("wheel_radius")) {
                spec.has_wheel_radius = true;
                spec.wheel_radius = parse_float(p->value.view());
            }
            if (const ParamInfo* p = hj.find_param("zero_position")) {
                spec.has_zero_position = true;
                spec.zero_position = parse_float(p->value.view());
            }
        }
    }

    // 4. Mechanical joint diagnostics.
    cfg.joint_order = build_joint_order(m);
    if (!cfg.joint_order.ok) {
        cfg.err_text = "cyclic joint graph";
        return cfg;
    }

    // ---- 5. 舵轮模块几何推导 ----
    // 模块中心 = steer 关节 origin 相对 base_link 的平移
    // Wheel radius and steering zero are explicit calibration parameters.
    const LinkInfo* base = m.find_link("base_link");
    if (base != nullptr) {
        for (std::uint16_t i = 0; i < m.joint_count; ++i) {
            const JointInfo& sj = m.joints[i];
            if (sj.type != JointType::Revolute && sj.type != JointType::Continuous) continue;
            if (!(sj.axis.z != 0.f && sj.axis.x == 0.f && sj.axis.y == 0.f)) continue;  // 只认舵向
            if (!sj.parent.is("base_link")) continue;

            // 在 steer_link 之下找一个绕 y 轴转的关节 = 轮向
            int drive_idx = -1;
            for (std::uint16_t k = 0; k < m.joint_count; ++k) {
                const JointInfo& dj = m.joints[k];
                if (dj.parent != sj.child) continue;
                if ((dj.type != JointType::Revolute && dj.type != JointType::Continuous) ||
                    dj.axis.y == 0.f || dj.axis.x != 0.f || dj.axis.z != 0.f) continue;
                if (drive_idx >= 0) { cfg.err_text = "ambiguous swerve drive joint"; return cfg; }
                drive_idx = static_cast<int>(k);
            }
            if (drive_idx < 0) continue;
            if (cfg.module_count >= FL.modules) { cfg.err_text = "swerve module capacity exceeded"; return cfg; }

            SwerveModuleView& mod = cfg.modules[cfg.module_count++];
            mod.steer_joint = sj.name;
            mod.drive_joint = m.joints[static_cast<std::uint16_t>(drive_idx)].name;
            mod.lx = sj.origin.xyz.x;
            mod.ly = sj.origin.xyz.y;
            mod.steer_motor = detail::find_motor_slot(cfg.motors, cfg.motor_count, sj.name.view());
            mod.drive_motor = detail::find_motor_slot(cfg.motors, cfg.motor_count,
                                                     m.joints[static_cast<std::uint16_t>(drive_idx)].name.view());

            const auto& drive_joint = m.joints[static_cast<std::uint16_t>(drive_idx)];
            if (sj.axis.z != 1.f || drive_joint.axis.y != 1.f ||
                sj.origin.rpy.x != 0.f || sj.origin.rpy.y != 0.f || sj.origin.rpy.z != 0.f ||
                drive_joint.origin.rpy.x != 0.f || drive_joint.origin.rpy.y != 0.f || drive_joint.origin.rpy.z != 0.f) {
                cfg.err_text = "swerve projection requires +Z/+Y axes and unrotated joint frames";
                return cfg;
            }
            if (mod.steer_motor < 0 || mod.drive_motor < 0) {
                cfg.err_text = "swerve motors are not configured"; return cfg;
            }
            const auto& steer = cfg.motors[mod.steer_motor];
            const auto& drive = cfg.motors[mod.drive_motor];
            if (!steer.has_zero_position || !drive.has_wheel_radius || drive.wheel_radius <= 0.f) {
                cfg.err_text = "swerve requires explicit zero_position (degrees) and positive wheel_radius (metres)";
                return cfg;
            }
            mod.zero_position = steer.zero_position;
            mod.wheel_radius = drive.wheel_radius;
        }
    }

    if (cfg.module_count > 0) {
        float min_x = cfg.modules[0].lx, max_x = min_x;
        float min_y = cfg.modules[0].ly, max_y = min_y;
        cfg.wheel_radius = cfg.modules[0].wheel_radius;
        cfg.wheel_diameter = 2.f * cfg.wheel_radius;
        for (std::uint16_t i = 0; i < cfg.module_count; ++i) {
            const auto& mod = cfg.modules[i];
            if (mod.wheel_radius != cfg.wheel_radius) {
                cfg.err_text = "POV chassis requires equal wheel radii"; return cfg;
            }
            if (mod.lx < min_x) min_x = mod.lx;
            if (mod.lx > max_x) max_x = mod.lx;
            if (mod.ly < min_y) min_y = mod.ly;
            if (mod.ly > max_y) max_y = mod.ly;
        }
        cfg.wheelbase = max_x - min_x; // URDF X: longitudinal direction
        cfg.track_width = max_y - min_y;
    }
    cfg.ok = m.ok;
    return cfg;
}

// 默认容量下的便捷入口（与上面的模板同名会歧义，故用不同名字）
constexpr DefaultFinemoteConfig build_config_default(sv urdf) noexcept {
    return build_config<kDefaultFinemoteLimits>(urdf);
}

}  // namespace fineurdf

#endif  // FINEURDF_FINEMOTE_HPP
