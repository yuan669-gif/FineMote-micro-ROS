/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

/**
 * @file  urdf.hpp
 * @brief Projects the generic XML document onto a typed robot description.
 *
 * @details
 * Keeping this separate from xml.hpp splits syntax errors from URDF semantics
 * errors. ModelLimits is a C++20 structural template parameter; names are owned
 * fixed-capacity strings so projected models do not retain document-local views.
 */

#ifndef FINEMOTE_URDF_URDF_HPP
#define FINEMOTE_URDF_URDF_HPP

#include "xml.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace fineurdf {

/**
 * @brief Fixed-capacity string without pointers, usable inside a template argument.
 */
template <std::size_t N>
struct FixStr {
    static_assert(N <= 255, "FixStr capacity exceeds length representation");
    std::array<char, N> data{};
    std::uint8_t len = 0;

    constexpr FixStr() noexcept = default;

    constexpr FixStr(sv s) noexcept : len(static_cast<std::uint8_t>(s.size() < N ? s.size() : N)) {
        for (std::uint8_t i = 0; i < len; ++i) data[i] = s[i];
    }

    constexpr sv view() const noexcept { return sv{data.data(), len}; }
    constexpr bool empty() const noexcept { return len == 0; }
    constexpr bool is(sv s) const noexcept { return view() == s; }

    friend constexpr bool operator==(const FixStr& a, const FixStr& b) noexcept {
        if (a.len != b.len) return false;
        for (std::uint8_t i = 0; i < a.len; ++i)
            if (a.data[i] != b.data[i]) return false;
        return true;
    }

    friend constexpr bool operator!=(const FixStr& a, const FixStr& b) noexcept { return !(a == b); }

    // No deduction guide: N cannot be deduced from a sv argument, and armclang
    // rejects a guide whose template parameter cannot be deduced.
};

/**
 * @brief Capacity for names. "transmission_interface/SimpleTransmission" is
 *        already 41 characters, so 64 keeps a safe margin.
 */
inline constexpr std::size_t kNameCap = 64;

using Name = FixStr<kNameCap>;

enum class JointType : std::uint8_t {
    Unknown = 0,
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
};

constexpr sv to_string(JointType t) noexcept {
    switch (t) {
        case JointType::Revolute: return "revolute";
        case JointType::Continuous: return "continuous";
        case JointType::Prismatic: return "prismatic";
        case JointType::Fixed: return "fixed";
        case JointType::Floating: return "floating";
        case JointType::Planar: return "planar";
        case JointType::Unknown: return "unknown";
    }
    return "unknown";
}

constexpr JointType joint_type_from(sv s) noexcept {
    if (s == "revolute") return JointType::Revolute;
    if (s == "continuous") return JointType::Continuous;
    if (s == "prismatic") return JointType::Prismatic;
    if (s == "fixed") return JointType::Fixed;
    if (s == "floating") return JointType::Floating;
    if (s == "planar") return JointType::Planar;
    return JointType::Unknown;
}

constexpr bool is_actuated(JointType t) noexcept {
    return t == JointType::Revolute || t == JointType::Continuous || t == JointType::Prismatic;
}

// ros2_control 的 <ros2_control type="..."> 取值
enum class HwType : std::uint8_t { Unknown = 0, System, Sensor, Actuator };

constexpr HwType hw_type_from(sv s) noexcept {
    if (s == "system") return HwType::System;
    if (s == "sensor") return HwType::Sensor;
    if (s == "actuator") return HwType::Actuator;
    return HwType::Unknown;
}

// ---------------------------------------------------------------- 几何 / 限位

struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;
    // URDF 的 origin xyz="x y z"，也用于 inertia 的 ixx.. 等
    constexpr Vec3() noexcept = default;
    constexpr Vec3(float a, float b, float c) noexcept : x(a), y(b), z(c) {}
};

struct JointLimit {
    bool present = false;
    float lower = 0.f;
    float upper = 0.f;
    float effort = 0.f;    // N 或 N*m
    float velocity = 0.f;  // rad/s 或 m/s
};

struct JointOrigin {
    Vec3 xyz{};
    Vec3 rpy{};  // 弧度。URDF 规范就是弧度，这里不做角度转换
};

struct LinkInfo {
    Name name{};
    bool has_inertial = false;
    float mass = 0.f;
};

struct JointInfo {
    Name name{};
    JointType type = JointType::Unknown;
    Name parent{};
    Name child{};
    Vec3 axis{1.f, 0.f, 0.f};  // URDF 默认轴 (1,0,0)
    JointOrigin origin{};
    JointLimit limit{};
};

// <transmission>（老式 ROS1 风格，ros2_control 仍支持读取）
struct TransmissionInfo {
    Name name{};
    Name type{};               // "transmission_interface/SimpleTransmission"
    Name joint{};              // <joint name="...">
    Name actuator{};           // <actuator name="...">
    float mechanical_reduction = 1.f;
    float offset = 0.f;
};

// ---------------------------------------------------------------- ros2_control

struct InterfaceInfo {
    Name name{};               // "position" / "velocity" / "effort"
    Name data_type{};          // "double"
    bool has_initial_value = false;
    double initial_value = 0.0;
    bool has_min = false, has_max = false;
    double min = 0.0, max = 0.0;
};

struct ParamInfo {
    Name name{};
    Name value{};              // 原样保留为字符串，具体含义由适配层解释
};

// <ros2_control> 下面的一个 <joint>（或 <sensor>/<gpio>，本原型只做 joint）
template <std::size_t MaxParams, std::size_t MaxIfaces, std::size_t MaxMimic>
struct HwJointInfo {
    Name name{};
    std::array<InterfaceInfo, MaxIfaces / 2> command_interfaces{};
    std::uint16_t command_count = 0;
    std::array<InterfaceInfo, MaxIfaces> state_interfaces{};
    std::uint16_t state_count = 0;
    std::array<InterfaceInfo, MaxMimic> mimic{};
    std::uint16_t mimic_count = 0;

    // 关节级 <param>
    std::array<ParamInfo, MaxParams> params{};
    std::uint16_t param_count = 0;

    constexpr const ParamInfo* find_param(sv n) const noexcept {
        for (std::uint16_t i = 0; i < param_count; ++i)
            if (params[i].name.is(n)) return &params[i];
        return nullptr;
    }
};

// 一个 <ros2_control> 块 = 一个 hardware component
template <std::size_t MaxParams, std::size_t MaxJoints, std::size_t MaxIfaces, std::size_t MaxMimic>
struct HardwareComponent {
    Name name{};
    HwType type = HwType::Unknown;
    Name plugin{};                 // <hardware><plugin>...</plugin>
    std::array<ParamInfo, MaxParams> params{};   // <hardware><param name value>
    std::uint16_t param_count = 0;
    std::array<HwJointInfo<MaxParams, MaxIfaces, MaxMimic>, MaxJoints> joints{};
    std::uint16_t joint_count = 0;

    // 按名字找 joint（joint 数很少，线性查找即可；这里也刻意避免运行时哈希表）
    constexpr const HwJointInfo<MaxParams, MaxIfaces, MaxMimic>* find_joint(sv n) const noexcept {
        for (std::uint16_t i = 0; i < joint_count; ++i)
            if (joints[i].name.is(n)) return &joints[i];
        return nullptr;
    }
    constexpr const ParamInfo* find_param(sv n) const noexcept {
        for (std::uint16_t i = 0; i < param_count; ++i)
            if (params[i].name.is(n)) return &params[i];
        return nullptr;
    }
};

/**
 * @brief Capacity of the semantic model. Kept explicit because the memory budget
 *        of a constant-evaluated parse has to be visible, not "as much as needed".
 *        Exceeding it is reported as a readable error instead of truncation.
 */
struct ModelLimits {
    std::size_t links = 64;
    std::size_t joints = 64;
    std::size_t transmissions = 64;
    std::size_t hardware = 4;
    std::size_t hw_joints = 64;
    std::size_t hw_params = 12;
    std::size_t ifaces = 8;
    // 语法层（XML 元素/属性）预算。必须 >= 语义层，因为 XML 元素比语义实体多。
    SyntaxBudget syntax{};
};

template <ModelLimits L>
struct RobotModel {
    static_assert(L.links < 65535 && L.joints < 65535 && L.transmissions < 65535 &&
                  L.hardware < 65535 && L.hw_joints < 65535 && L.hw_params < 65535 &&
                  L.ifaces < 65535, "Model budget exceeds index range");
    static constexpr ModelLimits limits = L;
    using HwJoint = HwJointInfo<L.hw_params, L.ifaces, 3>;
    using Hardware = HardwareComponent<L.hw_params, L.hw_joints, L.ifaces, 3>;

    Name name{};
    std::array<LinkInfo, L.links> links{};
    std::uint16_t link_count = 0;
    std::array<JointInfo, L.joints> joints{};
    std::uint16_t joint_count = 0;
    std::array<TransmissionInfo, L.transmissions> transmissions{};
    std::uint16_t transmission_count = 0;
    std::array<Hardware, L.hardware> hardware{};
    std::uint16_t hardware_count = 0;

    bool ok = false;
    XmlErr err = XmlErr::None;
    std::size_t err_offset = 0;
    sv err_text{};

    constexpr const JointInfo* find_joint(sv n) const noexcept {
        for (std::uint16_t i = 0; i < joint_count; ++i)
            if (joints[i].name.is(n)) return &joints[i];
        return nullptr;
    }
    constexpr const LinkInfo* find_link(sv n) const noexcept {
        for (std::uint16_t i = 0; i < link_count; ++i)
            if (links[i].name.is(n)) return &links[i];
        return nullptr;
    }
    constexpr const TransmissionInfo* find_transmission_by_joint(sv n) const noexcept {
        for (std::uint16_t i = 0; i < transmission_count; ++i)
            if (transmissions[i].joint.is(n)) return &transmissions[i];
        return nullptr;
    }
    // 某关节是否被任何 <ros2_control> 块覆盖
    constexpr bool joint_is_claimed(sv n) const noexcept {
        for (std::uint16_t i = 0; i < hardware_count; ++i)
            if (hardware[i].find_joint(n) != nullptr) return true;
        return false;
    }
    constexpr std::uint16_t actuated_joint_count() const noexcept {
        std::uint16_t k = 0;
        for (std::uint16_t i = 0; i < joint_count; ++i)
            if (is_actuated(joints[i].type)) ++k;
        return k;
    }
};

// 默认容量下的模型类型（大多数项目直接用这个）
inline constexpr ModelLimits kDefaultModelLimits{};
using DefaultRobotModel = RobotModel<kDefaultModelLimits>;

// ---------------------------------------------------------------- 投影实现
namespace detail {

// 在文档里遍历 parent 之下、tag 为 t 的所有直接子节点
template <std::size_t MN, std::size_t MA, typename F>
constexpr void for_each_child(const XmlDocument<MN, MA>& d, const XmlNode& parent, sv t, F&& f) noexcept {
    const std::uint16_t base = static_cast<std::uint16_t>(&parent - d.nodes.data());
    for (std::uint16_t c = d.nodes[base].first_child; c != XmlNode::kNone;
         c = d.nodes[c].next_sibling)
        if (d.nodes[c].tag == t) f(c);
}

template <std::size_t MN, std::size_t MA>
constexpr const XmlNode* find_child(const XmlDocument<MN, MA>& d, const XmlNode& parent, sv t) noexcept {
    const std::uint16_t base = static_cast<std::uint16_t>(&parent - d.nodes.data());
    for (std::uint16_t c = d.nodes[base].first_child; c != XmlNode::kNone;
         c = d.nodes[c].next_sibling)
        if (d.nodes[c].tag == t) return &d.nodes[c];
    return nullptr;
}

template <std::size_t MA>
constexpr sv attr_value(const std::array<XmlAttr, MA>& pool, const XmlNode& n, sv name) noexcept {
    for (std::uint8_t i = 0; i < n.attr_count; ++i) {
        const XmlAttr& a = pool[static_cast<std::size_t>(n.attr_begin) + i];
        if (a.name == name) return a.value;
    }
    return {};
}

template <std::size_t MA>
constexpr bool has_attr(const std::array<XmlAttr, MA>& pool, const XmlNode& n, sv name) noexcept {
    for (std::uint8_t i = 0; i < n.attr_count; ++i)
        if (pool[static_cast<std::size_t>(n.attr_begin) + i].name == name) return true;
    return false;
}

/**
 * @brief Read an attribute, falling back to the element text.
 * @note URDF uses both spellings for the same quantity: <mass value="1.85"/> and
 *       <mechanical_reduction>1.0</mechanical_reduction>. Looking only at the
 *       attribute yields 0.0 without any diagnostic.
 */
template <std::size_t MA>
constexpr sv attr_or_text(const std::array<XmlAttr, MA>& pool, const XmlNode& n, sv name) noexcept {
    const sv v = attr_value(pool, n, name);
    if (!v.empty()) return v;
    return trim(n.text);
}

// 把 "x y z" 形式的属性解析成 Vec3；缺失属性时保留默认值
constexpr Vec3 parse_vec3(sv s, Vec3 fallback = {}) noexcept {
    if (s.empty()) return fallback;
    float v[3] = {0.f, 0.f, 0.f};
    int k = 0;
    std::size_t i = 0;
    while (k < 3 && i <= s.size()) {
        while (i < s.size() && (is_space(s[i]))) ++i;
        const std::size_t b = i;
        while (i < s.size() && !is_space(s[i])) ++i;
        if (i > b) v[k++] = parse_float(s.substr(b, i - b));
        else break;
        if (i >= s.size()) break;
    }
    return Vec3{v[0], v[1], v[2]};
}

}  // namespace detail

/**
 * @brief Project an XML document onto RobotModel<L>.
 */
template <ModelLimits L, std::size_t MN, std::size_t MA>
constexpr RobotModel<L> project_urdf(const XmlDocument<MN, MA>& doc) noexcept {
    using namespace detail;

    RobotModel<L> m{};
    m.err = doc.err;
    m.err_offset = doc.err_offset;
    if (!doc.ok()) {
        m.err_text = xml_err_message(doc.err);
        return m;
    }

    const std::uint16_t root = doc.root;
    if (doc.nodes[root].tag != "robot") {
        m.err = XmlErr::NoRoot;
        m.err_text = "root element is not <robot>";
        return m;
    }
    const XmlNode& robot = doc.nodes[root];
    m.name = Name{attr_value(doc.attrs, robot, "name")};

    // Validate before copying into fixed-capacity names or converting numbers.
    for (std::uint16_t i = 0; i < doc.node_count; ++i) {
        const auto& n = doc.nodes[i];
        for (std::uint8_t a = 0; a < n.attr_count; ++a) {
            const auto& attr = doc.attrs[n.attr_begin + a];
            if ((attr.name == "name" || attr.name == "link" || attr.name == "data_type") &&
                attr.value.size() > kNameCap) {
                m.err_text = "name exceeds kNameCap"; return m;
            }
            if ((attr.name == "xyz" || attr.name == "rpy") && !valid_vector(attr.value)) {
                m.err_text = "invalid xyz/rpy vector"; return m;
            }
            if ((attr.name == "lower" || attr.name == "upper" || attr.name == "effort" ||
                 attr.name == "velocity" || attr.name == "initial_value") && !valid_float(attr.value)) {
                m.err_text = "invalid numeric attribute"; return m;
            }
        }
        if ((n.tag == "param" || n.tag == "plugin" || n.tag == "type") &&
            attr_or_text(doc.attrs, n, "value").size() > kNameCap) {
            m.err_text = "parameter text exceeds kNameCap"; return m;
        }
        if ((n.tag == "mass" || n.tag == "mechanical_reduction" || n.tag == "mechanicalReduction" ||
             n.tag == "offset" || n.tag == "min" || n.tag == "max") &&
            !valid_float(attr_or_text(doc.attrs, n, "value"))) {
            m.err_text = "invalid numeric element"; return m;
        }
    }

    // ---- <link> ----
    for_each_child(doc, robot, "link", [&](std::uint16_t c) {
        if (m.link_count >= L.links) { m.err_text = "link capacity exceeded"; return; }
        LinkInfo& l = m.links[m.link_count++];
        l.name = Name{attr_value(doc.attrs, doc.nodes[c], "name")};
        if (const XmlNode* in = find_child(doc, doc.nodes[c], "inertial")) {
            l.has_inertial = true;
            if (const XmlNode* ms = find_child(doc, *in, "mass"))
                l.mass = parse_float(attr_or_text(doc.attrs, *ms, "value"));
        }
    });

    // ---- <joint> ----
    for_each_child(doc, robot, "joint", [&](std::uint16_t c) {
        if (m.joint_count >= L.joints) { m.err_text = "joint capacity exceeded"; return; }
        const XmlNode& jn = doc.nodes[c];
        JointInfo& j = m.joints[m.joint_count++];
        j.name = Name{attr_value(doc.attrs, jn, "name")};
        j.type = joint_type_from(trim(attr_value(doc.attrs, jn, "type")));
        if (find_child(doc, jn, "mimic")) m.err_text = "mimic joints are not supported";

        if (const XmlNode* p = find_child(doc, jn, "parent"))
            j.parent = Name{attr_value(doc.attrs, *p, "link")};
        if (const XmlNode* ch = find_child(doc, jn, "child"))
            j.child = Name{attr_value(doc.attrs, *ch, "link")};
        if (const XmlNode* ax = find_child(doc, jn, "axis")) {
            const Vec3 a = parse_vec3(attr_value(doc.attrs, *ax, "xyz"), Vec3{1.f, 0.f, 0.f});
            j.axis = a;
            if (a.x == 0.f && a.y == 0.f && a.z == 0.f) m.err_text = "zero joint axis";
        }
        if (const XmlNode* o = find_child(doc, jn, "origin")) {
            j.origin.xyz = parse_vec3(attr_value(doc.attrs, *o, "xyz"));
            j.origin.rpy = parse_vec3(attr_value(doc.attrs, *o, "rpy"));
        }
        if (const XmlNode* li = find_child(doc, jn, "limit")) {
            j.limit.present = true;
            j.limit.lower = parse_float(attr_value(doc.attrs, *li, "lower"));
            j.limit.upper = parse_float(attr_value(doc.attrs, *li, "upper"));
            j.limit.effort = parse_float(attr_value(doc.attrs, *li, "effort"));
            j.limit.velocity = parse_float(attr_value(doc.attrs, *li, "velocity"));
        }
    });

    // ---- <transmission> ----
    for_each_child(doc, robot, "transmission", [&](std::uint16_t c) {
        if (m.transmission_count >= L.transmissions) { m.err_text = "transmission capacity exceeded"; return; }
        const XmlNode& tn = doc.nodes[c];
        TransmissionInfo& t = m.transmissions[m.transmission_count++];
        t.name = Name{attr_value(doc.attrs, tn, "name")};
        if (const XmlNode* ty = find_child(doc, tn, "type"))
            t.type = Name{trim(ty->text)};
        if (const XmlNode* j = find_child(doc, tn, "joint")) {
            t.joint = Name{attr_value(doc.attrs, *j, "name")};
            // Accepts both placements (inside <joint> or directly under <transmission>)
            // and both spellings, mechanical_reduction and mechanicalReduction.
            if (const XmlNode* mr = find_child(doc, *j, "mechanical_reduction"))
                t.mechanical_reduction = parse_float(attr_or_text(doc.attrs, *mr, "value"));
            else if (const XmlNode* mr2 = find_child(doc, tn, "mechanical_reduction"))
                t.mechanical_reduction = parse_float(attr_or_text(doc.attrs, *mr2, "value"));
            else if (const XmlNode* mr3 = find_child(doc, tn, "mechanicalReduction"))
                t.mechanical_reduction = parse_float(attr_or_text(doc.attrs, *mr3, "value"));
            else if (const XmlNode* a2 = find_child(doc, tn, "actuator")) {
                // 有些文件把 reduction 挂在 <actuator> 下
                if (const XmlNode* mr4 = find_child(doc, *a2, "mechanicalReduction"))
                    t.mechanical_reduction = parse_float(attr_or_text(doc.attrs, *mr4, "value"));
                else if (const XmlNode* mr5 = find_child(doc, *a2, "mechanical_reduction"))
                    t.mechanical_reduction = parse_float(attr_or_text(doc.attrs, *mr5, "value"));
            }
            if (const XmlNode* of = find_child(doc, *j, "offset"))
                t.offset = parse_float(attr_or_text(doc.attrs, *of, "value"));
        }
        if (const XmlNode* a = find_child(doc, tn, "actuator"))
            t.actuator = Name{attr_value(doc.attrs, *a, "name")};
    });

    // ---- <ros2_control> ----
    for_each_child(doc, robot, "ros2_control", [&](std::uint16_t c) {
        if (m.hardware_count >= L.hardware) { m.err_text = "hardware capacity exceeded"; return; }
        const XmlNode& hn = doc.nodes[c];
        typename RobotModel<L>::Hardware& h = m.hardware[m.hardware_count++];
        h.name = Name{attr_value(doc.attrs, hn, "name")};
        h.type = hw_type_from(trim(attr_value(doc.attrs, hn, "type")));

        if (const XmlNode* hw = find_child(doc, hn, "hardware")) {
            if (const XmlNode* pl = find_child(doc, *hw, "plugin"))
                h.plugin = Name{trim(pl->text)};
            for_each_child(doc, *hw, "param", [&](std::uint16_t pc) {
                if (h.param_count >= h.params.size()) { m.err_text = "hardware parameter capacity exceeded"; return; }
                ParamInfo& p = h.params[h.param_count++];
                p.name = Name{attr_value(doc.attrs, doc.nodes[pc], "name")};
                p.value = Name{attr_or_text(doc.attrs, doc.nodes[pc], "value")};
            });
        }

        for_each_child(doc, hn, "joint", [&](std::uint16_t jc) {
            if (h.joint_count >= h.joints.size()) { m.err_text = "hardware joint capacity exceeded"; return; }
            const XmlNode& jnode = doc.nodes[jc];
            typename RobotModel<L>::HwJoint& hj = h.joints[h.joint_count++];
            hj.name = Name{attr_value(doc.attrs, jnode, "name")};
            for_each_child(doc, jnode, "param", [&](std::uint16_t pc) {
                if (hj.param_count >= hj.params.size()) { m.err_text = "joint parameter capacity exceeded"; return; }
                ParamInfo& p = hj.params[hj.param_count++];
                p.name = Name{attr_value(doc.attrs, doc.nodes[pc], "name")};
                // 兼容两种写法：<param name="x" value="1"/> 与 <param name="x">1</param>
                p.value = Name{attr_or_text(doc.attrs, doc.nodes[pc], "value")};
            });
            for_each_child(doc, jnode, "command_interface", [&](std::uint16_t ic) {
                if (hj.command_count >= hj.command_interfaces.size()) { m.err_text = "command interface capacity exceeded"; return; }
                InterfaceInfo& ii = hj.command_interfaces[hj.command_count++];
                ii.name = Name{attr_value(doc.attrs, doc.nodes[ic], "name")};
                ii.data_type = Name{attr_value(doc.attrs, doc.nodes[ic], "data_type")};
            });
            for_each_child(doc, jnode, "state_interface", [&](std::uint16_t ic) {
                if (hj.state_count >= hj.state_interfaces.size()) { m.err_text = "state interface capacity exceeded"; return; }
                InterfaceInfo& ii = hj.state_interfaces[hj.state_count++];
                const XmlNode& in = doc.nodes[ic];
                ii.name = Name{attr_value(doc.attrs, in, "name")};
                ii.data_type = Name{attr_value(doc.attrs, in, "data_type")};
                // initial_value 有属性写法，也有官方文档里的 <param name="initial_value"> 写法
                if (has_attr(doc.attrs, in, "initial_value")) {
                    ii.has_initial_value = true;
                    ii.initial_value = parse_double(attr_value(doc.attrs, in, "initial_value"));
                }
                for_each_child(doc, in, "param", [&](std::uint16_t pc) {
                    if (ii.has_initial_value) return;
                    if (attr_value(doc.attrs, doc.nodes[pc], "name") != "initial_value") return;
                    ii.has_initial_value = true;
                    ii.initial_value = parse_double(attr_or_text(doc.attrs, doc.nodes[pc], "value"));
                });
                if (const XmlNode* mn = find_child(doc, in, "min")) {
                    ii.has_min = true;
                    ii.min = parse_double(attr_or_text(doc.attrs, *mn, "value"));
                }
                if (const XmlNode* mx = find_child(doc, in, "max")) {
                    ii.has_max = true;
                    ii.max = parse_double(attr_or_text(doc.attrs, *mx, "value"));
                }
            });
            for_each_child(doc, jnode, "mimic", [&](std::uint16_t mc) {
                if (hj.mimic_count >= hj.mimic.size()) { m.err_text = "mimic capacity exceeded"; return; }
                InterfaceInfo& mi = hj.mimic[hj.mimic_count++];
                mi.name = Name{attr_value(doc.attrs, doc.nodes[mc], "name")};
            });
        });
    });

    if (!m.err_text.empty()) return m;
    if (m.name.empty() || m.link_count == 0) { m.err_text = "robot requires a name and links"; return m; }
    for (std::uint16_t i = 0; i < m.link_count; ++i) {
        if (m.links[i].name.empty()) { m.err_text = "link requires a name"; return m; }
        for (std::uint16_t k = 0; k < i; ++k)
            if (m.links[i].name == m.links[k].name) { m.err_text = "duplicate link name"; return m; }
    }
    for (std::uint16_t i = 0; i < m.joint_count; ++i) {
        const auto& j = m.joints[i];
        if (j.name.empty() || j.type == JointType::Unknown) { m.err_text = "invalid joint name/type"; return m; }
        if (!m.find_link(j.parent.view()) || !m.find_link(j.child.view()) || j.parent == j.child) {
            m.err_text = "joint has a missing or self-referencing link"; return m;
        }
        if (j.limit.lower > j.limit.upper || j.limit.effort < 0 || j.limit.velocity < 0 ||
            ((j.type == JointType::Revolute || j.type == JointType::Prismatic) && !j.limit.present)) {
            m.err_text = "invalid joint limits"; return m;
        }
        for (std::uint16_t k = 0; k < i; ++k)
            if (j.name == m.joints[k].name || j.child == m.joints[k].child) {
                m.err_text = "duplicate joint name or child link"; return m;
            }
        Name current = j.child;
        for (std::uint16_t depth = 0; depth <= m.joint_count; ++depth) {
            const JointInfo* parent = nullptr;
            for (std::uint16_t k = 0; k < m.joint_count; ++k)
                if (m.joints[k].child == current) { parent = &m.joints[k]; break; }
            if (!parent) break;
            if (depth == m.joint_count) { m.err_text = "cyclic joint graph"; return m; }
            current = parent->parent;
        }
    }
    if (m.link_count != m.joint_count + 1) { m.err_text = "robot links must form one connected tree"; return m; }
    for (std::uint16_t i = 0; i < m.transmission_count; ++i) {
        const auto& t = m.transmissions[i];
        if (!m.find_joint(t.joint.view()) || t.mechanical_reduction <= 0) {
            m.err_text = "invalid transmission joint/reduction"; return m;
        }
        for (std::uint16_t k = 0; k < i; ++k)
            if (t.name == m.transmissions[k].name || t.joint == m.transmissions[k].joint) {
                m.err_text = "duplicate transmission"; return m;
            }
    }
    m.ok = true;
    return m;
}

}  // namespace fineurdf

#endif  // FINEURDF_URDF_HPP
