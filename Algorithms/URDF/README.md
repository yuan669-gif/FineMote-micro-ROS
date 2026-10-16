# URDF 配置解析

`xml.hpp` 负责定容 XML 子集解析，`urdf.hpp` 提取关节树与硬件描述，
`finemote.hpp` 校验并生成电机、舵轮配置数据。无需堆分配或第三方 XML 库。

```cpp
#include "URDF/finemote.hpp"
#include "my_robot_urdf.hpp"

inline constexpr auto config = fineurdf::build_config_default(my_robot_LITERAL);
static_assert(config.ok, "Invalid robot description: inspect config.err_text");
```

`constexpr` 初始化和 `static_assert` 才保证调用发生在编译期；这些函数也可在主机运行，
供测试使用。C++20 的结构化模板参数用于容量设置 `ModelLimits`、`FinemoteLimits`。
编译失败时可查看 `config.err_text`；它指向静态错误说明，不引用已销毁的局部对象。

当前能力与约束：

- 支持具名单根 link 树、fixed/revolute/continuous/prismatic 关节和 ros2_control joint 配置。
  固定关节无需硬件声明，每个可驱动关节必须且只能由一个硬件 joint 声明。
- 解析质量、轴、原点、限位、SimpleTransmission 所用减速比以及硬件参数。
  模型容量、名称长度和数值出错时返回 `ok == false`，不会截断后仍报告成功。
- 电机适配层要求 `motor_kind`、11 位 `can_id`、`control_type`、`target_type` 和匹配的 command interface。
  `division_factor` 默认 1，范围 1–255，与当前 MotorBase 构造参数一致。
  CAN ID 按数值和 `can_bus` 检查重复；缺少 `can_bus` 的声明归入同一默认域。
- 舵轮识别仅覆盖 base_link 下的 +Z 舵向关节及其直接子 +Y 驱动关节，要求原点 rpy 为零。
  轮关节必须显式给出 `wheel_radius`（米），舵向关节必须给出 `zero_position`（度）。
  单一底盘的轮半径必须相同。X 方向跨度为 wheelbase，Y 方向跨度为 track_width。
- `joint_order` 仅供机械关节拓扑诊断，包括 fixed 关节。它不代表设备注册顺序，
  也不控制 DeviceScheduler 的线程和分频桶。
- XML 子集支持注释、声明、处理指令、引号属性和 UTF-8 BOM；拒绝 DTD、CDATA、实体引用。
  不支持 Xacro 展开、mimic、多自由度电机适配及完整 visual/collision/inertia 几何建模。

输出是配置数据，不能直接转换成驱动对象或按内存布局强转。
URDF 限位使用 SI 单位，MotorBase 的角度、速度使用度和 DPS；
实例化适配器需要显式转换单位、选择总线和型号、创建控制器，并建立实际设备依赖顺序。
