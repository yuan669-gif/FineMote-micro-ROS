# FineMote 描述语言与自动生成实现交接

记录日期：2026-11-17。

2026-11-18 实现进展：已接入 URDF/YAML 到 C++17 的设备生成、分散状态发布、底盘命令订阅和命令超时保护。当前入口、协议和验证边界见 [DEVICE_ASSEMBLY.md](DEVICE_ASSEMBLY.md)；下文保留 11 月 17 日的设计交接状态。

本文记录本轮讨论的最终结果，供下一轮实现自动生成使用。配套输入为：

- [pov_simple.urdf](pov_simple.urdf)：机械结构、设备、连接、接口及底盘装配。
- [pov_controllers.yaml](pov_controllers.yaml)：控制算法、参数及电机调度分频。

**当前状态：这两份文件是已完成语法检查的设计草案，尚未接入生成器，不能直接据此构建固件。** 本轮只修改示例与文档，没有实现新的接口层、ROS 对象自动生成或运行时控制逻辑，也没有进行硬件验证。

## 1. 已确定的定位与边界

我们借鉴 ros2_control 的接口思想，设计 FineMote 自己的描述语言。保留 `ros2_control`、`joint`、`state_interface`、`command_interface` 等概念，但不要求这份输入能被标准 ros2_control 原样加载。

最终输入是 URDF 与 YAML 两份文件，在主机构建阶段解析并生成 C++ 固件代码。MCU 不需要运行时 XML/YAML 解析器。

面向用户不暴露 C++ 头文件、构造参数排列、对象指针或模板实例化细节。已有 Clang AST 构造能力应继续利用；型号别名、单位、驱动模式等语义需要设备适配层明确提供，不能由 AST 猜测。

已经取消或暂不引入的配置：

- 不写 `finemote`、`finemote_cpp`、`include header` 或 `transmission`。
- 不写 `hardware/plugin`；示例中的空 `hardware` 也已移除。
- 不写 micro-ROS 开关、串口、话题名、发布器或订阅器配置；这些由 FineMote 内部管理。
- 不使用遥控器；`remote` 与 Zorro 任务不纳入新示例。
- motor 不写 `output`、`period_ms` 或总线属性。
- 不创建汇总所有关节的状态发布器，采用分散发布。
- 不额外创建旧示例的心跳、LED 通信对象。

旧文件 `pov_chassis.urdf`、`pov_reflected.urdf` 及其说明描述的是旧入口，不应将其语法要求重新加到新示例中。后续兼容策略可以另行安排。

## 2. URDF 写法

### 2.1 机械描述与设备绑定

robot 下的标准 `link/joint` 描述机械关系。`ros2_control/joint` 通过唯一的 `name` 引用机械 joint；其内部的 motor 和 bus 共同描述该关节的设备。

```xml
<joint name="wheel_joint_FR">
  <command_interface name="velocity"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>

  <bus type="can">
    <param name="port">1</param>
    <param name="id">0x141</param>
  </bus>

  <motor type="4010"/>
</joint>
```

- 型号写 `4010`、`4315`，不要求用户写 C++ 类名 `Motor4010`。
- bus 与 motor 是同级元素；统一采用 `bus type` 加子级 `param` 的形式。
- CAN 使用 `port`、`id`；RS485 使用 `port`、`address`。
- `port` 是板级总线编号，不是 MCU 引脚号，也不是 PC 的串口路径。
- 首版每个受控 joint 对应一个 motor、一个 bus、一个命令接口。
- 不增加第二套 motor 名称或单独的 joint-device 绑定块。

RS485 示例：

```xml
<bus type="rs485">
  <param name="port">1</param>
  <param name="address">0x01</param>
</bus>
<motor type="4315"/>
```

### 2.2 等价圈位置处理

用户决定将 `multi_turn_same_position` 当作默认开启，暂不在 URDF 或 YAML 暴露字段。

它用于位置目标的等价圈选择，例如当前位置 350°、目标 10°时，选择等价的 370°。不是开启编码器多圈测量。

**实现注意：现有 `Motor_Param_t::multiTurnSamePosition` 默认是 false。新描述语言的默认行为必须由生成器或相应适配层落实，不能因为 XML 未写就沿用现有 C++ 默认值。** 不必为此顺带改变所有手写应用的全局默认行为。是否以后开放 YAML 覆盖，本轮不处理。

### 2.3 底盘

采用实例名、类型及子元素分工的写法：

```xml
<chassis name="chassis" type="pov">
  <command_interface name="velocity"/>
  <state_interface name="velocity"/>

  <param name="wheel_diameter">0.0483</param>
  <odometry type="planar"/>

  <module steer="steer_joint_FR" drive="wheel_joint_FR">
    <param name="zero_position">3.141592653589793</param>
  </module>
  <module steer="steer_joint_FL" drive="wheel_joint_FL"/>
  <module steer="steer_joint_BL" drive="wheel_joint_BL"/>
  <module steer="steer_joint_BR" drive="wheel_joint_BR">
    <param name="zero_position">3.141592653589793</param>
  </module>
</chassis>
```

- `name` 标识底盘实例，`type=pov` 选择实现。
- 数值参数采用 `param`；功能组成采用 `odometry`、`module` 等子元素。
- `pov` 对应 `POV_Chassis<N, OdomPolicy>`，`planar` 对应 `PlanarOdom`，N 由 module 数量推导。
- 生成器把 joint 引用解析成实际电机对象；不依赖 FR/FL 等字符串推测绑定。
- `wheel_diameter` 单位为 m；`zero_position` 单位为 rad，省略时为 0。
- 适配现有 `Swerve_t::zeroPosition` 时需要转换为度，不得重复应用零位偏移。
- 本例 steer joint 直接连接 base_link，其 origin 的 x/y 用于模块 lx/ly；drive joint 是其直接子关节。不应把这个特例当作已支持任意多级坐标变换。
- 底盘速度接口在 base_link 坐标系中表示 `(vx, vy, wz)`，单位为 m/s、m/s、rad/s。
- PlanarOdom 继续在内部工作，本版没有额外的 Odometry/位姿发布器。

当前 POV 实现持有电机指针，执行正运动学、里程计更新、逆运动学，并直接调用电机 setter。模板参数只有轮组数量和里程计策略；轮径、安装位置和设备对象通过构造参数传入。

## 3. YAML 写法与控制器语义

明确采用列表中的多个 `- controller:`，不使用 wheel_speed、steer_position 等强制实例名，不使用同一级重复的 YAML 映射键。

```yaml
version: 1

controllers:
  - controller:
      type: PID
      joints:
        - wheel_joint_FR
        - wheel_joint_FL
        - wheel_joint_BL
        - wheel_joint_BR
      division_factor: 1
      parameters:
        kp: 0.23
        ki: 0.008
        kd: 0.3

  - controller:
      type: Amplifier
      joints:
        - steer_joint_FR
        - steer_joint_FL
        - steer_joint_BL
        - steer_joint_BR
      division_factor: 20
      parameters:
        gain: 1
```

`division_factor` 与 `parameters` 并列，不增加 scheduling 包装层。

每一项为列出的每个 joint 分别创建控制器实例，仅复用参数值；PID 积分和历史误差不能在不同电机之间共享。首版一个 joint 只能绑定一个控制器配置。

FineMote 当前 PID、Amplifier 是被电机调用的计算模块；ros2_control 的 controller 是还承担接口申请、更新入口和生命周期等职责的框架组件。两者可以采用相同算法，但当前不能直接互换。这份 YAML 配置 MCU 内部算法，不是 PC 端可直接加载的 ROS 2 参数文件。

### 3.1 分频

执行周期 = RTOS 基础 tick 周期 × division_factor。

- 对当前电机构造参数，分频范围为整数 1～255，禁止零值或静默截断。
- 分频作用于绑定电机的 DeviceScheduler 周期，包括其 Update/Handle 调度，不是只对 PID 单独分频。
- MC_Board 当前 tick 为 1 ms；本例轮电机 1 ms、舵电机 20 ms。
- 底盘暂保留默认 1 ms 周期；不能套用舵电机的分频。
- 配置列表顺序不表示调度先后顺序。
- 当前 PID 按调用次数积累误差和差分，没有显式 dt；迁移时保持原控制周期和度/DPS内部单位。不能直接将旧增益用于 rad/rad/s 误差，也不能改周期后声称控制效果相同。

### 3.2 当前样例支持组合

|设备与对外命令|内部算法|原驱动模式|
|---|---|---|
|4010 + velocity|PID，速度反馈|targetType=Speed，ctrlType=Torque|
|4315 + position|Amplifier<1>，位置目标透传|targetType=Position，ctrlType=Position|

用户不写 output，以上关系由内部适配规则明确确定。不能从 PID 类名推断任何电机的输出模式。原始 Torque 协议量不等于物理单位 N·m。

Amplifier 的 gain 在当前实现中对应模板 `K`，类型为 size_t；构建期生成实例，不应未经实现就接受任意负数或小数增益。

## 4. micro-ROS 接口与资源数量

### 4.1 通信对象分配：最终采用分散发布

|对象|生成的通信对象|消息类型与含义|
|---|---|---|
|8 个电机 joint|每个 joint 一个状态发布器，共 8 个|sensor_msgs/msg/JointState，仅包含该 joint|
|chassis 状态接口|1 个状态发布器|geometry_msgs/msg/Twist，实际估计速度|
|chassis 命令接口|1 个命令订阅器|geometry_msgs/msg/Twist，目标速度|

**合计 9 个 Publisher + 1 个 Subscriber = 10 个 FineMote ROSAgent；executor 需要 1 个订阅句柄。** 普通发布器不占 executor 订阅句柄。

“八个电机各一个状态”指八个状态话题。每个 joint 的 position、velocity 仍是两个状态接口，打包进同一条单关节 JointState 消息。不能按每个标量接口分别生成发布器。

电机 command_interface 供 MCU 内底盘算法写入，不为八个电机额外生成命令订阅器。底盘持有这些关节的命令控制权。

控制流：

```text
ROS 2 底盘速度命令
  -> micro-ROS 订阅接收
  -> 底盘目标
  -> POV 逆运动学
  -> 各 joint 位置/速度命令
  -> 各电机的 PID / Amplifier
  -> CAN / RS485 输出

电机反馈 -> 各 joint 状态 -> 各自 JointState 发布器
电机反馈 -> POV 正运动学 -> 底盘实际速度 -> Twist 发布器
```

JointState 要填写真实 joint 名称，按声明填充 position/velocity，使用关节侧 SI 单位；不发布未经标定的 effort。不得以目标值冒充实测状态。

Twist 的 linear.x、linear.y、angular.z 分别为 vx、vy、wz；其他分量限定为零。MCU 内部应处理无效输入、命令陈旧与断连，不能让通信回调直接无同步地改写控制任务数据。

### 4.2 当前代码资源上限

在本次检查的代码中：

- MICROROS_MAX_AGENTS=10：FineMote 发布/订阅对象注册表容量。
- MICROROS_MAX_HANDLES=10：executor 句柄容量。
- 随库配置头中 RMW_UXRCE_MAX_PUBLISHERS=10、MAX_SUBSCRIPTIONS=5。
- 管理线程请求的栈大小为 20 KiB；所有对象共用该线程，不是每个对象各有一个线程。

新示例恰好用满注册表；不能同时保留原 TaskPOVChassis 的心跳、LED 订阅，或重复创建旧的底盘发布器。板级是否还有其他 ROS 对象必须检查，不能只数本示例。

10 是工程资源配置，不是 MCU 固有极限。现有 RegisterAgent 在容量满时静默忽略对象，后续实现应改成明确错误，并在生成阶段校验资源预算。底层池扩容涉及预编译 micro-ROS 静态库，单改头文件不会改变库内容量。

### 4.3 尚需在内部确定的实现细节

以下没有在讨论中冻结具体值，不能误写成已实现或用户已指定：

- topic 的确切路径、命名空间和是否沿用现有发布器的消息类型后缀。
- 发布频率、QoS、命令超时阈值、失联停车及重新激活策略。
- 状态有效性、设备掉线和时间戳同步的具体实现。
- 控制任务与 ROS 任务之间的邮箱/快照交换实现。
- URDF/YAML 的构建参数名称、文件配对入口和生成物目录。

这些细节应由 FineMote 内部统一约定，本轮没有要求增加用户配置字段。

## 5. 需要复用和检查的代码

以下路径均相对仓库根目录：

|路径|用途|
|---|---|
|tools/urdf_codegen/generate.py|现有 Clang AST 反射、对象构造表达式与依赖排序|
|cmake/reflected_urdf.cmake|现有生成流程与重建依赖|
|Interface/Examples/URDF/URDF_AST_MAPPING.md|旧 AST 路径的能力与限制|
|Algorithms/URDF、Components/RobotModel|另一条 constexpr 解析/静态装配路径，注意与新入口区分|
|Interface/Examples/TaskPOVChassis.cpp|原 Case 1 对象及参数基准，遥控与旧 ROS 部分不迁入新配置|
|Devices/Motors/MotorBase.hpp|目标设置、减速比、位置圈数选择、状态及现有 ROS binder|
|Devices/Motors/Motor4010.hpp、Motor4315.hpp|控制算法调用与协议输出|
|Algorithms/Control/PID.hpp、ImplementControlBase.hpp|PID、Amplifier 模板和算法接口|
|Components/Chassis/POV_Chassis.hpp、ChassisBase.hpp、SwerveCommand.hpp|底盘构造、运动学、里程计及电机调用|
|Devices/DeviceBase/DeviceScheduler.cpp|分频桶、Update 正序和 Handle 逆序调度|
|Services/MicroROS/MicroROS_Agent.hpp|现有发布/订阅封装|
|Services/MicroROS/MicroROS_Manager.hpp|注册表、线程、executor、连接状态机|
|Services/MicroROS/MicroROS_Transport.hpp|板级 UART 传输|
|ThirdParty/micro-ROS/include/rmw_microxrcedds_c/config.h|随库资源配置|

已知实现衔接点：

1. 现有 AST 生成器读取 finemote_cpp，不能直接处理新语法或 YAML。建议增加语义解析/适配层，复用已有构造能力；不要求重新手写所有驱动构造映射。
2. 模型别名与公开头文件入口由框架内部解决，用户不写 include。
3. 当前 MotorBase ROS binder 未填写 joint 名称，位置/速度直接使用度/DPS，torque 是协议值；新发布器不能原样沿用这些数值。
4. JointState 的字符串、序列需要提前准备内存；订阅缓冲、消息对象和回调引用必须具有稳定生命周期。
5. micro-ROS 管理线程当前每轮等待 200 ms，不能当成 1 ms 电机闭环。通信频率与设备调度分频分别处理。
6. AST 构造依赖顺序不等于运行时调度顺序。不能新增另一条循环重复调用同一电机 Handle 或 PID；要保证底盘目标与电机控制执行关系正确。
7. 现有 Enable/Disable 为空、Stop 的 Torque 分支未实现；不能将这些方法直接称为已完成的失联停机。
8. 单位、减速比、方向和零位必须只转换一次。MotorBase setter 已包含减速比处理；4010 的位置来自单圈编码器，需要明确累计位置语义。不要未经验证宣称多圈位置完整可用。
9. 旧 CMake 模式有互斥约束；新模式应避免与手写 TaskPOVChassis、旧生成对象同时链接，产生重复电机或 ROS 对象。

## 6. 建议实施与验收顺序

以下是实现建议，不是当前已有能力：

1. 解析两份输入，形成统一的关节、设备、控制器、底盘与通信装配描述。
2. 做语义校验：重名、未知字段、悬空引用、重复控制器绑定、不支持的型号/总线/接口组合、非法分频、错误数值、端口/地址冲突。
3. 通过内部适配规则及 AST 生成独立控制器、八个电机、一个 POV 底盘和十个 ROS 对象；同时生成可检查的绑定与资源清单。
4. 接通状态转换与底盘命令通路，明确任务间同步、控制权、断连行为及初始化次序。
5. 将 URDF、YAML、设备适配描述及关联头文件纳入 CMake 增量依赖。
6. 主机测试配置拒绝、引用乱序、独立 PID 状态、分频、构造顺序、SI 转换和消息内容；再进行目标编译链接与资源检查。
7. 硬件阶段检查八条状态话题、底盘状态/命令、实际运动方向、零位、周期及断连行为。软件检查通过不能替代硬件验收。

至少应能核对以下示例参数：

|轮组|4010 / CAN1 ID|4315 / RS4851 地址|舵向零位|
|---|---|---|---|
|FR|0x141|0x01|π rad|
|FL|0x142|0x02|0|
|BL|0x143|0x03|0|
|BR|0x144|0x04|π rad|

轮径 0.0483 m；舵轮中心 x=±0.1201125 m、y=±0.12 m；四个 PID 参数均为 0.23/0.008/0.3，四个 Amplifier 增益均为 1。几何及 limit 沿用已有示例，并非本轮重新标定的硬件数据。

## 7. 新对话启动说明

可以将下面这段作为新对话的任务入口：

> 请先阅读 Interface/Examples/URDF/FINEMOTE_DESCRIPTION_HANDOFF.md，以及同目录的 pov_simple.urdf、pov_controllers.yaml，然后实现这套 FineMote 描述语言到固件的自动生成。保留既定的 XML/YAML 写法，复用现有 AST 构造能力；micro-ROS 内部固定配置，生成八个电机状态发布器、一个底盘状态发布器和一个底盘命令订阅器，不添加汇总 joint 发布器、遥控、心跳或 LED 演示对象。区分已确定的描述约定和文档列出的待落实实现细节，完成适当的软件验证，并明确报告未进行的硬件验证。

## 8. 本次交接的验证范围

已用 XML 与 YAML 解析器读取示例，检查两组 controller 配置覆盖八个 joint 且没有重复绑定，确认底盘接口与轮组引用存在。新格式尚未构建固件或烧录。

历史参考：用户提供的桌面《micro-ROS开发文档.md》将发布器上限 10 解释为当时“5 个电机 + 底盘 odom + 余量”的资源预算，没有单独说明 MAX_AGENTS/MAX_HANDLES 同为 10 的选取理由。其栈大小等历史描述与当前代码有差异，不能用作本次新配置已通过测试的证据。
