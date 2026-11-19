# MC_Board：从 URDF 和控制器 YAML 构造设备

这是 `pov_simple.urdf` 的设备构造入口，不是旧 `finemote_cpp` 示例，也不是 constexpr URDF 入口。在仓库根目录使用 CMake、Ninja、ARM GCC、支持 JSON AST 的 armclang/clang 和安装了 PyYAML 的 Python 3.9+：

```powershell
cmake --preset ast-MC_Board-release -B build/device-pov `
  -DFINEMOTE_REFLECTED_URDF=Interface/Examples/URDF/pov_simple.urdf `
  -DFINEMOTE_CONTROLLERS_YAML=Interface/Examples/URDF/pov_controllers.yaml
cmake --build build/device-pov
```

若有多个 Python 或 Clang，可在配置命令中指定 `-DPython3_EXECUTABLE=...` 和 `-DFINEMOTE_AST_CLANG=...`。PyYAML 只在主机的新模式使用；旧 AST 示例不传 YAML，使用单独的构建目录。修改 URDF 或 YAML 后只需重新 build。

`reflected/robot.cpp` 是实际参与编译的 C++17 源码。`reflected/interfaces.json` 的 `joints` 记录 joint 到电机、控制器实例、端口、地址和分频的绑定，`chassis.modules` 保留模块顺序，`construction_order` 是构造顺序，不是运行时调度顺序。

本例生成四个速度 PID、四个位置 Amplifier、八个电机和一个 `POV_Chassis<4, PlanarOdom>`。零位从 rad 转为驱动使用的度；各电机显式开启等价圈位置处理。设备只由已有 DeviceScheduler 调度。

## micro-ROS 话题

沿用 MC_Board 板级 UART5 和节点前缀 FineMote。`interfaces.json` 的 `ros` 项列出相对该前缀的话题：本例占用 10 个 agent（9 个发布器、1 个订阅器）和 1 个 executor 句柄。生成阶段拒绝超预算的配置。旧的遥控、心跳、LED 对象不参与装配。

- `/FineMote/joints/<joint>/JointState`：每个关节一条，携带真实关节名和声明的 position（rad）、velocity（rad/s），effort 留空。4010 报告编码器角度，不保证累计多圈。时间戳为发布采样时刻的同步 ROS 时间，未同步为零，不是硬件采集时间。
- `/FineMote/chassis/Twist`：轮组反馈估计的底盘平面速度。缺少首次有效反馈或反馈超过 250 ms 时，关节或底盘相应观测量为 NaN。管理线程每轮处理后等待 200 ms；这不是保证的发布周期。
- `/FineMote/chassis/cmd_vel`：best-effort、深度 1 的 Twist。只接受 linear.x/y（m/s）和 angular.z（rad/s），其余分量必须为零。内部接收边界为平移模长 ≤ 1 m/s、角速度绝对值 ≤ 2 rad/s；这是软件边界，不代表实机安全速度。

## 命令与停车

上电、检测到 agent 断连、超过 **600 ms** 未收到有效命令或收到非法命令后，先发一条**全零 Twist** 解锁，再连续发送有效命令。零命令本身不驱动轮组；建议以 10 Hz 发送，并在发运动指令前持续发送零命令至少两个管理周期。超时按 MCU 本地接收时间计算；Twist 不含源端时间戳。恢复连接不会自动重放旧目标。

订阅回调只更新邮箱；底盘任务每个调度周期检查命令，电机按自己的分频执行保护。禁止运动时，4010 跳过 PID、清除积分和历史误差，并发零原始转矩；4315 有新鲜反馈时发送实测位置作为保持目标，无反馈时不发送新的位置。**零转矩是撤销驱动，不是机械制动**，也无法保证断电、执行器总线断线或 MCU 停摆后的停车距离。该保护针对 ROS 命令链路，不是电机掉线联锁或硬件急停。

主机回归：`python Tests/URDFReflection/test_devices.py`；在仓库根目录用 C++17 主机编译器编译运行 `Tests/URDFReflection/communication_tests.cpp`（include 路径：`Components`、`ThirdParty/micro-ROS/include`）。上板验收应先架空轮组并准备独立急停，核对话题、单位、方向、停发/拔线、重连后无旧命令和任务栈高水位。链接成功不能替代上述测试。
