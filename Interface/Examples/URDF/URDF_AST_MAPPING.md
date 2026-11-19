# URDF → Clang AST → C++17 固件

本方案在构建时读取 FineMote 的真实 C++ 声明，生成普通 C++17 对象定义。
生成器没有 `PID → PID_Param_t`、电机型号或配置成员名称的登记表。
固件不包含 XML 解析器，也不包含 AST 反射运行时。

## 使用方式

依赖：CMake、Ninja、ARM GCC、Python 3.9+，以及支持 JSON AST 输出的 Clang。
本机验证使用 Arm Compiler 6.24 的 armclang 和 ARM GCC 13.3.1。
Python 是主机侧构建工具；生成代码及本构建路径的 C++ 编译模式均为 C++17。
本文说明显式构造图的 AST 后端。主线设备描述入口见 [DEVICE_ASSEMBLY.md](DEVICE_ASSEMBLY.md)；旧 constexpr 实现保存在分支 `codex/micro-ROS-constexpr`。

在已经设置 ARM GCC、CMake、Ninja 的 PATH 后执行：

```powershell
cmake --preset ast-MC_Board-release -B build/pov-reflected `
  '-DFINEMOTE_CONTROLLERS_YAML=' `
  '-DFINEMOTE_REFLECTED_URDF=Interface/Examples/URDF/pov_reflected.urdf' `
  '-DFINEMOTE_AST_CLANG=D:/new_STM32/STM32/ARM/ARMCLANG/bin/armclang.exe'
cmake --build build/pov-reflected --parallel 6
```

新机器人可以复制并修改 URDF，然后在配置命令中选择该文件，无须新增机器人专用 C++ 源文件。
生成物位于构建目录的 `reflected/robot.cpp` 和 `reflected/interfaces.json`。
AST 的 MC_Board GCC 构建生成 `MC_Board.elf`、`.hex` 和 `.bin`。
修改 URDF 或项目接口头文件会触发重新生成。

## 覆盖范围

示例复现历史手写 POV 应用的 Case 1：

- 四个 PID、四个 Amplifier，每个电机拥有自己的控制器对象。
- 四个 Motor4010、四个 Motor4315，保留地址、总线和调度分频。
- 四个 Swerve_t 配置和采用 PlanarOdom 的底盘。
- RadioMaster_Zorro、UARTBuffer 及解码回调。
- 底盘状态发布、心跳计数及发布、LED 订阅。
- 遥控速度任务通过集成分支的 `make_task` 注册。

`Components/URDF/POVCallbacks.hpp` 提供可复用行为：串口解码转发、心跳、LED、
遥控速度更新。反射不能从参数类型推断业务行为，这些行为必须由 C++ 实现，URDF 负责选择及连接。
`POVInterfaces.hpp` 整理旧头文件的包含依赖顺序，不定义型号到参数类型的映射。

## URDF 中的构造描述

这是 URDF 的 `finemote_cpp` 扩展，并非 ROS 标准标签。普通 link/joint 保留。

```xml
<object id="wheel_BR" type="PID">
  <arg name="params">
    <aggregate>
      <field name="ki" value="0.008f"/>
      <field name="kp" value="0.23f"/>
      <field name="kd" value="0.3f"/>
    </aggregate>
  </arg>
</object>
```

生成器从 PID 的真实构造声明找到 `params` 类型，从该类型的 AST 找到字段顺序，生成：

```cpp
PID wheel_BR{PID_Param_t{0.23f, 0.008f, 0.3f}};
```

`aggregate` 可省略类型，由构造参数推导。类型无法唯一确定时必须明确给出。
函数参数包中的每个聚合值通常需要显式 `type`，例如 `Swerve_t`。

模板实参写成 `template`，名称来自源码：

```xml
<object id="CBRMotor" type="Motor4010">
  <template name="busID" value="1"/>
  <!-- params 聚合、_controller 引用、addr 实参 -->
</object>
```

生成器先读取主模板参数，再让 Clang 实例化 `Motor4010<1>`，读取具体构造接口。
对于 `ReflectionProbe<7>` 这样的新模板，嵌套参数类型 `ReflectionConfig<7>`
也会被实例化后查询；这用于验证配置字段随特化改变及 const 字段初始化。
源模板中默认实参的存在信息会关联到实例化的参数声明，实际默认值仍由目标 C++ 编译器处理。

`ref` 引用对象，`address="true"` 取地址；所有这些引用构成构造依赖图。
`factory` 调用真实工厂函数；嵌套 `call` 用于回调工厂等表达式。
`task` 生成调用对象方法的普通函数及任务注册项。

标量值限定为数值、布尔值、枚举、字符串和 nullptr，不支持嵌入任意 C++ 语句。
`joint="steer_joint_FR" coordinate="x"` 读取该关节 origin 的局部坐标。
本例四个舵向关节均直接连接 base_link；这里没有实现跨多级关节的坐标变换。

## 构造顺序与语义边界

生成器对显式对象引用进行拓扑排序，把依赖对象放在使用者前面。
生成的对象位于同一个翻译单元，以普通全局对象形式依次定义。
XML 对象和字段的书写顺序不影响输出；相互独立的对象按名称确定顺序。
这保证显式依赖的构造次序，不声称复现手写程序所有独立对象的注册先后。
引用缺失和环由生成工具报错。

AST 提供接口信息，C++ 编译器检查最终构造表达式。两者责任分别是：

| 问题 | 当前负责方 |
|---|---|
| XML 格式错误、重复名字、未知字段、无法映射的命名参数 | 生成工具 |
| 模板及构造表达式中的类型不兼容、const 初始化、构造函数体约束 | C++ 编译器 |
| 原生 CAN_Agent 非法总线断言 | C++ 编译器中的 FineMote 原有 static_assert |
| 引用顺序及依赖环 | 生成工具排序和报错 |
| 同一总线重复接收 ID | 原生 FineMote 仍允许编译；本方案不新增禁止规则 |
| 同类型字段的数值含义填反 | 编译器不能自动识别 |

PID 缺省末尾 kd 时输出两项聚合初始化，沿用原生 C++ 的补零规则。
不会通过额外 `make_pid` 改成严格三参数。
聚合的字段按实际声明顺序初始化，因此 const 字段也能设置，避免先构造再赋值。
默认成员初始化值保留在省略的末尾字段中。

## 当前支持边界

这是有限语法的构造映射工具，不是任意 C++ 接口的完全自动反射系统。

- XML 名称与 C++ 类型、模板参数、构造参数、字段名称直接对应；源码改名会改变配置接口。
- 多个构造重载无法仅凭参数名称唯一确定时拒绝生成，交由用户明确接口。
- 支持本例的函数模板、构造函数模板、CTAD 和参数包；不保证任意模板元编程都可直接映射。
- 不展开带基类子对象的复杂聚合、匿名联合或任意容器序列。
- 省略带默认成员初始化器的中间字段而又指定后续字段时，目前要求显式提供该值。
- 自动发现依赖仅涵盖 XML 显式 ref；业务函数隐藏的全局依赖需要显式设计。
- Clang 负责反射，最终 ARM GCC 编译链接负责验证真实目标代码。两种编译器的条件宏或标准库
  可能存在差异；应检查 interfaces.json，并为新增设备补充真实目标构建验证。
- 未进行硬件烧录或闭环运动试验。编译和构造映射验证不等于硬件行为验证。

## 固定验收

`Tests/URDFReflection/verify.py` 在实验开始前冻结，末行输出通过数量。
20 项覆盖固件链接、C++17、原例关键参数、乱序对象与字段、缺省值、错误字段和类型、
非法总线、引用与环、新模板特化、URDF 修改触发重建、以及原生允许的同总线重复 ID。
环境可通过 FINEMOTE_AST_CLANG 和 FINEMOTE_GCC_BIN 指定编译工具路径。

```powershell
py -3 Tests/URDFReflection/verify.py
```

日志和结果写入 `build/reflection-verification`，固定验收不会修改 Git 可见文件。
