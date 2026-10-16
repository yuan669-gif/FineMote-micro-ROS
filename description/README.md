# URDF 示例与提交检查

本次检查原提交 `e8bb932`、`3621547`，基线为 `7afb539`。

## 原提交中的问题

| 级别 | 问题与影响 | 整理结果 |
| --- | --- | --- |
| P1 | `build_config` 未调用 `validate`，未知硬件关节被跳过，重复声明、缺失电机也可能得到 `ok=true` | 接通校验，检查关节引用、唯一声明、控制接口和参数 |
| P1 | 名称、link、joint、参数、电机、舵轮容量不足时静默截断 | 返回明确错误，并补充越界和错误数值用例 |
| P1 | CAN ID 按文本比较，只检查单个 hardware 块；`1` 和 `0x01` 可绕过 | 按数值在同一总线内跨 hardware 检查，保留总线元数据 |
| P1 | 轮半径来自关节 Z 偏移，舵向零位来自底盘到轮心的方位角 | 改为显式半径和标定参数，修正 X/Y 轴距定义 |
| P1 | 机械关节树被当成设备依赖树，但框架还包含总线、控制器、底盘、遥控器等设备；验证函数也无人调用 | 移除 DeviceOrder 封装和全局自动包含；保留机械 joint_order 诊断 |
| P2 | 多个 URDF 共用宏和 include guard；应用目标缺少显式生成依赖；语言标准在应用目标创建之后设置 | 自动发现文件，使用独立符号，补充依赖并提前设置 C++20 |
| P2 | XML 允许多根、无值属性；直接解析入口接受未闭合根；数字前缀或无效文本变成有效数值 | 严格检查支持的 XML/数字子集，减少重复解析路径 |
| P2 | 第二个提交宣称移动旧文件，但最终提交中仍保留 Include/fineurdf、Python 生成器和旧 DeviceOrder | 仅保留 Algorithms/URDF 与 CMake 文本嵌入器 |

## 使用示例

将 `name.urdf` 放入 `description/urdf/`，重新配置或构建后生成
`build/<preset>/finemote_model/name_urdf.hpp`，其中定义 `name_LITERAL`。
文件名须为有效的 C++ 标识符。

`FINEMOTE_URDF_DIR` 可指定描述目录，`FINEMOTE_MODEL_DIR` 可指定生成目录。
`GENERATE_URDF_HEADER=OFF` 使用对应目录中的预生成头文件；缺少文件时配置立即报错。
URDF 内容和生成脚本都是构建依赖，修改后会重新生成头文件并重编译使用它的源文件。

`Components/RobotModel/POVChassisModel.hpp` 提供 `finemote::pov::kConfig`。
存在 pov_chassis 模型时，单独的 `.cpp` 会验证它，不再从全局 ProjectConfig.h 反复包含。
示例使用八个电机和四个舵轮模块，轮直径为 0.0483 米，FR/BR 舵向标定为 180 度。

**当前尚未实现自动创建真实机器人部件。** `TaskPOVChassis.cpp` 仍使用手写对象；
这里没有自动构造 Motor4010/Motor4315、PID/Amplifier 或 POV_Chassis。
继续实现时，需要一个型号与总线适配器，以及真实设备依赖图，不能把机械树直接用作设备注册序。
原示例也与手写任务存在差异：轴距 0.24 与 0.240225、舵向分频 2 与 20，
舵向 PID 描述与实际 Amplifier 等。配置示例不能直接当作硬件等价替换。

## 验证

测试独立于 BSP 和子模块，可以使用本机 C++20 编译器：

```powershell
cmake -S Tests/URDF -B build/urdf-tests -G Ninja -DCMAKE_CXX_COMPILER=<C++20编译器路径>
cmake --build build/urdf-tests
ctest --test-dir build/urdf-tests --output-on-failure
```

Windows 下需把所选编译器的 bin 目录加入 PATH。
测试包含编译期断言与 40 项运行期检查：正常模型、错误输入、容量不足、总线作用域、
非方形底盘、独立标定、多个嵌入头文件及 raw string 定界符冲突。
Windows GNU 主机测试使用 16 MiB 栈以容纳多个定容测试模型；固件使用编译期初始化。

本机验证结果（2026-10-16）：

- GCC 13.1 主机测试通过。
- arm-none-eabi-g++ 13.3：Robomaster_C Debug 固件编译、链接通过；newlib 存在系统调用占位警告。
- armclang 6.24：模型及固件源文件编译通过；完整链接被中文路径下 micro-ROS 静态库加载错误 L6002U 阻断。
- 未进行硬件上电验证，也未宣称解析覆盖完整 URDF 规范。
