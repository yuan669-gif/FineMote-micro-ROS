# FineMote：从机器人描述构建固件

在 `integration/all` 中，AST 是可选的 MC_Board 固件入口：构建主机读取 URDF 和控制器 YAML，用 Clang AST 核对 C++ 构造接口，再生成并编译 C++17 设备代码。MCU 不解析 XML/YAML。

## 构建示例

准备 ARM GCC、Ninja、CMake（预设格式 v5 需要 3.24+）、支持 JSON AST 的 Clang，以及安装了 PyYAML 的 Python 3.9+。在仓库根目录执行：

```powershell
cmake --preset ast-MC_Board-release
cmake --build build/ast-MC_Board-release
```

调试配置为 `ast-MC_Board-debug`。先运行 `git submodule update --init --recursive`，包括根预设引用的 Esp32s3。AST 预设只启用此 MC_Board 应用需要的组件，不链接 BMS、电路板示例任务和用户任务，并复用板级 UART5 micro-ROS 后端。其他板级预设不默认绑定示例机器人。

- [URDF](Interface/Examples/URDF/pov_simple.urdf) 和 [控制器 YAML](Interface/Examples/URDF/pov_controllers.yaml)：主线示例输入。
- [设备构建与验收](Interface/Examples/URDF/DEVICE_ASSEMBLY.md)：自定义输入、生成物、话题、失联保护和测试。
- [AST 后端说明](Interface/Examples/URDF/URDF_AST_MAPPING.md)：显式构造图示例及反射回归，不是第二种解析器。
- 旧 constexpr 解析器、静态装配、测试和说明保存在本地分支 `codex/micro-ROS-constexpr`。无需切换分支即可查看旧说明：`git show codex/micro-ROS-constexpr:Interface/Examples/URDF/URDF_FIRMWARE.md`。
