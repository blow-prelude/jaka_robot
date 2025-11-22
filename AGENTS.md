# Repository Guidelines

## 项目结构与模块组织
本 ROS2 工作区内容集中在 `src/` 下的多个包。`jaka_driver` 提供底层驱动、控制器插件以及示例节点；`jaka_description`、`jaka_zu5_moveit_config` 与 `my_jaka_zu5_moveit_config` 保持 URDF、MoveIt 和 xacro 参数；`jaka_msgs` 维护自定义消息接口；`jaka_planner` 汇集规划样例与工具；`user_commands` 存放示教脚本与数据。构建产物位于根目录下的 `build/`、`install/` 与 `log/`，无需手动修改这些目录。

## 构建、测试与开发命令
在工作区根目录执行 `source /opt/ros/humble/setup.bash && colcon build --symlink-install` 进行全量构建；如需缩小范围，可添加 `--packages-select jaka_driver jaka_planner`。使用 `colcon test` 或 `colcon test --packages-select jaka_driver` 运行单包测试。调试前运行 `source install/setup.bash`，再用 `ros2 launch jaka_driver bringup.launch.py robot_ip:=<地址>` 或 `ros2 launch jaka_planner demo.launch.py` 启动控制与规划；示教脚本可通过 `ros2 run jaka_driver user_command_player` 调用。

## 编码风格与命名约定
C++ 源码采用 C++17 与 `ament_cmake`，统一四空格缩进。类名使用 `CamelCase`，函数、主题与帧名保持 `snake_case`，消息文件以 `VerbNoun.msg` 格式命名。头文件统一放入 `include/<pkg>/`；公共常量集中在 `include/.../constants.hpp`。提交前运行 `ament_clang_format` 或 `ament_uncrustify`（可通过 `colcon test --event-handlers console_direct+` 自动触发）以保证风格一致。

## 测试指引
测试主要依赖 GTest 与 launch testing；每个功能新增 `test_<feature>.cpp` 或 `.py` 并置于包内 `test/`。测试需覆盖驱动连接超时、轨迹边界检查与 MoveIt 规划失败回退。修改消息或控制接口时应补充仿真回归，利用 `ros2 launch my_jaka_zu5_moveit_config demo.launch.py` 验证。提交前至少运行一次 `colcon test --packages-select <受影响包>` 并检查 `log/latest_test` 报告。

## 提交与合并请求指南
当前分发版本未附带 Git 历史，可参照 Conventional Commits（如 `feat: add torque limit guard`、`fix: resolve planner joint bounds`）。提交信息需简述动机与影响。PR 描述中说明复现步骤、测试结果及截图或终端日志；若关联 Issue，请在描述末尾添加 `Fixes #<id>`。涉及硬件或网络配置的更改需记录在 `user_commands/README` 或对应 launch 文件中，并附安全提示。

## 安全与配置提示
在真实机器人上运行前，确认 `robot_ip`、`safety_level` 与急停状态在参数 YAML 或 launch 文件中一致，避免在未接管网段发送运动命令。仿真或离线验证建议先复制 `my_jaka_zu5_moveit_config` 再修改，防止覆盖官方 MoveIt 包。私钥、证书以及厂商固件不要提交到仓库，可放入 `.local/` 并在文档中引用。
