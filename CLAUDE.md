# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 ROS2 Humble 的 Jaka ZU5 机械臂控制工作空间，提供仿真和实体的机械臂控制功能。项目集成了 MoveIt 运动规划、Gazebo 仿真环境、以及 PyQt5 图形界面，支持关节运动、末端直线运动、夹爪控制等操作。

## 常用命令

### 构建系统
```bash
# 进入工作空间根目录（不是 src 目录）
cd /home/wtr/programs/cpp/ros2/jaka

# 全量构建
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 选择性构建特定包
colcon build --packages-select jaka_driver jaka_planner user_commands

# 每次构建后环境生效
source install/setup.bash
```

### 测试
```bash
# 运行所有测试
colcon test

# 运行特定包的测试
colcon test --packages-select jaka_driver

# 查看测试结果
cat log/latest_test/stdout.log
```

### 运行指令

#### 仿真模式
```bash
# 启动仿真 + UI（完整仿真环境，包含 Gazebo + RViz + 控制界面）
ros2 launch user_commands sim_ui.launch.py

# 仅启动 RViz（无 Gazebo 仿真，仅用于可视化）
ros2 launch jaka_robot_moveit_config demo.launch.py

# 仅启动 Gazebo + RViz（无 UI 界面）
ros2 launch jaka_robot_moveit_config rviz_gazebo.launch.py
```

#### 实体机械臂模式
```bash
# 1. 启动实体机械臂驱动节点（需配置机器人 IP）
ros2 launch jaka_driver robot_start.launch.py ip:=192.168.1.100

# 2. 启动 MoveIt 规划服务器（新终端）
ros2 launch jaka_planner moveit_server.launch.py

# 3. 启动实体控制 UI（新终端）
ros2 launch user_commands ui.launch.py
```

#### 其他实用命令
```bash
# 关闭机器人（紧急停止）
ros2 run user_commands rob_shutdown

# 测试驱动连接
ros2 run jaka_driver sdk_test
```

## 代码架构

### 包结构及依赖关系

```
jaka_workspace/
├── jaka_msgs/              # 自定义消息定义（所有包的基础依赖）
│   └── msg/
│       ├── RobotMsg.msg    # 机器人状态消息
│       └── MyPoseCmd.msg   # 位姿命令消息
│
├── jaka_description/       # 机器人模型描述
│   ├── urdf/               # URDF/Xacro 文件
│   ├── meshes/             # 3D 模型文件
│   └── config/             # 配置文件
│
├── jaka_driver/            # 底层驱动（C++）
│   ├── src/
│   │   ├── jaka_driver.cpp      # 主驱动节点
│   │   ├── client.cpp           # 网络通信客户端
│   │   └── servoj_demo.cpp      # 伺服关节运动示例
│   ├── include/jaka_driver/     # 头文件
│   ├── lib/libjakaAPI.so        # 厂商 SDK（不要修改）
│   └── launch/
│
├── jaka_planner/           # 运动规划接口（C++）
│   ├── src/                # MoveIt 集成代码
│   └── launch/
│
├── user_commands/          # 用户界面和高级命令（C++ + Python）
│   ├── src/
│   │   ├── moveit_client.cpp    # MoveIt 客户端
│   │   └── rob_shutdown.cpp     # 机器人关闭程序
│   ├── script/
│   │   ├── sim_main_window.py   # 仿真主界面（PyQt5）
│   │   ├── sim_joint_move_ui.py # 仿真关节运动 UI
│   │   ├── sim_linear_move_ui.py# 仿真直线运动 UI
│   │   ├── sim_gripper_ui.py    # 仿真夹爪 UI
│   │   ├── real_main_window.py  # 实体主界面
│   │   ├── real_joint_move.py   # 实体关节运动
│   │   └── real_linear_move.py  # 实体直线运动
│   └── launch/
│       ├── sim_ui.launch.py     # 仿真 UI 启动
│       └── ui.launch.py         # 实体 UI 启动
│
└── jaka_robot_moveit_config/  # MoveIt 配置
    ├── config/              # 运动学、碰撞检测等配置
    └── launch/
        ├── demo.launch.py            # RViz 演示
        ├── rviz_gazebo.launch.py     # Gazebo + RViz
        └── move_group.launch.py      # MoveIt 规划组
```

### 通信架构

```
┌─────────────────┐
│   UI Layer      │  user_commands (Python PyQt5)
│  (用户交互层)    │
└────────┬────────┘
         │
         ↓ Topics/Services
┌─────────────────┐
│ Planning Layer  │  jaka_planner + MoveIt
│  (规划层)        │
└────────┬────────┘
         │
         ↓ Action/Topics
┌─────────────────┐
│  Driver Layer   │  jaka_driver
│  (驱动层)        │
└────────┬────────┘
         │
         ↓ Ethernet
┌─────────────────┐
│ Robot Hardware  │  Jaka ZU5 (192.168.1.100)
└─────────────────┘
```

### 关键设计模式

1. **消息通信**：使用 `jaka_msgs` 包统一定义所有自定义消息
2. **启动时序**：launch 文件使用 `TimerAction` 确保节点按正确顺序启动
3. **仿真/实体分离**：UI 和控制逻辑完全分离，通过不同的 launch 文件启动
4. **MoveIt 集成**：使用 MoveIt Planning Interface 进行高阶运动规划

## 网络配置

### 连接到 Jaka 控制柜
```bash
# 设置以太网接口
IP: 192.168.1.20
Netmask: 255.255.255.0
Gateway: 192.168.1.1

# 机器人默认 IP
Robot IP: 192.168.1.100
```

## 开发规范

### C++ 编码规范
- 语言标准：C++17
- 缩进：4 空格
- 命名约定：
  - 类名：`CamelCase`
  - 函数/变量/话题：`snake_case`
  - 消息文件：`VerbNoun.msg`
- 头文件位置：`include/<package_name>/`

### Python 编码规范
- 使用 rclpy 进行 ROS2 节点开发
- UI 使用 PyQt5
- 脚本位置：`user_commands/script/`
- 安装后的可执行文件在 `install/user_commands/lib/user_commands/`

### 代码风格检查
```bash
# 运行 linter
colcon test --event-handlers console_direct+

# 使用 ament 工具
ament_clang_format
ament_uncrustify
```

## 开发工作流程

1. **修改代码后**：
   ```bash
   colcon build --packages-select <修改的包>
   source install/setup.bash
   ```

2. **测试新功能**：
   - 先在仿真环境验证（使用 `sim_ui.launch.py`）
   - 确认无误后在实体测试（使用 `ui.launch.py`）

3. **添加新的 UI 功能**：
   - 在 `user_commands/script/` 创建新的 Python 脚本
   - 在 `user_commands/CMakeLists.txt` 添加 install 指令
   - 在对应的 launch 文件中添加 Node

## 安全注意事项

1. **实体机器人操作前**：
   - 确认网络配置正确（192.168.1.100）
   - 确认急停按钮可用
   - 先在仿真环境验证代码
   - 检查 `robot_ip`、`safety_level` 参数

2. **代码提交前**：
   - 运行相关测试：`colcon test --packages-select <受影响包>`
   - 不要提交敏感信息（IP 地址、密钥等）
   - 遵循 Conventional Commits 格式：
     - `feat: 添加新功能`
     - `fix: 修复问题`
     - `docs: 文档更新`

3. **配置文件**：
   - 不要修改 `jaka_robot_moveit_config` 中的 MoveIt 自动生成文件
   - 如需修改，先复制一份再修改
   - 敏感配置放在 `.local/` 目录（不提交到 git）

## 故障排查

### 常见问题
1. **找不到包**：检查是否 `source install/setup.bash`
2. **连接失败**：确认机器人 IP 正确，网络连接正常
3. **规划失败**：检查 MoveIt 配置，确认目标位姿可达
4. **UI 无响应**：检查对应的 ROS2 节点是否正常运行

### 调试命令
```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看话题数据
ros2 topic echo /topic_name

# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查节点日志
ros2 run rqt_console rqt_console
```

## 项目现状

根据 [README.md](src/README.md) 和 [AGENTS.md](src/AGENTS.md)：



## 额外资源

- ROS2 Humble 官方文档：https://docs.ros.org/en/humble/
- MoveIt2 教程：https://moveit.picknik.ai/main/
- PyQt5 文档：https://www.riverbankcomputing.com/static/Docs/PyQt5/
