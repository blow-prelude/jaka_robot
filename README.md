### 如何连接到控制柜
1. 用网线连接电脑和控制柜

2. 设置以太网
    - ip  192.168.1.20
    - 子网掩码  255.255.255.0
    - 网关   192.168.1.1

3. 输入机器人ip  192.168.1.100



### 更新日志
#### 2026-4-9   v0.1
- 使用 jaka_driver 功能包 和 jaka_planner 功能包 分别实现了仿真、实体的关节运动、末端直线运动

###### 仿真模式
```
启动仿真 + UI（完整仿真环境，包含 Gazebo + RViz + 控制界面）
ros2 launch user_commands sim_ui.launch.py
```

###### 实体机械臂模式
```bash
# 1. 启动实体机械臂驱动节点
ros2 launch jaka_driver robot_start.launch.py ip:=192.168.1.100

# 2. 启动 MoveIt 规划服务器
ros2 launch jaka_planner moveit_server.launch.py

# 3. 启动实体控制 UI
ros2 launch user_commands ui.launch.py
```
