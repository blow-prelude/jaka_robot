#!/usr/bin/env python3

from PyQt5 import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from jaka_msgs.srv import Move , GetIK


class JointMoveWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("joint_move")
        self.setFixedSize(480, 640)

        # 存储每个轴对应的输入框
        self.pose_inputs = {}

        # 关节名称与当前关节角
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joint_positions = [0.0] * len(self.joint_names)


        # ROS 相关
        self.node = None
        self.joint_state_sub = None
        self.ros_timer = None

        self._build_ui()
        self.init_ros()

    def _build_ui(self):
        # 标题
        title = QtWidgets.QLabel("joint_move")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")

        # pose 说明标签
        pose_label = QtWidgets.QLabel("pose")
        pose_label.setAlignment(QtCore.Qt.AlignLeft)
        pose_label.setStyleSheet("font-size: 14px;")

        # 2 行 3 列的网格布局：x,y,z,rx,ry,rz
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(10)

        axes = ["x", "y", "z", "rx", "ry", "rz"]
        for idx, axis in enumerate(axes):
            row = idx // 3
            col = idx % 3

            # 每个文本框前的标签
            lbl = QtWidgets.QLabel(axis)
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setFixedSize(60, 40)

            edit = QtWidgets.QLineEdit()
            edit.setPlaceholderText(axis)
            edit.setFixedHeight(30)

            # 水平布局： [label][line edit]
            cell_layout = QtWidgets.QHBoxLayout()
            cell_layout.addWidget(lbl)
            cell_layout.addWidget(edit)

            grid.addLayout(cell_layout, row, col)
            self.pose_inputs[axis] = edit

        # joint_value 区域：3 行 2 列的固定标签 joint1~joint6
        joint_label = QtWidgets.QLabel("joint_value")
        joint_label.setAlignment(QtCore.Qt.AlignLeft)
        joint_label.setStyleSheet("font-size: 14px;")

        joint_grid = QtWidgets.QGridLayout()
        joint_grid.setSpacing(10)

        self.joint_value_labels = {}
        for idx, name in enumerate(self.joint_names):
            row = idx // 2
            col = idx % 2

            name_label = QtWidgets.QLabel(name)
            name_label.setAlignment(QtCore.Qt.AlignCenter)
            name_label.setFixedSize(60, 40)

            value_label = QtWidgets.QLabel("")
            value_label.setAlignment(QtCore.Qt.AlignCenter)

            cell_layout = QtWidgets.QHBoxLayout()
            cell_layout.addWidget(name_label)
            cell_layout.addWidget(value_label)

            joint_grid.addLayout(cell_layout, row, col)
            self.joint_value_labels[name] = value_label
        # current_joint_value 区域：实时显示订阅到的关节角
        current_joint_label = QtWidgets.QLabel("current_joint_value")
        current_joint_label.setAlignment(QtCore.Qt.AlignLeft)
        current_joint_label.setStyleSheet("font-size: 14px;")

        current_joint_grid = QtWidgets.QGridLayout()
        current_joint_grid.setSpacing(10)

        self.current_joint_value_labels = {}
        for idx, name in enumerate(self.joint_names):
            row = idx // 2
            col = idx % 2

            name_label = QtWidgets.QLabel(name)
            name_label.setAlignment(QtCore.Qt.AlignCenter)
            name_label.setFixedSize(60, 40)

            value_label = QtWidgets.QLabel("")
            value_label.setAlignment(QtCore.Qt.AlignCenter)

            cell_layout = QtWidgets.QHBoxLayout()
            cell_layout.addWidget(name_label)
            cell_layout.addWidget(value_label)

            current_joint_grid.addLayout(cell_layout, row, col)
            self.current_joint_value_labels[name] = value_label

        # 底部按钮：get IK 和 exec
        button_layout = QtWidgets.QHBoxLayout()
        self.get_ik_button = QtWidgets.QPushButton("get IK")
        self.exec_button = QtWidgets.QPushButton("exec")
        self.get_ik_button.setFixedSize(100, 40)
        self.exec_button.setFixedSize(100, 40)
        button_layout.addStretch(1)
        button_layout.addWidget(self.get_ik_button)
        button_layout.addWidget(self.exec_button)
        button_layout.addStretch(1)

        # 整体纵向布局，底部预留空间给后续控件
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(pose_label)
        layout.addLayout(grid)
        layout.addWidget(joint_label)
        layout.addLayout(joint_grid)
        layout.addWidget(current_joint_label)
        layout.addLayout(current_joint_grid)
        layout.addLayout(button_layout)
        layout.addStretch(1)

        self.setLayout(layout)

        # 连接按钮信号
        self.get_ik_button.clicked.connect(self.get_ik_cb)
        self.exec_button.clicked.connect(self.exec_cb)

    # ---------------- ROS 相关 ----------------
    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("joint_move_ui")
        # 订阅话题/jaka_driver/joint_position ，得到各个关节的角度
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "/jaka_driver/joint_position",
            self.joint_state_callback,
            10,
        )

        # 创建 客户端，求运动学逆解
        self.get_ik_client = self.node.create_client(GetIK,"/jaka_driver/get_ik")
        # 创建关节运动客户端，发送关节运动请求
        self.joint_move_client = self.node.create_client(Move,"/jaka_driver/joint_move")

        # 使用 QTimer 定期调用 spin_once，让订阅回调在 Qt 线程中执行
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(50)

    def ros_spin_once(self):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.0)


    def update_current_joint_display(self):
        for i, name in enumerate(self.joint_names):
            value_label = self.current_joint_value_labels.get(name)
            if value_label is not None:
                value_label.setText(f"{self.current_joint_positions[i]:.4f}")

    def shutdown_ros(self):
        if self.ros_timer is not None:
            self.ros_timer.stop()

        if self.node is not None:
            self.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    def closeEvent(self, event):
        self.shutdown_ros()
        event.accept()


    def get_ik_cb(self):
        if not self.get_ik_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warn("get_ik service unavailable")
            return

        # 检查并获取处理后的 TCP 值
        cartesian_pose = self.check_pose_value()
        if cartesian_pose is None:
            return

        req = GetIK.Request()
        req.ref_joint = list(map(float, self.current_joint_positions))
        req.cartesian_pose = cartesian_pose

        future = self.get_ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            self.node.get_logger().warn("get_ik call failed")
            return

        res = future.result()
        # 将返回的 joint 保存下来，供 exec 使用
        self.last_ik_result = list(res.joint)
        # 在 joint_value 区域显示这组解
        for i, name in enumerate(self.joint_names):
            if i < len(self.last_ik_result):
                label = self.joint_value_labels.get(name)
                if label is not None:
                    label.setText(f"{self.last_ik_result[i]:.4f}")



    def exec_cb(self):
        if not self.joint_move_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warn("joint_move service unavailable")
            return

        if not hasattr(self, "last_ik_result"):
            self.node.get_logger().warn("no IK result available, please click get IK first")
            return

        req = Move.Request()
        req.pose = list(self.last_ik_result)
        req.has_ref = True
        req.ref_joint = list(map(float, self.current_joint_positions))
        req.mvvelo = 1.5
        req.mvacc = 2.5
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0

        # 请求服务
        future = self.joint_move_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            self.node.get_logger().warn("joint_move call failed")

    def check_pose_value(self):
        """该函数负责检查输入的TCP是否合法，x,y需要在-900到900之间，z需要在-350到900之间
        rx,ry,rz需要在-3.14到3.14之间
        如果不在区间内，就用填写的数值对允许区间最大值取余，使其落在该区间内"""

        axes = ["x", "y", "z", "rx", "ry", "rz"]
        raw_values = []

        # 读取并转换输入，如果为空或非数字则报错
        for axis in axes:
            edit = self.pose_inputs.get(axis)
            text = edit.text().strip() if edit is not None else ""
            try:
                value = float(text)
            except (TypeError, ValueError):
                self.node.get_logger().warn(f"invalid value for {axis}: '{text}'")
                return None
            raw_values.append(value)

        x, y, z, rx, ry, rz = raw_values

        # 限制范围并做取余
        def clamp_mod(val, min_v, max_v):
            span = max_v - min_v
            if span <= 0:
                return min_v
            # 将值映射到 [min_v, max_v) 区间
            return min_v + ((val - min_v) % span)

        x = clamp_mod(x, -900.0, 900.0)
        y = clamp_mod(y, -900.0, 900.0)
        z = clamp_mod(z, -350.0, 900.0)
        rx = clamp_mod(rx, -3.14, 3.14)
        ry = clamp_mod(ry, -3.14, 3.14)
        rz = clamp_mod(rz, -3.14, 3.14)

        processed = [x, y, z, rx, ry, rz]
        # 输出处理后的数值日志
        self.node.get_logger().info(
            f"checked cartesian pose: {processed}"
        )

        return processed


    def joint_state_callback(self, msg: JointState):
        # 将数据保存到变量中
        for i, name in enumerate(self.joint_names):
            if i < len(msg.position):
                self.current_joint_positions[i] = msg.position[i]

        # 更新 UI 显示
        self.update_current_joint_display()

