#!/usr/bin/env python3

from PyQt5 import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


from jaka_msgs.msg import MyPoseCmd


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


        # 底部按钮：exec
        button_layout = QtWidgets.QHBoxLayout()
        self.exec_button = QtWidgets.QPushButton("exec")
        self.exec_button.setFixedSize(100, 40)
        button_layout.addStretch(1)
        button_layout.addWidget(self.exec_button)
        button_layout.addStretch(1)

        # 整体纵向布局，底部预留空间给后续控件
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(pose_label)
        layout.addLayout(grid)
        layout.addLayout(button_layout)
        layout.addStretch(1)

        self.setLayout(layout)

        # 连接按钮信号
        self.exec_button.clicked.connect(self.exec_cb)

    # ---------------- ROS 相关 ----------------
    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("joint_move_ui")

        # 创建发布者，用于发布末端位姿命令
        self.pose_cmd_pub = self.node.create_publisher(
            MyPoseCmd,
            "/pose_cmd",
            10
        )

        # 使用 QTimer 定期调用 spin_once，让订阅回调在 Qt 线程中执行
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(50)

    def ros_spin_once(self):
        if self.node is not None and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.0)
            except Exception:
                # 关闭过程中可能抛出异常，忽略
                pass


    def publish_pose_cmd(self):
        """从UI获取末端位姿数据并发布到/pose_cmd话题"""
        # 检查并获取处理后的 TCP 值
        cartesian_pose = self.check_pose_value()
        if cartesian_pose is None:
            return

        msg = MyPoseCmd()
        msg.x = float(cartesian_pose[0]) / 1000.0  # 转换为米
        msg.y = float(cartesian_pose[1]) / 1000.0  # 转换为米
        msg.z = float(cartesian_pose[2]) / 1000.0  # 转换为米
        msg.rx = float(cartesian_pose[3])
        msg.ry = float(cartesian_pose[4])
        msg.rz = float(cartesian_pose[5])
        msg.cartesian_path = False

        self.pose_cmd_pub.publish(msg)
        # 日志中位置按毫米单位输出，方便对照界面
        self.node.get_logger().info(
            f"publish /pose_cmd (mm): x={cartesian_pose[0]:.4f}, y={cartesian_pose[1]:.4f}, z={cartesian_pose[2]:.4f}, "
            f"rx={msg.rx:.4f}, ry={msg.ry:.4f}, rz={msg.rz:.4f}, cartesian_path={msg.cartesian_path}"
        )

    def shutdown_ros(self):
        if self.ros_timer is not None:
            self.ros_timer.stop()

        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def closeEvent(self, event):
        self.shutdown_ros()
        event.accept()


    def exec_cb(self):
        """执行按钮回调函数，发布末端位姿命令"""
        self.check_pose_value()
        self.publish_pose_cmd()

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



