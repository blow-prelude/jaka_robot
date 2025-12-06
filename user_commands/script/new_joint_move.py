#!/usr/bin/env python3

import sys
from typing import Dict, List, Optional

from PyQt5 import QtCore, QtWidgets

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from jaka_msgs.msg import MyPoseCmd


class NewJointMoveWindow(QtWidgets.QWidget):
    """基于 /pose_cmd 的笛卡尔位姿输入窗口（cartesian_path = False）。"""

    def __init__(self):
        super().__init__()

        self.setWindowTitle("new_joint_move")
        self.setFixedSize(480, 360)

        # 存储每个轴对应的输入框
        self.pose_inputs: Dict[str, QtWidgets.QLineEdit] = {}

        # 当前关节角（rad），以及对应的显示标签
        self.joint_positions: List[float] = [0.0] * 6
        self.joint_value_labels: Dict[int, QtWidgets.QLabel] = {}

        # ROS 相关
        self.node: Optional[Node] = None
        self.pose_cmd_pub = None
        self.joint_sub = None
        self.ros_timer: Optional[QtCore.QTimer] = None

        self._build_ui()
        self.init_ros()

    def _build_ui(self):
        # 标题
        title = QtWidgets.QLabel("new_joint_move")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")

        # pose 说明标签
        pose_label = QtWidgets.QLabel("pose (x, y, z, rx, ry, rz)")
        pose_label.setAlignment(QtCore.Qt.AlignLeft)
        pose_label.setStyleSheet("font-size: 14px;")

        # 2 行 3 列的网格布局：x,y,z,rx,ry,rz
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(10)

        axes = ["x", "y", "z", "rx", "ry", "rz"]
        for idx, axis in enumerate(axes):
            row = idx // 3
            col = idx % 3

            lbl = QtWidgets.QLabel(axis)
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setFixedSize(60, 40)

            edit = QtWidgets.QLineEdit()
            edit.setPlaceholderText(axis)
            edit.setFixedHeight(30)

            cell_layout = QtWidgets.QHBoxLayout()
            cell_layout.addWidget(lbl)
            cell_layout.addWidget(edit)

            grid.addLayout(cell_layout, row, col)
            self.pose_inputs[axis] = edit

        # 当前关节角显示
        joint_title = QtWidgets.QLabel("current joint (rad)")
        joint_title.setAlignment(QtCore.Qt.AlignLeft)
        joint_title.setStyleSheet("font-size: 14px;")

        joint_grid = QtWidgets.QGridLayout()
        joint_grid.setSpacing(10)
        for i in range(6):
            row = i // 3
            col = i % 3
            lbl = QtWidgets.QLabel(f"j{i + 1}: {self.joint_positions[i]:.4f}")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setFixedSize(120, 30)
            joint_grid.addWidget(lbl, row, col)
            self.joint_value_labels[i] = lbl

        # 底部按钮：只有 exec
        button_layout = QtWidgets.QHBoxLayout()
        self.exec_button = QtWidgets.QPushButton("exec")
        self.exec_button.setFixedSize(100, 40)
        button_layout.addStretch(1)
        button_layout.addWidget(self.exec_button)
        button_layout.addStretch(1)

        # 整体纵向布局
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(pose_label)
        layout.addLayout(grid)
        layout.addWidget(joint_title)
        layout.addLayout(joint_grid)
        layout.addLayout(button_layout)
        layout.addStretch(1)

        self.setLayout(layout)

        # 连接按钮信号
        self.exec_button.clicked.connect(self.exec_cb)

    # ---------------- ROS 相关 ----------------
    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("new_joint_move_ui")
        self.pose_cmd_pub = self.node.create_publisher(MyPoseCmd, "pose_cmd", 10)

        # 订阅 /joint_states 获取当前关节角
        self.joint_sub = self.node.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # 使用 QTimer 定期 spin_once，让订阅回调在 Qt 线程中执行
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.on_ros_timer)
        self.ros_timer.start(20)

    def on_ros_timer(self):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.0)

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

    # ---------------- 关节角显示相关 ----------------
    def joint_states_callback(self, msg: JointState):
        # 优先根据关节名 joint_1 ... joint_6 进行映射
        positions: List[float] = [0.0] * 6
        if msg.name:
            name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
            for i in range(6):
                positions[i] = float(name_to_pos.get(f"joint_{i + 1}", 0.0))
        else:
            for i in range(min(6, len(msg.position))):
                positions[i] = float(msg.position[i])

        self.joint_positions = positions
        self.update_joint_display()

    def update_joint_display(self):
        for i in range(6):
            lbl = self.joint_value_labels.get(i)
            if lbl is not None:
                lbl.setText(f"j{i + 1}: {self.joint_positions[i]:.4f}")

    # ---------------- 按钮回调 ----------------
    def exec_cb(self):
        if self.pose_cmd_pub is None or self.node is None:
            return

        cartesian_pose = self.check_pose_value()
        if cartesian_pose is None:
            return

        x, y, z, rx, ry, rz = cartesian_pose

        msg = MyPoseCmd()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.rx = float(rx)
        msg.ry = float(ry)
        msg.rz = float(rz)
        msg.cartesian_path = False

        self.pose_cmd_pub.publish(msg)
        self.node.get_logger().info(
            f"Publish /pose_cmd: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}, "
            f"rx={msg.rx:.4f}, ry={msg.ry:.4f}, rz={msg.rz:.4f}, cartesian_path={msg.cartesian_path}"
        )

    def check_pose_value(self) -> Optional[List[float]]:
        """检查输入的 TCP 是否在合理范围，并进行取余归一化。

        约定：
        - 界面中 x,y,z 以毫米填写，范围约 [-900, 900]、[-350, 900]；
        - 在发布到 /pose_cmd 前，会自动转换为米（除以 1000）；
        - rx, ry, rz 以弧度填写，范围 [-3.14, 3.14]。
        """

        axes = ["x", "y", "z", "rx", "ry", "rz"]
        raw_values: List[float] = []

        # 读取并转换输入，如果为空或非数字则报错
        for axis in axes:
            edit = self.pose_inputs.get(axis)
            text = edit.text().strip() if edit is not None else ""
            try:
                value = float(text)
            except (TypeError, ValueError):
                if self.node is not None:
                    self.node.get_logger().warn(f"invalid value for {axis}: '{text}'")
                return None
            raw_values.append(value)

        x, y, z, rx, ry, rz = raw_values

        # 限制范围并做取余
        def clamp_mod(val: float, min_v: float, max_v: float) -> float:
            span = max_v - min_v
            if span <= 0.0:
                return min_v
            return min_v + ((val - min_v) % span)

        x = clamp_mod(x, -900.0, 900.0)
        y = clamp_mod(y, -900.0, 900.0)
        z = clamp_mod(z, -350.0, 900.0)
        rx = clamp_mod(rx, -3.14, 3.14)
        ry = clamp_mod(ry, -3.14, 3.14)
        rz = clamp_mod(rz, -3.14, 3.14)

        # 将毫米转换为米
        processed = [x / 1000.0, y / 1000.0, z / 1000.0, rx, ry, rz]
        if self.node is not None:
            self.node.get_logger().info(f"checked cartesian pose: {processed}")

        return processed


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = NewJointMoveWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
