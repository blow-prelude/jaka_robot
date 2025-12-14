#!/usr/bin/env python3
import sys
import math
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtWidgets

from geometry_msgs.msg import TwistStamped
from jaka_msgs.msg import MyPoseCmd


class NewLinearMoveWindow(QtWidgets.QWidget):
    """基于 /tcp_pose 和 /pose_cmd 的笛卡尔控制窗口."""

    AXES = [
        ("x", 0),
        ("y", 1),
        ("z", 2),
        ("rx", 3),
        ("ry", 4),
        ("rz", 5),
    ]

    # 末端执行器 TF 帧名，按当前 URDF/MoveIt 约定使用 Link_6
    EEF_FRAME = "Link_6"
    # 基坐标系，与 moveit_client 中 header.frame_id 保持一致
    BASE_FRAME = "Link_0"

    def __init__(self):
        super().__init__()

        # 位置显示缩放系数：统一按实体机器人，界面用毫米显示，内部用米计算
        self.position_scale: float = 1000.0

        # pose / target_pose 使用 [x, y, z, roll, pitch, yaw]，单位：m, rad
        self.pose: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_pose: List[float] = list(self.pose)

        # 平移步长（作用于 x,y,z）
        self.move_step: float = 0.05
        # 旋转步长（作用于 rx,ry,rz）
        self.rotate_step: float = 0.05
        self.MOVE_STEP_INCREMENT: float = 0.01
        self.ROTATE_STEP_INCREMENT: float = 0.01

        # UI 组件
        self.pose_value_labels: Dict[str, QtWidgets.QLabel] = {}
        self.move_step_label: Optional[QtWidgets.QLabel] = None
        self.rotate_step_label: Optional[QtWidgets.QLabel] = None

        # ROS 相关
        self.node: Optional[Node] = None
        self.pose_cmd_pub = None
        self.tcp_pose_sub = None
        self.ros_timer: Optional[QtCore.QTimer] = None

        self.init_ros()

        # UI 布局
        self.setWindowTitle("new_linear_move")
        self.setFixedSize(520, 680)
        self._build_ui()
        self.update_pose_display()
        self.update_step_display()

    # -------- ROS 初始化与关闭 --------
    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("new_linear_move_ui")

        # 发布 /pose_cmd
        self.pose_cmd_pub = self.node.create_publisher(MyPoseCmd, "pose_cmd", 10)

        # 订阅 /tcp_pose，直接获取末端在基坐标系下的位姿
        self.tcp_pose_sub = self.node.create_subscription(
            TwistStamped, "/tcp_pose", self.tcp_pose_callback, 10
        )

        # 使用 QTimer 定期调用 spin_once，让订阅回调在 Qt 线程中执行
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.on_ros_timer)
        self.ros_timer.start(20)

    def ros_spin_once(self):
        if self.node is not None and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.0)
            except Exception:
                # 关闭过程中可能抛出异常，忽略
                pass

    def on_ros_timer(self):
        # 处理 ROS 回调
        self.ros_spin_once()

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

    # -------- UI 构建 --------
    def _build_ui(self):
        title = QtWidgets.QLabel("new_linear_move (tf -> pose_cmd)")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px;")

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(10)

        # 位姿控制按钮和显示
        for row, (axis, idx) in enumerate(self.AXES):
            plus_btn = QtWidgets.QPushButton(f"+{axis}")
            minus_btn = QtWidgets.QPushButton(f"-{axis}")

            # 位置轴使用 move_step，姿态轴使用 rotate_step
            if idx <= 2:
                plus_btn.clicked.connect(
                    lambda _, i=idx: self.adjust_pose(i, +self.move_step)
                )
                minus_btn.clicked.connect(
                    lambda _, i=idx: self.adjust_pose(i, -self.move_step)
                )
            else:
                plus_btn.clicked.connect(
                    lambda _, i=idx: self.adjust_pose(i, +self.rotate_step)
                )
                minus_btn.clicked.connect(
                    lambda _, i=idx: self.adjust_pose(i, -self.rotate_step)
                )

            plus_btn.setFixedSize(70, 40)
            minus_btn.setFixedSize(70, 40)

            label = QtWidgets.QLabel(f"{axis}: {self.pose[idx]:.4f}")
            label.setFixedSize(220, 40)
            label.setAlignment(QtCore.Qt.AlignCenter)
            self.pose_value_labels[axis] = label

            grid.addWidget(plus_btn, row, 0)
            grid.addWidget(minus_btn, row, 1)
            grid.addWidget(label, row, 2)

        # Reset 按钮：将目标位姿设为当前订阅位姿并发送一次
        reset_btn = QtWidgets.QPushButton("reset_to_current")
        reset_btn.setFixedSize(260, 40)
        reset_btn.clicked.connect(self.on_reset_to_current)
        grid.addWidget(reset_btn, len(self.AXES), 0, 1, 3)

        # 平移步长调节
        plus_move_step = QtWidgets.QPushButton("+move_step")
        minus_move_step = QtWidgets.QPushButton("-move_step")
        plus_move_step.clicked.connect(self.on_plus_move_step)
        minus_move_step.clicked.connect(self.on_minus_move_step)
        plus_move_step.setFixedSize(100, 40)
        minus_move_step.setFixedSize(100, 40)

        self.move_step_label = QtWidgets.QLabel(f"move_step: {self.move_step:.4f}")
        self.move_step_label.setFixedSize(220, 40)
        self.move_step_label.setAlignment(QtCore.Qt.AlignCenter)

        step_row = len(self.AXES) + 1
        grid.addWidget(plus_move_step, step_row, 0)
        grid.addWidget(minus_move_step, step_row, 1)
        grid.addWidget(self.move_step_label, step_row, 2)

        # 旋转步长调节
        plus_rotate_step = QtWidgets.QPushButton("+rotate_step")
        minus_rotate_step = QtWidgets.QPushButton("-rotate_step")
        plus_rotate_step.clicked.connect(self.on_plus_rotate_step)
        minus_rotate_step.clicked.connect(self.on_minus_rotate_step)
        plus_rotate_step.setFixedSize(100, 40)
        minus_rotate_step.setFixedSize(100, 40)

        self.rotate_step_label = QtWidgets.QLabel(
            f"rotate_step: {self.rotate_step:.4f}"
        )
        self.rotate_step_label.setFixedSize(220, 40)
        self.rotate_step_label.setAlignment(QtCore.Qt.AlignCenter)

        rotate_row = step_row + 1
        grid.addWidget(plus_rotate_step, rotate_row, 0)
        grid.addWidget(minus_rotate_step, rotate_row, 1)
        grid.addWidget(self.rotate_step_label, rotate_row, 2)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(title)
        layout.addLayout(grid)
        layout.addStretch(1)
        self.setLayout(layout)

    # -------- UI 更新 --------
    def update_pose_display(self):
        for axis, idx in self.AXES:
            label = self.pose_value_labels.get(axis)
            if label:
                value = self.pose[idx]
                # 对 x,y,z 使用位置缩放（米或毫米），姿态保持弧度
                if idx <= 2:
                    value *= self.position_scale
                label.setText(f"{axis}: {value:.4f}")

    def update_step_display(self):
        if self.move_step_label is not None:
            move_display = self.move_step * self.position_scale
            self.move_step_label.setText(f"move_step: {move_display:.4f}")
        if self.rotate_step_label is not None:
            self.rotate_step_label.setText(f"rotate_step: {self.rotate_step:.4f}")

    # -------- 按钮回调 --------
    def adjust_pose(self, index: int, delta: float):
        # 以当前末端位姿为基础生成目标位姿并发布 /pose_cmd
        self.target_pose = list(self.pose)
        self.target_pose[index] += delta
        self.node.get_logger().info(
            f"adjust axis {index}: delta {delta}, current pose {self.pose}, target pose {self.target_pose}"
        )
        self.publish_pose_cmd()

    def on_reset_to_current(self):
        # 将目标位姿重置为当前末端位姿并发布一次
        self.target_pose = list(self.pose)
        self.node.get_logger().info(
            f"reset target pose to current pose: {self.target_pose}"
        )
        self.publish_pose_cmd()

    def on_plus_move_step(self):
        self.move_step += self.MOVE_STEP_INCREMENT
        self.update_step_display()
        self.node.get_logger().info(f"plus move_step to {self.move_step}")

    def on_minus_move_step(self):
        self.move_step = max(self.MOVE_STEP_INCREMENT, self.move_step - self.MOVE_STEP_INCREMENT)
        self.update_step_display()
        self.node.get_logger().info(f"minus move_step to {self.move_step}")

    def on_plus_rotate_step(self):
        self.rotate_step += self.ROTATE_STEP_INCREMENT
        self.update_step_display()
        self.node.get_logger().info(f"plus rotate_step to {self.rotate_step}")

    def on_minus_rotate_step(self):
        self.rotate_step = max(
            self.ROTATE_STEP_INCREMENT, self.rotate_step - self.ROTATE_STEP_INCREMENT
        )
        self.update_step_display()
        self.node.get_logger().info(f"minus rotate_step to {self.rotate_step}")

    # -------- 从 /tcp_pose 更新当前末端位姿 --------
    def tcp_pose_callback(self, msg: TwistStamped):
        # /tcp_pose 中线速度部分为位置（m），角速度部分为姿态（度）
        x = float(msg.twist.linear.x)
        y = float(msg.twist.linear.y)
        z = float(msg.twist.linear.z)

        rx_deg = float(msg.twist.angular.x)
        ry_deg = float(msg.twist.angular.y)
        rz_deg = float(msg.twist.angular.z)

        rx = math.radians(rx_deg)
        ry = math.radians(ry_deg)
        rz = math.radians(rz_deg)

        self.pose = [x, y, z, rx, ry, rz]
        self.update_pose_display()

    # -------- 发布 /pose_cmd --------
    def publish_pose_cmd(self):
        if self.pose_cmd_pub is None:
            return
        msg = MyPoseCmd()
        # 内部统一用米，发送给 MoveIt 时不做单位切换，仅影响显示
        msg.x = float(self.target_pose[0])
        msg.y = float(self.target_pose[1])
        msg.z = float(self.target_pose[2])
        msg.rx = float(self.target_pose[3])
        msg.ry = float(self.target_pose[4])
        msg.rz = float(self.target_pose[5])
        msg.cartesian_path = True

        self.pose_cmd_pub.publish(msg)
        # 日志中位置按当前显示单位输出，方便对照界面
        unit = "mm"
        log_x = msg.x * self.position_scale
        log_y = msg.y * self.position_scale
        log_z = msg.z * self.position_scale
        self.node.get_logger().info(
            f"publish /pose_cmd ({unit}): x={log_x:.4f}, y={log_y:.4f}, z={log_z:.4f}, "
            f"rx={msg.rx:.4f}, ry={msg.ry:.4f}, rz={msg.rz:.4f}, cartesian_path={msg.cartesian_path}"
        )

    # -------- Qt 关闭事件 --------
    def closeEvent(self, event):
        self.shutdown_ros()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = NewLinearMoveWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
