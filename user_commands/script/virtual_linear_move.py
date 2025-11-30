#!/usr/bin/env python3
from typing import List, Dict
import math

import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtWidgets
from geometry_msgs.msg import TwistStamped
from jaka_msgs.srv import Move


class LinearMoveWindow(QtWidgets.QWidget):
    """独立的 Linear Move 控制窗口，可被 MainWindow 调用"""
    AXES = [
        ("x", 0),
        ("y", 1),
        ("z", 2),
        ("rx", 3),
        ("ry", 4),
        ("rz", 5),
    ]

    def __init__(self):
        super().__init__()

        self.pose: List[float] = [-170.0, 180.0, 700.0, -1.57, 0.0, 0.0]
        self.target_pose: List[float] = []
        self.step_size: float = 50.0
        self.STEP_INCREMENT = 5.0

        # ---- UI 组件存储 ----
        self.pose_value_labels: Dict[str, QtWidgets.QLabel] = {}
        self.step_value_label: QtWidgets.QLabel = None

        # ---- ROS ----
        self.node: Node = None
        self.linear_move_client = None
        self.tool_position_sub = None
        self.ros_timer: QtCore.QTimer = None
        self.init_ros()

        # ---- UI ----
        self.setWindowTitle("linear_move")
        self.setFixedSize(480, 640)
        self._build_ui()
        self.update_pose_display()
        self.update_step_display()


    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("linear_move_ui")
        # 请求 Move 类型的服务
        self.linear_move_client = self.node.create_client(Move, "/jaka_driver/linear_move")
        # 订阅末端实时位姿
        self.tool_position_sub = self.node.create_subscription(
            TwistStamped,
            "/jaka_driver/tool_position",
            self.tool_position_callback,
            10,
        )

        # 使用 QTimer 定期调用 spin_once，让订阅回调在 Qt 线程中执行
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(50)

    def ros_spin_once(self):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown_ros(self):
        if self.ros_timer is not None:
            self.ros_timer.stop()
        if self.node is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



    def _build_ui(self):
        title = QtWidgets.QLabel("linear_move")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px;")

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(10)

        # --- 位姿相关控件 ---
        for row, (axis, idx) in enumerate(self.AXES):
            plus_btn = QtWidgets.QPushButton(f"+{axis}")
            minus_btn = QtWidgets.QPushButton(f"-{axis}")

            plus_btn.clicked.connect(lambda _, i=idx: self.adjust_pose(i, +self.step_size))
            minus_btn.clicked.connect(lambda _, i=idx: self.adjust_pose(i, -self.step_size))

            plus_btn.setFixedSize(60, 40)
            minus_btn.setFixedSize(60, 40)

            label = QtWidgets.QLabel(f"{axis}: {self.pose[idx]:.4f}")
            label.setFixedSize(180, 40)
            label.setAlignment(QtCore.Qt.AlignCenter)
            self.pose_value_labels[axis] = label

            grid.addWidget(plus_btn, row, 0)
            grid.addWidget(minus_btn, row, 1)
            grid.addWidget(label, row, 2)

        # --- Reset ---
        reset_btn = QtWidgets.QPushButton("reset")
        reset_btn.setFixedSize(240, 40)
        reset_btn.clicked.connect(self.on_reset)
        grid.addWidget(reset_btn, len(self.AXES), 0, 1, 3)

        # --- Step 调整 ---
        plus_step = QtWidgets.QPushButton("+step")
        minus_step = QtWidgets.QPushButton("-step")

        plus_step.clicked.connect(self.on_plus_step)
        minus_step.clicked.connect(self.on_minus_step)

        plus_step.setFixedSize(60, 40)
        minus_step.setFixedSize(60, 40)

        self.step_value_label = QtWidgets.QLabel(f"step: {self.step_size:.4f}")
        self.step_value_label.setFixedSize(180, 40)
        self.step_value_label.setAlignment(QtCore.Qt.AlignCenter)

        step_row = len(self.AXES) + 1
        grid.addWidget(plus_step, step_row, 0)
        grid.addWidget(minus_step, step_row, 1)
        grid.addWidget(self.step_value_label, step_row, 2)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(title)
        layout.addLayout(grid)
        layout.addStretch(1)
        self.setLayout(layout)



    def update_pose_display(self):
        for axis, idx in self.AXES:
            label = self.pose_value_labels.get(axis)
            if label:
                label.setText(f"{axis}: {self.pose[idx]:.4f}")

    def update_step_display(self):
        self.step_value_label.setText(f"step: {self.step_size:.4f}")

    def adjust_pose(self, index: int, delta: float):
        # 以当前订阅到的实际位姿为基础生成目标位姿，
        # 这样在只调整 x/y/z 时不会无意改变当前姿态。
        self.target_pose = list(self.pose)
        self.target_pose[index] += delta
        self.node.get_logger().debug(
            f"current pose: {self.pose},  target pose: {self.target_pose}"
        )
        self.request_linear_move()
        self.node.get_logger().info(
            f"move axis {index}: delta {delta}, toward {self.target_pose[index]}"
        )

    def on_reset(self):
        # 重置为初始位姿，同时保持与 pose 的表示方式一致（mm, rad）
        self.target_pose = [-170.0, 480.0, 700.0, -1.57, 0.0, 0.0]
        self.request_linear_move()
        self.node.get_logger().info(f"reset ,move to {self.target_pose}")

    def on_plus_step(self):
        self.step_size += self.STEP_INCREMENT
        self.update_step_display()
        self.node.get_logger().info(f"plus move step to {self.step_size}")

    def on_minus_step(self):
        self.step_size = max(self.STEP_INCREMENT, self.step_size - self.STEP_INCREMENT)
        self.update_step_display()
        self.node.get_logger().info(f"minus move step to {self.step_size}")


    def tool_position_callback(self, msg: TwistStamped):
        # 将订阅的末端位姿转换为内部表示（平移 mm，姿态 rad）
        x = msg.twist.linear.x
        y = msg.twist.linear.y
        z = msg.twist.linear.z
        rx = math.radians(msg.twist.angular.x)
        ry = math.radians(msg.twist.angular.y)
        rz = math.radians(msg.twist.angular.z)

        self.pose = [x, y, z, rx, ry, rz]
        self.update_pose_display()

    # 请求服务
    def request_linear_move(self):
        if not self.linear_move_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warn("linear_move service unavailable")
            return

        req = Move.Request()
        # 直接将目标位姿以 [x, y, z, rx, ry, rz] 形式发送，
        # 与 jaka_driver 中 linear_move_callback 的 RPY 约定一致。
        req.pose = list(map(float, self.target_pose))
        self.node.get_logger().debug(f"req pose: {req.pose}")
        req.has_ref = False
        req.ref_joint = [0.0]
        req.mvvelo = 300.0
        req.mvacc = 400.0
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0

        future = self.linear_move_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.done():
            response = future.result()
            if response is None:
                self.node.get_logger().error("linear_move 调用失败，未获得响应")
                return
            if response.ret == 0:
                self.node.get_logger().info(
                    f"linear_move 成功，ret={response.ret}, message={response.message}"
                )
            else:
                self.node.get_logger().warn(
                    f"linear_move 失败，ret={response.ret}, message={response.message}"
                )
        else:
            self.node.get_logger().warn("linear_move 调用超时")


    def closeEvent(self, event):
        self.shutdown_ros()
        event.accept()
