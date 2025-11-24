#!/usr/bin/env python3
from typing import List, Dict

import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtWidgets
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

        self.pose: List[float] = [-170.0, 480.0, 700.0, -1.57, 0.0, 0.0]
        self.step_size: float = 50.0
        self.STEP_INCREMENT = 5.0

        # ---- UI 组件存储 ----
        self.pose_value_labels: Dict[str, QtWidgets.QLabel] = {}
        self.step_value_label: QtWidgets.QLabel = None

        # ---- ROS ----
        self.node: Node = None
        self.linear_move_client = None
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

    def shutdown_ros(self):
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
        self.pose[index] += delta
        self.update_pose_display()
        self.request_linear_move()
        self.node.get_logger().info(
            f"move axis {index}: delta {delta}, now {self.pose[index]}"
        )

    def on_reset(self):
        self.pose = [-170.0, 480.0, 700.0, -1.57, 0.0, 0.0]
        self.update_pose_display()
        self.request_linear_move()
        self.node.get_logger().info(f"reset ,move to {self.pose}")

    def on_plus_step(self):
        self.step_size += self.STEP_INCREMENT
        self.update_step_display()
        self.node.get_logger().info(f"plus move step to {self.step_size}")

    def on_minus_step(self):
        self.step_size = max(self.STEP_INCREMENT, self.step_size - self.STEP_INCREMENT)
        self.update_step_display()
        self.node.get_logger().info(f"minus move step to {self.step_size}")


    # 请求服务
    def request_linear_move(self):
        if not self.linear_move_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warn("linear_move service unavailable")
            return

        req = Move.Request()
        req.pose = list(map(float, self.pose))
        req.has_ref = False
        req.ref_joint = [0.0]
        req.mvvelo = 300.0
        req.mvacc = 400.0
        req.mvtime = 0.0
        req.mvradii = 0.0
        req.coord_mode = 0
        req.index = 0

        future = self.linear_move_client.call_async(req)
        # TODO 根据 future.result() 判断运动是否成功 等等
        rclpy.spin_until_future_complete(self.node, future)


    def closeEvent(self, event):
        self.shutdown_ros()
        event.accept()
