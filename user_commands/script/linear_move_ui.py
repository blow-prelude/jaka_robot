#!/usr/bin/env python3

import sys
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from PyQt5 import QtCore, QtWidgets

from jaka_msgs.srv import Move

INITIAL_POSE: List[float] = [-170.0, 480.0, 700.0, -1.57, 0.0, 0.0]
DEFAULT_STEP = 50
STEP_INCREMENT = 5

AXES = [
    ("x", 0),
    ("y", 1),
    ("z", 2),
    ("rx", 3),
    ("ry", 4),
    ("rz", 5),
]

pose: List[float] = INITIAL_POSE.copy()
step_size = DEFAULT_STEP

node: Optional[Node] = None
linear_move_client = None
pose_value_labels: Dict[str, QtWidgets.QLabel] = {}
step_value_label: Optional[QtWidgets.QLabel] = None


def init_ros():
    """Initialize ROS node and create linear_move client."""
    global node, linear_move_client
    if node is not None:
        return
    if not rclpy.ok():
        rclpy.init(args=None)
    node = rclpy.create_node("linear_move_ui")
    linear_move_client = node.create_client(Move, "/jaka_driver/linear_move")


def shutdown_ros():
    """Shutdown ROS on app close."""
    global node, linear_move_client
    linear_move_client = None
    if node is not None:
        node.destroy_node()
        node = None
    if rclpy.ok():
        rclpy.shutdown()


def update_pose_display():
    """在UI中更新pose"""
    for axis, idx in AXES:
        label = pose_value_labels.get(axis)
        if label is not None:
            label.setText(f"{axis}: {pose[idx]:.4f}")


def update_step_display():
    """在UI中更新步进值"""
    if step_value_label is not None:
        step_value_label.setText(f"step: {step_size:.4f}")


def request_linear_move():
    """向 /jaka_driver/linear_move 发送请求"""
    if node is None:
        init_ros()
    if linear_move_client is None:
        return

    if not linear_move_client.wait_for_service(timeout_sec=0.5):
        node.get_logger().warn("/jaka_driver/linear_move 服务不可用")
        return

    request = Move.Request()
    request.pose = [float(value) for value in pose]
    request.has_ref = False
    request.ref_joint = [0.0]
    request.mvvelo = 100.0
    request.mvacc = 100.0
    request.mvtime = 0.0
    request.mvradii = 0.0
    request.coord_mode = 0
    request.index = 0

    future = linear_move_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if future.done():
        response = future.result()
        if response is None:
            node.get_logger().error("linear_move 调用失败，未获得响应")
            return
        node.get_logger().info(
            f"linear_move ret={response.ret}, message={response.message}"
        )
    else:
        node.get_logger().warn("linear_move 调用超时")


class LinearMoveWindow(QtWidgets.QWidget):
    """该类继承了 QWidget 这个窗口组件"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("linear_move")
        self.setFixedSize(480, 640)
        self._build_ui()

    def _build_ui(self):
        # 创建居中标题
        title = QtWidgets.QLabel("linear_move")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px;")

        # 创建网络布局
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(10)

        pose_value_labels.clear()
        global step_value_label
        step_value_label = None

        for row, (axis, idx) in enumerate(AXES):
            plus_button = QtWidgets.QPushButton(f"+{axis}")
            minus_button = QtWidgets.QPushButton(f"-{axis}")
            self.set_button_size(plus_button,60,40)
            self.set_button_size(minus_button,60,40)

            # 信号槽 
            plus_button.clicked.connect(lambda _, i=idx: self.adjust_pose(i, 1.0))
            minus_button.clicked.connect(lambda _, i=idx: self.adjust_pose(i, -1.0))

            value_label = QtWidgets.QLabel(f"{axis}: {pose[idx]:.4f}")
            pose_value_labels[axis] = value_label
            self.set_button_size(value_label,180,40)

            grid.addWidget(plus_button, row, 0)
            grid.addWidget(minus_button, row, 1)
            grid.addWidget(value_label, row, 2)

        reset_button = QtWidgets.QPushButton("reset")
        reset_button.clicked.connect(self.on_reset)
        # 起始在len(AXES)这一行。0这一列，  1行3列
        grid.addWidget(reset_button, len(AXES), 0, 1, 3)

        plus_step_button = QtWidgets.QPushButton("+step")
        minus_step_button = QtWidgets.QPushButton("-step")
        self.set_button_size(plus_step_button,60,40)
        self.set_button_size(minus_step_button,60,40)
        
        plus_step_button.clicked.connect(self.on_plus_step)
        minus_step_button.clicked.connect(self.on_minus_step)

        step_value_label = QtWidgets.QLabel(f"step: {step_size:.4f}")

        step_row = len(AXES) + 1
        grid.addWidget(plus_step_button, step_row, 0)
        grid.addWidget(minus_step_button, step_row, 1)
        grid.addWidget(step_value_label, step_row, 2)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(title)
        layout.addLayout(grid)
        layout.addStretch(1)
        self.setLayout(layout)

    def adjust_pose(self, index: int, delta: float):
        global pose
        pose[index] += delta
        update_pose_display()
        request_linear_move()
        if node is not None:
            node.get_logger().info(
                f"adjust axis {index} by {delta:.4f}, new value {pose[index]:.4f}"
            )

    def on_reset(self):
        global pose
        pose = INITIAL_POSE.copy()
        update_pose_display()
        request_linear_move()
        if node is not None:
            node.get_logger().info("Reset pose to defaults.")

    def on_plus_step(self):
        global step_size
        step_size += STEP_INCREMENT
        update_step_display()
        if node is not None:
            node.get_logger().info(f"step++, value:{step_size:.4f}")

    def on_minus_step(self):
        global step_size
        step_size = max(STEP_INCREMENT, step_size - STEP_INCREMENT)
        update_step_display()
        if node is not None:
            node.get_logger().info(f"step--, value:{step_size:.4f}")

    def closeEvent(self, event):
        shutdown_ros()
        event.accept()


    def set_button_size(self, button, width, height):
        button.setFixedSize(width, height)


def main():
    init_ros()
    app = QtWidgets.QApplication(sys.argv)
    window = LinearMoveWindow()
    update_pose_display()
    update_step_display()
    window.show()
    app.exec_()
    shutdown_ros()


if __name__ == "__main__":
    main()
