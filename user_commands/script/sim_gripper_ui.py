#!/usr/bin/env python3

import sys
import signal

from PyQt5 import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class GripperWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("gripper")
        self.setFixedSize(480, 640)

        # 当前夹爪开合位置 [m]，范围 [0.0, 0.04]
        self.position_value = 0.0

        # ROS 相关
        self.node = None
        self.pub = None
        self.ros_timer = None

        self._init_ros()
        self._build_ui()
        self._update_position_label()

    # ---------- ROS ----------
    def _init_ros(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("sim_gripper_ui")
        self.pub = self.node.create_publisher(Float64MultiArray, "gripper_cmd", 10)

        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self._on_ros_timer)
        self.ros_timer.start(20)

    def _on_ros_timer(self):
        if self.node is not None and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.0)
            except Exception:
                # 在关闭过程中可能会抛出异常，忽略即可
                pass

    def _shutdown_ros(self):
        if self.ros_timer is not None:
            self.ros_timer.stop()
            self.ros_timer = None
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                self.node.get_logger().error("sth went wrong when try to shutdown ros node")
            self.node = None
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                print('[ERROR] sth went wrong when try to shutdown rclpy')

    # ---------- UI ----------
    def _build_ui(self):
        main_layout = QtWidgets.QVBoxLayout()

        # 顶部：滑动条 + 数值显示
        slider_layout = QtWidgets.QHBoxLayout()

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        # 使用 0 ~ 40，对应 0.000 ~ 0.040 m，步长 0.001
        self.slider.setMinimum(0)
        self.slider.setMaximum(40)
        self.slider.setSingleStep(1)
        self.slider.setValue(0)
        self.slider.valueChanged.connect(self._on_slider_changed)

        self.position_label = QtWidgets.QLabel("0.0000")
        self.position_label.setFixedWidth(80)
        self.position_label.setAlignment(QtCore.Qt.AlignCenter)

        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(self.position_label)

        # 中间：+ / - 按钮
        button_row = QtWidgets.QHBoxLayout()

        self.plus_button = QtWidgets.QPushButton("+")
        self.minus_button = QtWidgets.QPushButton("-")
        self.plus_button.setFixedSize(80, 40)
        self.minus_button.setFixedSize(80, 40)

        self.plus_button.clicked.connect(self._on_plus_clicked)
        self.minus_button.clicked.connect(self._on_minus_clicked)

        button_row.addStretch(1)
        button_row.addWidget(self.minus_button)
        button_row.addWidget(self.plus_button)
        button_row.addStretch(1)

        # 底部：exec 按钮
        exec_row = QtWidgets.QHBoxLayout()
        self.exec_button = QtWidgets.QPushButton("exec")
        self.exec_button.setFixedSize(100, 40)
        self.exec_button.clicked.connect(self._on_exec_clicked)
        exec_row.addStretch(1)
        exec_row.addWidget(self.exec_button)
        exec_row.addStretch(1)

        # 组合整体布局
        main_layout.addStretch(1)
        main_layout.addLayout(slider_layout)
        main_layout.addSpacing(40)
        main_layout.addLayout(button_row)
        main_layout.addStretch(2)
        main_layout.addLayout(exec_row)
        main_layout.addStretch(1)

        self.setLayout(main_layout)

    # ---------- 事件处理 ----------
    def _on_slider_changed(self, value: int):
        # slider: 0 ~ 40 -> 0.000 ~ 0.040 m
        self.position_value = float(value) / 1000.0
        self._update_position_label()

    def _on_plus_clicked(self):
        value = self.slider.value()
        if value < self.slider.maximum():
            self.slider.setValue(value + self.slider.singleStep())

    def _on_minus_clicked(self):
        value = self.slider.value()
        if value > self.slider.minimum():
            self.slider.setValue(value - self.slider.singleStep())

    def _on_exec_clicked(self):
        if self.pub is None or self.node is None or not rclpy.ok():
            return
        left_pos = float(self.position_value)
        right_pos = -left_pos
        msg = Float64MultiArray()
        msg.data = [left_pos, right_pos]
        self.pub.publish(msg)
        self.node.get_logger().info(
            f"Publish /gripper_cmd: left={left_pos:.4f} m, right={right_pos:.4f} m"
        )

    def _update_position_label(self):
        self.position_label.setText(f"{self.position_value:.4f}")

    def closeEvent(self, event):
        self._shutdown_ros()
        event.accept()


def _install_sigint_handler(app: QtWidgets.QApplication):
    def handler(signum, frame):
        app.quit()
    signal.signal(signal.SIGINT, handler)


def main():
    app = QtWidgets.QApplication(sys.argv)
    _install_sigint_handler(app)
    win = GripperWindow()
    win.show()
    app.exec_()


if __name__ == "__main__":
    main()
