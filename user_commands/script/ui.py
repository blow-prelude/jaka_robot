#!/usr/bin/env python3

import tkinter as tk
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from jaka_msgs.srv import Move

INITIAL_POSE: List[float] = [-170.0, 480.0, 700.0, -1.57 ,0.0, 0.0]
DEFAULT_STEP = 10
STEP_INCREMENT = 1

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
pose_value_vars: Dict[str, tk.StringVar] = {}
step_value_var: Optional[tk.StringVar] = None


def init_ros():
    """Initialize ROS node and create linear_move client."""
    global node, linear_move_client
    if node is not None:
        return
    if not rclpy.ok():
        rclpy.init(args=None)

    node = rclpy.create_node("virtual_controller_ui")

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
        value_var = pose_value_vars.get(axis)
        if value_var is not None:
            value_var.set(f"{axis}: {pose[idx]:.4f}")


def update_step_display():
    """在UI中更新步进值"""
    if step_value_var is not None:
        step_value_var.set(f"step: {step_size:.4f}")


def on_reset():
    """将机械臂运动到reset位姿"""
    global pose
    pose = INITIAL_POSE.copy()
    if node is not None:
        node.get_logger().info("Reset pose to defaults.")
    update_pose_display()
    request_linear_move()


def adjust_pose(index: int, delta: float):
    """将机械臂运动到指定位姿"""
    global pose
    pose[index] += delta
    update_pose_display()
    request_linear_move()


def on_plus_step():
    """增加步进值"""
    global step_size
    step_size += STEP_INCREMENT
    if node is not None:
        node.get_logger().info(f"step++, value:{step_size:.4f}")
    update_step_display()


def on_minus_step():
    """Decrease the step size, bounded by STEP_INCREMENT."""
    global step_size
    step_size = max(STEP_INCREMENT, step_size - STEP_INCREMENT)
    if node is not None:
        node.get_logger().info(f"step--, value:{step_size:.4f}")
    update_step_display()


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
    request.mvvelo = 200.0
    request.mvacc = 200.0
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


def main():
    init_ros()

    def on_close():
        shutdown_ros()
        root.destroy()

    def make_adjust(index: int, sign: float):
        return lambda: adjust_pose(index, sign * step_size)

    root = tk.Tk()
    root.title("virtual_controller")
    root.geometry("480x960")
    root.resizable(False, False)
    root.protocol("WM_DELETE_WINDOW", on_close)

    title_label = tk.Label(root, text="virtual_controller", font=("Arial", 16))
    title_label.pack(pady=12)

    controls_frame = tk.Frame(root)
    controls_frame.pack(fill="both", expand=True, padx=10, pady=10)

    pose_value_vars.clear()
    global step_value_var
    step_value_var = None

    for row, (axis, idx) in enumerate(AXES):
        plus_button = tk.Button(
            controls_frame,
            text=f"+{axis}",
            width=10,
            height=2,
            command=make_adjust(idx, 1.0),
        )
        minus_button = tk.Button(
            controls_frame,
            text=f"-{axis}",
            width=10,
            height=2,
            command=make_adjust(idx, -1.0),
        )

        plus_button.grid(row=row, column=0, padx=5, pady=5, sticky="nsew")
        minus_button.grid(row=row, column=1, padx=5, pady=5, sticky="nsew")

        value_var = tk.StringVar(controls_frame, value=f"{axis}: {pose[idx]:.4f}")
        pose_value_vars[axis] = value_var
        value_label = tk.Label(
            controls_frame, textvariable=value_var, anchor="w", width=18
        )
        value_label.grid(row=row, column=2, padx=5, pady=5, sticky="w")

    for col in range(3):
        controls_frame.columnconfigure(col, weight=1)

    reset_row = len(AXES)
    reset_button = tk.Button(
        controls_frame,
        text="reset",
        height=2,
        command=on_reset,
    )
    reset_button.grid(
        row=reset_row,
        column=0,
        columnspan=3,
        padx=5,
        pady=(10, 5),
        sticky="nsew",
    )

    step_row = reset_row + 1
    plus_step_button = tk.Button(
        controls_frame, text="+step", width=10, height=2, command=on_plus_step
    )
    minus_step_button = tk.Button(
        controls_frame, text="-step", width=10, height=2, command=on_minus_step
    )
    plus_step_button.grid(row=step_row, column=0, padx=5, pady=5, sticky="nsew")
    minus_step_button.grid(row=step_row, column=1, padx=5, pady=5, sticky="nsew")

    step_value_var = tk.StringVar(controls_frame, value=f"step: {step_size:.4f}")
    step_label = tk.Label(
        controls_frame, textvariable=step_value_var, anchor="w", width=18
    )
    step_label.grid(row=step_row, column=2, padx=5, pady=5, sticky="w")

    update_pose_display()
    update_step_display()

    root.mainloop()


if __name__ == "__main__":
    main()
