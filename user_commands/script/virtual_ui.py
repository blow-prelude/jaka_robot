#!/usr/bin/env python3

import tkinter as tk
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from jaka_msgs.msg import MyPoseCmd

INITIAL_POSE: List[float] = [0.0, 0.107, 0.0, -1.57 ,0.0, 0.0]
DEFAULT_STEP = 0.05
STEP_INCREMENT = 0.01

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
pose_publisher: Optional[Publisher] = None
pose_value_vars: Dict[str, tk.StringVar] = {}
step_value_var: Optional[tk.StringVar] = None


def init_ros():
    """Initialize ROS node and pose publisher."""
    global node, pose_publisher
    if node is not None:
        return
    if not rclpy.ok():
        rclpy.init(args=None)
    node = rclpy.create_node("virtual_controller_ui")
    pose_publisher = node.create_publisher(MyPoseCmd, "/pose_cmd", 10)


def shutdown_ros():
    """Shutdown ROS on app close."""
    global node, pose_publisher
    pose_publisher = None
    if node is not None:
        node.destroy_node()
        node = None
    if rclpy.ok():
        rclpy.shutdown()


def update_pose_display():
    """Refresh pose labels in the UI."""
    for axis, idx in AXES:
        value_var = pose_value_vars.get(axis)
        if value_var is not None:
            value_var.set(f"{axis}: {pose[idx]:.4f}")


def update_step_display():
    """Refresh step label in the UI."""
    if step_value_var is not None:
        step_value_var.set(f"step: {step_size:.4f}")


def publish_pose():
    """Publish the current pose on /pose_cmd and log the payload."""
    if node is None or pose_publisher is None:
        return
    msg = MyPoseCmd()
    msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = pose
    pose_publisher.publish(msg)
    node.get_logger().info(
        "Publish pose_cmd: x={:.4f}, y={:.4f}, z={:.4f}, rx={:.4f}, ry={:.4f}, rz={:.4f}".format(
            msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz
        )
    )


def adjust_pose(index: int, delta: float):
    global pose
    pose[index] += delta
    update_pose_display()
    publish_pose()


def on_reset():
    """Reset pose to defaults and publish the result."""
    global pose
    pose = INITIAL_POSE.copy()
    if node is not None:
        node.get_logger().info("Reset pose to defaults.")
    update_pose_display()
    publish_pose()


def on_plus_step():
    """Increase the step size."""
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
