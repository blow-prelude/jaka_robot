import sys
import rclpy  # 替换 rospy
import numpy as np
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

class CmdLineControlNode(Node):
    def __init__(self):
        super().__init__("cmd_line_end_effector_control")
        # 初始化 MoveIt!（ROS 2 需先初始化 rclpy）
        roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.arm = MoveGroupCommander("jaka_zu5")  # 对应 SRDF 中的运动组名称

        # 配置运动参数
        self.arm.set_goal_joint_tolerance(0.01)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)

        self.get_logger().info("="*50)
        self.get_logger().info("命令行机械臂控制 - 输入末端位姿（单位：m / 度）")
        self.get_logger().info("格式：x y z roll pitch yaw（例如：0.5 0.0 0.5 0 90 0）")
        self.get_logger().info("输入 'q' 退出程序")
        self.get_logger().info("="*50)

    def run(self):
        while rclpy.ok():  # 替换 rospy.is_shutdown()
            # 命令行输入
            input_str = input("请输入末端位姿（x y z r p y）：").strip()
            if input_str.lower() == 'q':
                self.get_logger().info("退出程序...")
                break

            # 解析输入参数
            try:
                x, y, z, roll_deg, pitch_deg, yaw_deg = map(float, input_str.split())
            except:
                self.get_logger().error("输入格式错误！请按 x y z r p y 顺序输入，空格分隔")
                continue

            # 角度转换：度 → 弧度
            roll = np.radians(roll_deg)
            pitch = np.radians(pitch_deg)
            yaw = np.radians(yaw_deg)

            # 生成目标位姿（基坐标系 base_link）
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.header.stamp = self.get_clock().now().to_msg()  # 替换 rospy.Time.now()
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z

            # 欧拉角 → 四元数
            quat = quaternion_from_euler(roll, pitch, yaw)
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]

            # 设置目标位姿并规划执行
            self.arm.set_pose_target(target_pose)
            plan = self.arm.plan()
            if plan[0]:  # 规划成功
                self.get_logger().info("轨迹规划成功，执行运动...")
                self.arm.execute(plan[1], wait=True)
                self.get_logger().info("运动完成！")
            else:
                self.get_logger().error("轨迹规划失败！可能存在碰撞或超出工作空间")

        # 关闭资源
        self.arm.clear_pose_targets()
        roscpp_initialize.shutdown()

def main(args=None):
    rclpy.init(args=args)  # 初始化 rclpy
    node = CmdLineControlNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()