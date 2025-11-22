#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "jaka_msgs/msg/my_pose_cmd.hpp"

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/moveit_msgs/msg/collision_object.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;


class Commander{
public:
    Commander(std::shared_ptr<rclcpp::Node> node){
        _node = node;
        // 默认使用zu5,
        std::string model = _node->declare_parameter<std::string>("model", "zu5");
        // 创建规划组
        std::string PLANNING_GROUP = "jaka_" + model;
        RCLCPP_INFO(_node->get_logger(), "Using PLANNING_GROUP: %s", PLANNING_GROUP.c_str());

        _arm = std::make_shared<MoveGroupInterface>(_node,PLANNING_GROUP);
        _arm->setMaxVelocityScalingFactor(1.0);
        _arm->setMaxAccelerationScalingFactor(1.0);

        _pose_cmd_sub = node->create_subscription<jaka_msgs::msg::MyPoseCmd>(
            "pose_cmd", 10, std::bind(&Commander::pose_cmd_callback, this, _1));
    }

    void go_to_named_target(const std::string name){
        _arm->setStartStateToCurrentState();
        _arm->setNamedTarget(name);
        RCLCPP_INFO(_node->get_logger(),"try to move to a named target");
        plan_and_execute(_arm);
    }

    void go_to_joint_target(const std::vector<double> &joints){
        _arm->setStartStateToCurrentState();
        _arm->setJointValueTarget(joints);
        RCLCPP_INFO(_node->get_logger(),"try to move to a joint target");
        plan_and_execute(_arm);
    }

    void go_to_pose_target(const double x, const double y, const double z, const double roll, const double pitch, const double yaw, bool cartesian_path=false){
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "Link_0";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        _arm->setStartStateToCurrentState();

        RCLCPP_INFO(_node->get_logger(),"try to move to a pose target");
        if(!cartesian_path){
            _arm->setPoseTarget(target_pose);
            plan_and_execute(_arm);
        }

        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = _arm->computeCartesianPath(waypoints,0.01,0.01,trajectory);

            if(fraction == 1){
                _arm->execute(trajectory);
            }
        }

    }

private:

    void plan_and_execute(const std::shared_ptr<MoveGroupInterface> &interface)
    /*
    * params:const std::shared_ptr<MoveGroupInterface> &interface   控制器接口
    */{
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success){
            RCLCPP_INFO(_node->get_logger(),"successfully plan ,now start to execute");
            interface->execute(plan);
        }
    }


    void pose_cmd_callback(const jaka_msgs::msg::MyPoseCmd::SharedPtr msg){
        RCLCPP_INFO(_node->get_logger(),"Received pose command");
        go_to_pose_target(
            msg->x,
            msg->y,
            msg->z,
            msg->rx,
            msg->ry,
            msg->rz,
            false);
    }

    


    std::shared_ptr<rclcpp::Node> _node;

    // 机械臂控制器
    std::shared_ptr<MoveGroupInterface> _arm;

    rclcpp::Subscription<jaka_msgs::msg::MyPoseCmd>::SharedPtr _pose_cmd_sub;

};


int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    // 创建节点
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<rclcpp::Node>("my_commander",options);
    // 
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
