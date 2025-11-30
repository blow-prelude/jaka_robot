#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>

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

        _worker = std::thread([this](){ this->process_commands(); });
    }

    ~Commander(){
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _stop_worker = true;
        }
        _command_cv.notify_all();
        if(_worker.joinable()){
            _worker.join();
        }
    }

    Commander(const Commander&) = delete;
    Commander& operator=(const Commander&) = delete;

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
        {
            std::lock_guard<std::mutex> lock(_command_mutex);
            _pending_pose_cmd = *msg;
        }
        _command_cv.notify_one();
    }

    void process_commands(){
        std::unique_lock<std::mutex> lock(_command_mutex);
        while(!_stop_worker){
            // 阻塞线程等待，直到另外一个线程调用唤醒
            // 唤醒后执行lambda表达式，只有返回值为1才会执行接下来的程序
            _command_cv.wait(lock, [this](){
                return _stop_worker || _pending_pose_cmd.has_value();
            });
            if(_stop_worker){
                break;
            }
            auto current_cmd = *_pending_pose_cmd;
            _pending_pose_cmd.reset();
            lock.unlock();
            go_to_pose_target(
                current_cmd.x,
                current_cmd.y,
                current_cmd.z,
                current_cmd.rx,
                current_cmd.ry,
                current_cmd.rz,
                false);
            lock.lock();
        }
    }


    std::shared_ptr<rclcpp::Node> _node;

    // 机械臂控制器
    std::shared_ptr<MoveGroupInterface> _arm;

    rclcpp::Subscription<jaka_msgs::msg::MyPoseCmd>::SharedPtr _pose_cmd_sub;
    std::mutex _command_mutex;
    std::condition_variable _command_cv;
    std::optional<jaka_msgs::msg::MyPoseCmd> _pending_pose_cmd;
    std::thread _worker;
    bool _stop_worker{false};

};


int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    // 创建节点
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<rclcpp::Node>("my_commander",options);
    // 
    [[maybe_unused]] auto commander = std::make_shared<Commander>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
