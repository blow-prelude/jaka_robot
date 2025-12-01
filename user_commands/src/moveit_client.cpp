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

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>



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

        //设置目标位置所使用的参考坐标系
        std::string reference_frame = "Link_0";
        _arm->setPoseReferenceFrame(reference_frame);

        //当运动规划失败后，允许重新规划
        _arm->allowReplanning(true);

        //设置位置(单位：米)和姿态（单位：弧度）的允许误差
        _arm->setGoalPositionTolerance(0.001);
        _arm->setGoalOrientationTolerance(0.01);

        //设置允许的最大速度和加速度
        _arm->setMaxAccelerationScalingFactor(1.0);
        _arm->setMaxVelocityScalingFactor(1.0);

        //获取终端link的名称
        end_effector_link = _arm->getEndEffectorLink();  
        RCLCPP_INFO(_node->get_logger(),"end effector link name: %s",end_effector_link.c_str()); 

        // 订阅末端位姿移动话题
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
            // 获取当前位姿数据最为机械臂运动的起始位姿
            geometry_msgs::msg::Pose start_pose = _arm->getCurrentPose(end_effector_link).pose;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(start_pose);
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = 0.0;
            int attempts = 0;     //已经尝试规划次数


            while(fraction < 1.0 && attempts < this->max_tries)
            {
                fraction = _arm->computeCartesianPath(waypoints, this->eef_step, this->jump_threshold, trajectory);
                attempts++;
        
                if(attempts % 10 == 0)
                    RCLCPP_INFO(_node->get_logger(), "Still trying after %d attempts...", attempts);
            }


            if(fraction == 1){
                RCLCPP_INFO(_node->get_logger(),"successfullly do cartesian plan");

                // 生成机械臂的运动规划数据
	            moveit::planning_interface::MoveGroupInterface::Plan plan;
	            plan.trajectory_ = trajectory;
                // 遍历所有时间序列上的关节状态采样点，包括位姿、速度、受力等
                for(auto it : trajectory.joint_trajectory.points){
                    // 遍历一个关节采样点的位姿信息
                    for(auto posit : it.positions){
                        RCLCPP_INFO(_node->get_logger(),"%f ",posit);
                    }
                }

                // 做时间参数化

                // 创建一个 robot_trajectory 类型的变量，比msg类型更适合做时间参数化、插值等 ，   存储关节路径的轨迹
                robot_trajectory::RobotTrajectory rt(_arm->getRobotModel(), _arm->getName());
                rt.setRobotTrajectoryMsg(*_arm->getCurrentState(), trajectory);
                // 创建一个时间参数化的对象，用来给轨迹里的每个点分配时间，根据速度、加速度平滑轨迹
                trajectory_processing::IterativeParabolicTimeParameterization iptp;
                // 根据速度、加速度约束给轨迹重新打上时间戳，计算各点速度、加速度信息
                bool retrysuccess = iptp.computeTimeStamps(rt, 0.5, 0.5);
                RCLCPP_INFO(_node->get_logger(),"result of retry :%d",retrysuccess);

                // 把做完时间参数化的对象再次转回msg类型,传给参数对象
                rt.getRobotTrajectoryMsg(trajectory);
                 plan.trajectory_ = trajectory;
                _arm->execute(plan);
            }

            else{
                RCLCPP_INFO(_node->get_logger(),"Path planning failed with only %0.6f success after %d attempts.", fraction, max_tries);
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
                current_cmd.cartesian_path);
            lock.lock();
        }
    }


    std::shared_ptr<rclcpp::Node> _node;

    std::string end_effector_link;

    // 机械臂控制器
    std::shared_ptr<MoveGroupInterface> _arm;

    rclcpp::Subscription<jaka_msgs::msg::MyPoseCmd>::SharedPtr _pose_cmd_sub;
    std::mutex _command_mutex;
    std::condition_variable _command_cv;
    std::optional<jaka_msgs::msg::MyPoseCmd> _pending_pose_cmd;
    std::thread _worker;
    bool _stop_worker{false};


    // 笛卡尔空间下的路径规划
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
    int max_tries = 100;   //最大尝试规划次数

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
