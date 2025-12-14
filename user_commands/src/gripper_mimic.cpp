#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <optional>
#include <string>
#include <vector>


class GripperMimicNode : public rclcpp::Node
{
public:
    GripperMimicNode()
        : rclcpp::Node("gripper_mimic_node")
    {
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&GripperMimicNode::joint_state_callback, this, std::placeholders::_1));

        command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "gripper_mimic_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(),
                    "GripperMimicNode started, mirroring gripper_left_finger_joint to gripper_right_finger_joint");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!left_index_.has_value()) {
            for (std::size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "gripper_left_finger_joint") {
                    left_index_ = static_cast<int>(i);
                    RCLCPP_INFO(this->get_logger(),
                                "Found gripper_left_finger_joint in joint_states at index %d",
                                left_index_.value());
                    break;
                }
            }
            if (!left_index_.has_value()) {
                return;
            }
        }

        const int idx = left_index_.value();
        if (idx < 0 || static_cast<std::size_t>(idx) >= msg->position.size()) {
            return;
        }

        const double left_pos = msg->position[static_cast<std::size_t>(idx)];
        double target_right = -left_pos;

        // Clamp to URDF limits [-0.04, 0.0]
        constexpr double min_pos = -0.04;
        constexpr double max_pos = 0.0;
        if (target_right < min_pos) {
            target_right = min_pos;
        } else if (target_right > max_pos) {
            target_right = max_pos;
        }

        if (last_command_.has_value()) {
            const double diff = std::abs(target_right - last_command_.value());
            if (diff < 1e-4) {
                return;
            }
        }

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data.resize(1);
        cmd_msg.data[0] = target_right;
        command_pub_->publish(cmd_msg);

        last_command_ = target_right;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    std::optional<int> left_index_;
    std::optional<double> last_command_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperMimicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

