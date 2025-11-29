#include "user_commands/JAKAZuRobot.h"
#include <thread>
#include <chrono>


int main(){
    JAKAZuRobot robot;
    std::string robot_ip = "192.168.1.100";

    robot.login_in(robot_ip.c_str(), false);
    robot.power_on();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.enable_robot();
    std::this_thread::sleep_for(std::chrono::seconds(4));

    robot.disable_robot();
    robot.power_off();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.login_out();
}
