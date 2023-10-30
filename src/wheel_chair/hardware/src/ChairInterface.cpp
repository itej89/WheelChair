#include "wheel_chair/ChairInterface.hpp"
#include "rclcpp/rclcpp.hpp"

ChairInterface::ChairInterface(std::string nodeName): Node(nodeName) {
    subscriber = this->create_subscription<geometry_msgs::msg::Twist>("/wheel_chair_base_controller/cmd_vel_unstamped", 10, 
                            std::bind(&ChairInterface::cmdvelCallback, this, std::placeholders::_1));

    base_port = std::make_shared<serial::Serial>("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
    if(base_port->isOpen()){
        RCLCPP_FATAL(
          rclcpp::get_logger("WheelChairSystemHardware"),
          "Base platform connection has been succesfully opened.");
    }
    else
    {
        RCLCPP_FATAL(
          rclcpp::get_logger("WheelChairSystemHardware"),
          "Unable to open connection to the base platform device.");
    }
}


void ChairInterface::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Recieved velocity: " << msg->linear.x << "\n z : " << msg->angular.z);
}



int main(int argc, char *argv[])
{
    //Initialize ros
    rclcpp::init(argc, argv);

    //Create ros node
    auto node = std::make_shared<ChairInterface>("chair_interface");

    //Spin ros node
    rclcpp::spin(node);

    //Shutdown ros node
    rclcpp::shutdown();
}