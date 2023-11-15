#include "wheel_chair/ChairInterface.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fstream>

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
    
    std::ofstream f("log.txt");

// msg->linear.x = 0.1;
// msg->angular.z = 0.05;
    double Vr =  (msg->linear.x - (0.55/2)*msg->angular.z)/0.095;
    double Vl =  (msg->linear.x + (0.55/2)*msg->angular.z)/0.095;

    f << msg->linear.x  << ", " << msg->angular.z << "::" << Vr << ", " << Vl << "\n";

    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("ChairInterface"), "Writing...");



    uint8_t BASE_COMMAND_WRITE_VELOCITIES[18] = {0x1F, 0xA8, 0x0B, 0x10, 
    0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 
    0x00, 0x00, 0x00, 0x00,
    0x01, 0x01 ,0x1F, 0xA9};

    BASE_COMMAND_WRITE_VELOCITIES[4] = Vl < 0 ? 0x01 : 0x00;
    BASE_COMMAND_WRITE_VELOCITIES[9] = Vr < 0 ? 0x01 : 0x00;

    double vel_l_mm_per_sec = abs(Vl) * 1000;
    double vel_r_mm_per_sec = abs(Vr) * 1000;

    uint32_t left_vel  = vel_l_mm_per_sec;
    uint32_t right_vel = vel_r_mm_per_sec;

    BASE_COMMAND_WRITE_VELOCITIES[5] = (left_vel >> 24) & 0xFF;
    BASE_COMMAND_WRITE_VELOCITIES[6] = (left_vel >> 16) & 0xFF;
    BASE_COMMAND_WRITE_VELOCITIES[7] = (left_vel >> 8) & 0xFF;
    BASE_COMMAND_WRITE_VELOCITIES[8] = (left_vel) & 0xFF;

    BASE_COMMAND_WRITE_VELOCITIES[10] = (right_vel >> 24) & 0xFF;
    BASE_COMMAND_WRITE_VELOCITIES[11] = (right_vel >> 16) & 0xFF;
    BASE_COMMAND_WRITE_VELOCITIES[12] = (right_vel >> 8) & 0xFF;
    BASE_COMMAND_WRITE_VELOCITIES[13] = (right_vel) & 0xFF;



    std::stringstream str_command_stream;

    for(int i=0; i<18; i++){
        str_command_stream << std::hex << std::setw(2) << (int)BASE_COMMAND_WRITE_VELOCITIES[i]; 
        str_command_stream << " ";
    }
    
    str_command_stream << "\n";
    std::string str_command = str_command_stream.str();

    RCLCPP_INFO(
        rclcpp::get_logger("ChairInterface"), "Sending Command '%s'!", str_command.c_str()); 


    std::vector<uint8_t> write_command(BASE_COMMAND_WRITE_VELOCITIES, BASE_COMMAND_WRITE_VELOCITIES + sizeof(BASE_COMMAND_WRITE_VELOCITIES) / sizeof(BASE_COMMAND_WRITE_VELOCITIES[0]) );
    base_port->write(write_command);


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