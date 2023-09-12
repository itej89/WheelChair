// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "wheel_chair/WheelChairSystemHardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wheel_chair
{
CallbackReturn WheelChairSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (
      hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS)
      {
        return CallbackReturn::ERROR;
      }



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
        return CallbackReturn::ERROR;
    }


    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one cwheel_chair.urdfommand interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("WheelChairSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("WheelChairSystemHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("WheelChairSystemHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("WheelChairSystemHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("WheelChairSystemHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> WheelChairSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WheelChairSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn WheelChairSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WheelChairSystemHardware"), "Activating ...please wait...");
    
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("WheelChairSystemHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn WheelChairSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("WheelChairSystemHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("WheelChairSystemHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type WheelChairSystemHardware::read()
{
  double SCALE_FACTOR = 10000;

  uint8_t BASE_COMMAND_READ_STATE[8] = {0x1F, 0xA8, 0x01, 0x20, 0x01, 0x01 ,0x1F, 0xA9};
  std::vector<uint8_t> read_command(BASE_COMMAND_READ_STATE, BASE_COMMAND_READ_STATE + sizeof(BASE_COMMAND_READ_STATE) / sizeof(BASE_COMMAND_READ_STATE[0]) );

  std::vector<uint8_t> BASE_STATE;
  
  base_port->write(read_command);

  base_port->read (BASE_STATE, 23);

  uint32_t LeftWheelPos = BASE_STATE[3];
  LeftWheelPos = (LeftWheelPos << 8 | BASE_STATE[4]);
  LeftWheelPos = (LeftWheelPos << 8 | BASE_STATE[5]);
  LeftWheelPos = (LeftWheelPos << 8 | BASE_STATE[6]);

  uint32_t RightWheelPos = BASE_STATE[7];
  RightWheelPos = (RightWheelPos << 8 | BASE_STATE[8]);
  RightWheelPos = (RightWheelPos << 8 | BASE_STATE[9]);
  RightWheelPos = (RightWheelPos << 8 | BASE_STATE[10]);

  uint8_t LEFT_WHEEL_DIR = BASE_STATE[16];
  uint32_t LeftWheelVel = BASE_STATE[12];
  LeftWheelVel = (LeftWheelVel << 8 | BASE_STATE[13]);
  LeftWheelVel = (LeftWheelVel << 8 | BASE_STATE[14]);
  LeftWheelVel = (LeftWheelVel << 8 | BASE_STATE[15]);

  uint8_t RIGHT_WHEEL_DIR = BASE_STATE[16];
  uint32_t RightWheelVel = BASE_STATE[17];
  RightWheelVel = (RightWheelVel << 8 | BASE_STATE[18]);
  RightWheelVel = (RightWheelVel << 8 | BASE_STATE[19]);
  RightWheelVel = (RightWheelVel << 8 | BASE_STATE[20]);

  hw_positions_[0] = (LeftWheelPos*1.0)/SCALE_FACTOR;
  hw_positions_[1] = (RightWheelPos*1.0)/SCALE_FACTOR;

  hw_velocities_[0] = (LEFT_WHEEL_DIR*-2+1)*(LeftWheelVel*1.0)/SCALE_FACTOR;
  hw_velocities_[1] = (RIGHT_WHEEL_DIR*-2+1)*(RightWheelVel*1.0)/SCALE_FACTOR;

  hw_velocities_[0] = hw_velocities_[0]/1000;
  hw_velocities_[1] = hw_velocities_[1]/1000;


  // for (std::size_t i = 0; i < hw_velocities_.size(); i++)
  // {

  //   RCLCPP_INFO(
  //     rclcpp::get_logger("WheelChairSystemHardware"),
  //     "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
  //     hw_velocities_[i], info_.joints[i].name.c_str());
  // }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheelChairSystemHardware::write()
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // RCLCPP_INFO(rclcpp::get_logger("WheelChairSystemHardware"), "Writing...");

  // for (auto i = 0u; i < hw_commands_.size(); i++)
  // {
  //   // Simulate sending commands to the hardware
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("WheelChairSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
  //     info_.joints[i].name.c_str());

  //   hw_velocities_[i] = hw_commands_[i];
  // }


  uint8_t BASE_COMMAND_WRITE_VELOCITIES[18] = {0x1F, 0xA8, 0x0B, 0x10, 
  0x00, 
  0x00, 0x00, 0x00, 0x00, 
  0x00, 
  0x00, 0x00, 0x00, 0x00,
  0x01, 0x01 ,0x1F, 0xA9};

  BASE_COMMAND_WRITE_VELOCITIES[4] = hw_velocities_[0] < 0 ? 0x01 : 0x00;
  BASE_COMMAND_WRITE_VELOCITIES[9] = hw_velocities_[1] < 0 ? 0x01 : 0x00;

  double vel_l_mm_per_sec = hw_velocities_[0] * 1000;
  double vel_r_mm_per_sec = hw_velocities_[1] * 1000;

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


  std::vector<uint8_t> write_command(BASE_COMMAND_WRITE_VELOCITIES, BASE_COMMAND_WRITE_VELOCITIES + sizeof(BASE_COMMAND_WRITE_VELOCITIES) / sizeof(BASE_COMMAND_WRITE_VELOCITIES[0]) );
  base_port->write(write_command);

  // RCLCPP_INFO(rclcpp::get_logger("WheelChairSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace wheel_chair

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  wheel_chair::WheelChairSystemHardware, hardware_interface::SystemInterface)