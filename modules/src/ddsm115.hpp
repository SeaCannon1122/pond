#pragma once

#ifndef DDSM115_HPP
#define DDSM115_HPP

#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "stdbool.h"
#include "ddsm115/DDSM115CMD.h"

namespace quac_hardware
{



class DDSM115 : public hardware_interface::SystemInterface
{

public:

  DDSM115();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  rclcpp::Logger m_Logger;
  std::chrono::time_point<std::chrono::system_clock> m_Time;
  std::vector<ddsm115_motor> m_Wheels;

  DDSM115CMD m_CMD;
  std::string m_Port;
  int m_Act;
};

}

#endif