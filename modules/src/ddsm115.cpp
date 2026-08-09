#include "ddsm115/ddsm115.hpp"

#include <chrono>
#include <limits>
#include <memory>
#include <vector>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quac_hardware
{

DDSM115::DDSM115() : m_Logger(rclcpp::get_logger("DDSM115"))
{

}

hardware_interface::CallbackReturn DDSM115::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;

  RCLCPP_INFO(m_Logger, "Hardware declaration:");

  m_Port = info.hardware_parameters.at("port");
  m_Act = std::stoi(info.hardware_parameters.at("act"));

  RCLCPP_INFO(m_Logger, "  port: '%s'", m_Port.c_str());
  RCLCPP_INFO(m_Logger, "  act: %d", m_Act);

  m_Wheels.resize(0);

  for (size_t i = 0; i < info.joints.size(); i++)
  {
    const auto &joint = info_.joints[i];

    ddsm115_motor w;
    w.name = joint.name;
    w.command_velocity = 0.0;
    w.state_position = 0.0;
    w.state_velocity = 0.0;
    w.id = 0;
    w.scalar = 1;
    w.last_position = 0;
    w.read = false;

    for (const auto &p : joint.parameters) if (p.first == "id") w.id = std::stoi(p.second);
    for (const auto &p : joint.parameters) if (p.first == "scalar") w.scalar = std::stoi(p.second);

    RCLCPP_INFO(m_Logger, "  Joint %s:", joint.name.c_str());
    RCLCPP_INFO(m_Logger, "    id: %d", w.id);
    RCLCPP_INFO(m_Logger, "    scalar: %d", w.scalar);
    m_Wheels.push_back(w);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DDSM115::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < m_Wheels.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(m_Wheels[i].name, hardware_interface::HW_IF_POSITION, &m_Wheels[i].state_position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(m_Wheels[i].name, hardware_interface::HW_IF_VELOCITY, &m_Wheels[i].state_velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(m_Wheels[i].name, "current", &m_Wheels[i].state_current));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DDSM115::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < m_Wheels.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(m_Wheels[i].name, hardware_interface::HW_IF_VELOCITY, &m_Wheels[i].command_velocity));
  }
  
  return command_interfaces;
}

hardware_interface::CallbackReturn DDSM115::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(m_Logger, "Configuring ...please wait...");

  if (m_CMD.connect(m_Port) == false)
  {
    RCLCPP_INFO(m_Logger, m_CMD.get_error());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(m_Logger, "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSM115::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(m_Logger, "Cleaning up ...please wait...");
  
  m_CMD.disconnect();

  RCLCPP_INFO(m_Logger, "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DDSM115::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();

  for (size_t i = 0; i < m_Wheels.size(); i++)
  {
    double vel = m_Wheels[i].command_velocity, pos = m_Wheels[i].state_position + m_Wheels[i].state_velocity * dt, cur = 0.;

    uint8_t fb_id, fb_mode, fb_error_code;
    double fb_vel, fb_pos, fb_cur;

    if (m_CMD.drive_feedback(&fb_id, &fb_mode, &fb_pos, &fb_vel, &fb_cur, &fb_error_code) == false) RCLCPP_INFO(m_Logger, m_CMD.get_error());
    else if (fb_id != m_Wheels[i].id) RCLCPP_INFO(m_Logger, "Received response for wheel %d instead of %d", fb_id, m_Wheels[i].id);
    else
    {
      if (m_Wheels[i].read == false)
      {
        m_Wheels[i].last_position = fb_pos;
        m_Wheels[i].read = true;
      }

      vel = (double)m_Wheels[i].scalar * fb_vel;

      double delta = fb_pos - m_Wheels[i].last_position;
      m_Wheels[i].last_position = fb_pos;

      if (delta > M_PI) delta -= 2.0 * M_PI;
      else if (delta < -M_PI) delta += 2.0 * M_PI;

      pos = m_Wheels[i].state_position - (double)m_Wheels[i].scalar * delta;
      cur = fb_cur;

    }

    m_Wheels[i].state_velocity = vel;
    m_Wheels[i].state_position = pos;
    m_Wheels[i].state_current = cur;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DDSM115::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < m_Wheels.size(); i++)
    if (m_CMD.drive(m_Wheels[i].id, m_Wheels[i].command_velocity * m_Wheels[i].scalar, m_Act, 0) == false)
      RCLCPP_INFO(m_Logger, m_CMD.get_error());

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quac_hardware::DDSM115, hardware_interface::SystemInterface)