#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <vector>

namespace bluepill_ros2_control
{

class BluepillSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Params
  std::string port_{"/dev/serial0"};
  int baud_{115200};
  double ticks_per_rev_left_{1496.0};
  double ticks_per_rev_right_{748.0};
  double max_wheel_w_{20.0};              // rad/s (usado só pra escalar pro Bluepill)
  bool encoders_are_incremental_{false};  // se Bluepill manda delta por leitura
  
  // “ganho” por roda no bluepill_ros2_control
  double cmd_scale_left_{1.0};
  double cmd_scale_right_{1.0};
  
  // Joint order: 0=left, 1=right
  std::vector<double> pos_{0.0, 0.0};
  std::vector<double> vel_{0.0, 0.0};
  std::vector<double> cmd_{0.0, 0.0};

  // Encoder bookkeeping
  bool have_prev_{false};
  int64_t prev_enc_[2]{0, 0};

  // Serial (POSIX)
  int fd_{-1};
  rclcpp::Logger logger_{rclcpp::get_logger("BluepillSystem")};

  bool open_serial();
  void close_serial();
  bool write_frame(const std::string & s);
  bool read_frame(std::string & out, int timeout_ms);
  bool parse_encoders(const std::string & frame, int64_t & left, int64_t & right);
  static double clamp(double x, double lo, double hi);
};

}  // namespace bluepill_ros2_control
