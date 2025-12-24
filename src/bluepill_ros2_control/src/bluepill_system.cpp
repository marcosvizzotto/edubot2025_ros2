#include "bluepill_ros2_control/bluepill_system.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <chrono>

namespace bluepill_ros2_control
{

static int32_t diff16(int32_t now, int32_t prev)
{
  int32_t d = now - prev;
  if (d >  32767) d -= 65536;
  if (d < -32768) d += 65536;
  return d;
}

static speed_t to_baud(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B115200;
  }
}

hardware_interface::CallbackReturn BluepillSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // lê parâmetros do URDF (ros2_control <param .../>)
  auto get = [&](const std::string & key, const std::string & def) {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  port_ = get("serial_port", port_);
  baud_ = std::stoi(get("baud", std::to_string(baud_)));
  ticks_per_rev_left_  = std::stod(get("ticks_per_rev_left",  std::to_string(ticks_per_rev_left_)));
  ticks_per_rev_right_ = std::stod(get("ticks_per_rev_right", std::to_string(ticks_per_rev_right_)));
  max_wheel_w_ = std::stod(get("max_wheel_angular_velocity", std::to_string(max_wheel_w_)));
  encoders_are_incremental_ = (get("encoders_are_incremental", "false") == "true");
  cmd_scale_left_  = std::stod(get("cmd_scale_left",  "1.0"));
  cmd_scale_right_ = std::stod(get("cmd_scale_right", "1.0"));

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Esperado 2 joints (left/right). Recebi: %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_WARN(logger_, "HW params: cmd_scale_left=%.3f cmd_scale_right=%.3f max_w=%.3f", cmd_scale_left_, cmd_scale_right_, max_wheel_w_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BluepillSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < 2; i++) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> BluepillSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < 2; i++) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_[i]);
  }
  return ci;
}

hardware_interface::CallbackReturn BluepillSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!open_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  have_prev_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BluepillSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BluepillSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  std::string frame;
  if (!read_frame(frame, 20)) {  // 20ms timeout
    return hardware_interface::return_type::OK; // não trava o control loop
  }

  int64_t enc_l = 0, enc_r = 0;
  if (!parse_encoders(frame, enc_l, enc_r)) {
    return hardware_interface::return_type::OK;
  }

  // converte ticks -> rad
  const double ticks_to_rad_l = (2.0 * M_PI) / ticks_per_rev_left_;
  const double ticks_to_rad_r = (2.0 * M_PI) / ticks_per_rev_right_;

  int64_t dL = 0, dR = 0;
  if (encoders_are_incremental_) {
    dL = enc_l;
    dR = enc_r;
  } else {
    if (!have_prev_) {
      prev_enc_[0] = enc_l;
      prev_enc_[1] = enc_r;
      have_prev_ = true;
      vel_[0] = vel_[1] = 0.0;
      return hardware_interface::return_type::OK;
    }
    dL = diff16(static_cast<int32_t>(enc_l), static_cast<int32_t>(prev_enc_[0]));
    dR = diff16(static_cast<int32_t>(enc_r), static_cast<int32_t>(prev_enc_[1]));
    prev_enc_[0] = enc_l;
    prev_enc_[1] = enc_r;
  }

  const double dt = period.seconds();
  const double dpos_l = static_cast<double>(dL) * ticks_to_rad_l;
  const double dpos_r = static_cast<double>(dR) * ticks_to_rad_r;

  pos_[0] += dpos_l;
  pos_[1] += dpos_r;

  if (dt > 1e-6) {
    vel_[0] = dpos_l / dt;
    vel_[1] = dpos_r / dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BluepillSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // cmd_ chega em rad/s (do diff_drive_controller). Precisamos mandar pro Bluepill no formato <L,R>.
  // Como você não quer mexer no firmware, a saída mais segura é escalar pra [-1, 1] (ou pra faixa que seu firmware espera).
  const double l = clamp((cmd_[0] / max_wheel_w_) * cmd_scale_left_,  -1.0, 1.0);
  const double r = clamp((cmd_[1] / max_wheel_w_) * cmd_scale_right_, -1.0, 1.0);

  char buf[64];
  std::snprintf(buf, sizeof(buf), "<%.3f,%.3f>", l, r);
  write_frame(std::string(buf));

  static int c = 0;
  if (++c % 50 == 0) {RCLCPP_INFO(logger_, "WRITE: cmd(rad/s) L=%.3f R=%.3f -> send <%.3f,%.3f>", cmd_[0], cmd_[1], l, r);}

  return hardware_interface::return_type::OK;
}

// ---------------- Serial helpers ----------------

bool BluepillSystem::open_serial()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger_, "Falha abrindo %s: %s", port_.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr: %s", std::strerror(errno));
    close_serial();
    return false;
  }

  cfmakeraw(&tty);
  speed_t spd = to_baud(baud_);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS; // sem flow control
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr: %s", std::strerror(errno));
    close_serial();
    return false;
  }

  RCLCPP_INFO(logger_, "Serial OK: %s @ %d", port_.c_str(), baud_);
  return true;
}

void BluepillSystem::close_serial()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool BluepillSystem::write_frame(const std::string & s)
{
  if (fd_ < 0) return false;
  const ssize_t n = ::write(fd_, s.c_str(), s.size());
  return n == (ssize_t)s.size();
}

bool BluepillSystem::read_frame(std::string & out, int timeout_ms)
{
  out.clear();
  if (fd_ < 0) return false;

  bool started = false;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    auto now = std::chrono::steady_clock::now();
    auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
    if (remaining.count() <= 0) break;

    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd_, &set);

    timeval tv{};
    tv.tv_sec  = static_cast<int>(remaining.count() / 1000000);
    tv.tv_usec = static_cast<int>(remaining.count() % 1000000);

    int rv = select(fd_ + 1, &set, nullptr, nullptr, &tv);
    if (rv <= 0) {
      continue; // timeout ou sinal
    }

    char buf[128];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) continue;

    for (ssize_t i = 0; i < n; ++i) {
      char c = buf[i];

      if (!started) {
        if (c == '<') {
          started = true;
          out.push_back(c);
        }
        continue;
      }

      out.push_back(c);

      // proteção contra lixo infinito (opcional)
      if (out.size() > 256) {
        out.clear();
        started = false;
        continue;
      }

      if (c == '>') return true;
    }
  }

  return false;
}

bool BluepillSystem::parse_encoders(const std::string & frame, int64_t & left, int64_t & right)
{
  // espera algo tipo: "<123,456>"
  if (frame.size() < 5) return false;
  if (frame.front() != '<' || frame.back() != '>') return false;

  const auto body = frame.substr(1, frame.size() - 2);
  const auto comma = body.find(',');
  if (comma == std::string::npos) return false;

  try {
    left = std::stoll(body.substr(0, comma));
    right = std::stoll(body.substr(comma + 1));
  } catch (...) {
    return false;
  }
  return true;
}

double BluepillSystem::clamp(double x, double lo, double hi)
{
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

}  // namespace bluepill_ros2_control

PLUGINLIB_EXPORT_CLASS(bluepill_ros2_control::BluepillSystem, hardware_interface::SystemInterface)
