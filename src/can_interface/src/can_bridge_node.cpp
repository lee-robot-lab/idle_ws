// CAN <-> ROS bridge node for MIT protocol (Type02 state/error, Type01 command relay).
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "msgs/msg/motor_cmd.hpp"
#include "msgs/msg/motor_cmd_array.hpp"
#include "msgs/msg/motor_error.hpp"
#include "msgs/msg/motor_error_array.hpp"
#include "msgs/msg/motor_state.hpp"
#include "msgs/msg/motor_state_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr uint8_t TYPE00_GET_DEVICE_ID = 0x00;
constexpr uint8_t TYPE01_OPERATION_CONTROL = 0x01;
constexpr uint8_t TYPE02_FEEDBACK = 0x02;
constexpr uint8_t TYPE03_ENABLE = 0x03;
constexpr uint8_t TYPE04_STOP = 0x04;
constexpr uint8_t TYPE_MASK = 0x1F;
constexpr float kHomeReturnDurationSec = 5.0F;
constexpr float kHomeEpsilon = 1e-4F;
constexpr char kDefaultHomeQDesByMotorJson[] =
  "{\"1\":0.0,\"2\":0.0,\"3\":3.141592653589793,\"4\":0.0}";
constexpr char kDefaultHomeKpByMotorJson[] =
  "{\"1\":5.0,\"2\":37.0,\"3\":30.0,\"4\":30.0}";
constexpr char kDefaultHomeKdByMotorJson[] =
  "{\"1\":0.5,\"2\":6.0,\"3\":5.0,\"4\":5.0}";
constexpr char kDefaultQLimitMinByMotorJson[] =
  "{\"1\":-3.141592653589793,\"2\":-1.78,\"3\":-3.141592653589793,\"4\":-3.141592653589793}";
constexpr char kDefaultQLimitMaxByMotorJson[] =
  "{\"1\":3.141592653589793,\"2\":1.78,\"3\":3.141592653589793,\"4\":3.141592653589793}";

struct MitRanges
{
  float p_min;
  float p_max;
  float v_min;
  float v_max;
  float t_min;
  float t_max;
  float kp_min;
  float kp_max;
  float kd_min;
  float kd_max;
};

const MitRanges MIT_RS02{
  -4.0F * static_cast<float>(M_PI), 4.0F * static_cast<float>(M_PI),
  -44.0F, 44.0F,
  -17.0F, 17.0F,
  0.0F, 500.0F,
  0.0F, 5.0F,
};

const MitRanges MIT_RS03{
  -4.0F * static_cast<float>(M_PI), 4.0F * static_cast<float>(M_PI),
  -20.0F, 20.0F,
  -60.0F, 60.0F,
  0.0F, 5000.0F,
  0.0F, 100.0F,
};

inline uint32_t pack_ext_id(uint8_t comm_type, uint16_t data2, uint8_t data1)
{
  return (static_cast<uint32_t>(comm_type & TYPE_MASK) << 24) |
         (static_cast<uint32_t>(data2) << 8) |
         static_cast<uint32_t>(data1);
}

inline uint8_t comm_type_from_arb(uint32_t arb_id)
{
  return static_cast<uint8_t>((arb_id >> 24) & TYPE_MASK);
}

inline uint16_t data2_from_arb(uint32_t arb_id)
{
  return static_cast<uint16_t>((arb_id >> 8) & 0xFFFFU);
}

inline uint8_t data1_from_arb(uint32_t arb_id)
{
  return static_cast<uint8_t>(arb_id & 0xFFU);
}

inline float clamp(float x, float lo, float hi)
{
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

inline float nearest_periodic_target(
  const float target_base,
  const float current,
  const float period,
  const float lo,
  const float hi)
{
  if (period <= 0.0F) {
    return clamp(target_base, lo, hi);
  }
  float target = target_base + std::round((current - target_base) / period) * period;
  while (target < lo) {
    target += period;
  }
  while (target > hi) {
    target -= period;
  }
  return clamp(target, lo, hi);
}

inline uint16_t float_to_u16(float x, float xmin, float xmax)
{
  const float clamped = clamp(x, xmin, xmax);
  const float scaled = (clamped - xmin) * 65535.0F / (xmax - xmin);
  const float rounded = std::round(scaled);
  return static_cast<uint16_t>(rounded);
}

inline float u16_to_float(uint16_t u, float xmin, float xmax)
{
  const float ratio = static_cast<float>(u) / 65535.0F;
  return xmin + ratio * (xmax - xmin);
}

inline uint16_t be_u16(const uint8_t hi, const uint8_t lo)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

inline builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & t)
{
  constexpr int64_t kNsecPerSec = 1000000000LL;
  const int64_t ns = t.nanoseconds();
  int64_t sec = ns / kNsecPerSec;
  int64_t nsec = ns % kNsecPerSec;
  if (nsec < 0) {
    --sec;
    nsec += kNsecPerSec;
  }

  builtin_interfaces::msg::Time out {};
  out.sec = static_cast<int32_t>(sec);
  out.nanosec = static_cast<uint32_t>(nsec);
  return out;
}
}  // namespace

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode()
  : Node("can_bridge_node")
  {
    channel_ = declare_parameter<std::string>("channel", "can0");
    host_id_ = static_cast<uint8_t>(declare_parameter<int>("host_id", 0xFD) & 0xFF);
    rs03_id_ = static_cast<uint8_t>(declare_parameter<int>("rs03_id", 2) & 0xFF);
    scan_min_id_ = declare_parameter<int>("scan_min_id", 1);
    scan_max_id_ = declare_parameter<int>("scan_max_id", 10);
    scan_wait_ms_ = declare_parameter<int>("scan_wait_ms", 200);
    rx_max_frames_per_tick_ = declare_parameter<int>("rx_max_frames_per_tick", 100);
    default_tx_hz_ = declare_parameter<double>("default_tx_hz", 500.0);
    tx_hz_default_ = declare_parameter<double>("tx_hz_default", default_tx_hz_);
    tx_hz_by_motor_json_ = declare_parameter<std::string>("tx_hz_by_motor_json", "{}");
    cmd_timeout_ms_ = declare_parameter<int>("cmd_timeout_ms", 100);
    home_q_des_ = declare_parameter<double>("home_q_des", 0.0);
    home_q_des_by_motor_json_ = declare_parameter<std::string>(
      "home_q_des_by_motor_json", kDefaultHomeQDesByMotorJson);
    home_qd_des_ = declare_parameter<double>("home_qd_des", 0.0);
    home_kp_ = declare_parameter<double>("home_kp", 30.0);
    home_kp_by_motor_json_ = declare_parameter<std::string>(
      "home_kp_by_motor_json", kDefaultHomeKpByMotorJson);
    home_kd_ = declare_parameter<double>("home_kd", 5.0);
    home_kd_by_motor_json_ = declare_parameter<std::string>(
      "home_kd_by_motor_json", kDefaultHomeKdByMotorJson);
    home_tau_ff_ = declare_parameter<double>("home_tau_ff", 0.0);
    q_limit_min_by_motor_json_ = declare_parameter<std::string>(
      "q_limit_min_by_motor_json", kDefaultQLimitMinByMotorJson);
    q_limit_max_by_motor_json_ = declare_parameter<std::string>(
      "q_limit_max_by_motor_json", kDefaultQLimitMaxByMotorJson);
    if (default_tx_hz_ <= 0.0) {
      default_tx_hz_ = 500.0;
    }
    if (tx_hz_default_ <= 0.0) {
      tx_hz_default_ = default_tx_hz_;
    }
    if (cmd_timeout_ms_ < 0) {
      cmd_timeout_ms_ = 0;
    }
    if (home_kp_ < 0.0) {
      home_kp_ = 0.0;
    }
    if (home_kd_ < 0.0) {
      home_kd_ = 0.0;
    }

    std::unordered_map<int, double> tx_by_motor;
    std::string parse_error;
    if (!parse_tx_hz_by_motor_json(tx_hz_by_motor_json_, tx_by_motor, parse_error)) {
      RCLCPP_WARN(
        get_logger(),
        "invalid initial tx_hz_by_motor_json (%s): falling back to {}",
        parse_error.c_str());
      tx_hz_by_motor_json_ = "{}";
      tx_by_motor.clear();
    }
    apply_tx_policy(tx_hz_default_, tx_by_motor, false);

    std::unordered_map<int, double> home_q_des_by_motor;
    std::unordered_map<int, double> home_kp_by_motor;
    std::unordered_map<int, double> home_kd_by_motor;
    std::unordered_map<int, double> q_limit_min_by_motor;
    std::unordered_map<int, double> q_limit_max_by_motor;
    if (!parse_numeric_by_motor_json(home_q_des_by_motor_json_, home_q_des_by_motor, parse_error)) {
      RCLCPP_WARN(
        get_logger(),
        "invalid initial home_q_des_by_motor_json (%s): falling back to defaults",
        parse_error.c_str());
      home_q_des_by_motor_json_ = kDefaultHomeQDesByMotorJson;
      home_q_des_by_motor.clear();
      std::string ignored_error;
      (void)parse_numeric_by_motor_json(home_q_des_by_motor_json_, home_q_des_by_motor, ignored_error);
    }
    if (!parse_numeric_by_motor_json(home_kp_by_motor_json_, home_kp_by_motor, parse_error)) {
      RCLCPP_WARN(
        get_logger(),
        "invalid initial home_kp_by_motor_json (%s): falling back to defaults",
        parse_error.c_str());
      home_kp_by_motor_json_ = kDefaultHomeKpByMotorJson;
      home_kp_by_motor.clear();
      std::string ignored_error;
      (void)parse_numeric_by_motor_json(home_kp_by_motor_json_, home_kp_by_motor, ignored_error);
    }
    if (!parse_numeric_by_motor_json(home_kd_by_motor_json_, home_kd_by_motor, parse_error)) {
      RCLCPP_WARN(
        get_logger(),
        "invalid initial home_kd_by_motor_json (%s): falling back to defaults",
        parse_error.c_str());
      home_kd_by_motor_json_ = kDefaultHomeKdByMotorJson;
      home_kd_by_motor.clear();
      std::string ignored_error;
      (void)parse_numeric_by_motor_json(home_kd_by_motor_json_, home_kd_by_motor, ignored_error);
    }
    if (!parse_numeric_by_motor_json(q_limit_min_by_motor_json_, q_limit_min_by_motor, parse_error)) {
      RCLCPP_WARN(
        get_logger(),
        "invalid initial q_limit_min_by_motor_json (%s): falling back to defaults",
        parse_error.c_str());
      q_limit_min_by_motor_json_ = kDefaultQLimitMinByMotorJson;
      q_limit_min_by_motor.clear();
      std::string ignored_error;
      (void)parse_numeric_by_motor_json(q_limit_min_by_motor_json_, q_limit_min_by_motor, ignored_error);
    }
    if (!parse_numeric_by_motor_json(q_limit_max_by_motor_json_, q_limit_max_by_motor, parse_error)) {
      RCLCPP_WARN(
        get_logger(),
        "invalid initial q_limit_max_by_motor_json (%s): falling back to defaults",
        parse_error.c_str());
      q_limit_max_by_motor_json_ = kDefaultQLimitMaxByMotorJson;
      q_limit_max_by_motor.clear();
      std::string ignored_error;
      (void)parse_numeric_by_motor_json(q_limit_max_by_motor_json_, q_limit_max_by_motor, ignored_error);
    }
    apply_home_policy(
      home_q_des_,
      home_qd_des_,
      home_kp_,
      home_kd_,
      home_tau_ff_,
      home_q_des_by_motor,
      home_kp_by_motor,
      home_kd_by_motor,
      q_limit_min_by_motor,
      q_limit_max_by_motor,
      false);

    if (!open_socketcan(channel_)) {
      throw std::runtime_error("failed to open socketcan channel: " + channel_);
    }

    const auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    const auto qos_state = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    const auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();

    state_array_pub_ = create_publisher<msgs::msg::MotorStateArray>("/motor_state_array", qos_state);
    error_array_pub_ = create_publisher<msgs::msg::MotorErrorArray>("/motor_error_array", qos_reliable);
    cmd_array_sub_ = create_subscription<msgs::msg::MotorCMDArray>(
      "/motor_cmd_array",
      qos_cmd,
      std::bind(&CanBridgeNode::on_cmd_array, this, std::placeholders::_1));

    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&CanBridgeNode::on_parameters_set, this, std::placeholders::_1));

    scan_motor_ids();
    send_enable_once_from_scan();
    RCLCPP_INFO(
      get_logger(),
      "driver parameter write(Type18/Type22) is disabled in can_bridge_node; use maintenance tools only");
    RCLCPP_INFO(
      get_logger(),
      "tx policy initialized: tx_hz_default=%.3f per_motor=%zu",
      tx_policy_.tx_hz_default,
      tx_policy_.tx_hz_by_motor.size());
    RCLCPP_INFO(
      get_logger(),
      "home policy initialized: default(q_des=%.3f qd_des=%.3f kp=%.3f kd=%.3f tau_ff=%.3f), "
      "per_motor(q_des=%zu kp=%zu kd=%zu qmin=%zu qmax=%zu)",
      home_policy_.q_des_default,
      home_policy_.qd_des_default,
      home_policy_.kp_default,
      home_policy_.kd_default,
      home_policy_.tau_ff_default,
      home_policy_.q_des_by_motor.size(),
      home_policy_.kp_by_motor.size(),
      home_policy_.kd_by_motor.size(),
      home_policy_.q_limit_min_by_motor.size(),
      home_policy_.q_limit_max_by_motor.size());

    io_timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&CanBridgeNode::poll_can_frames, this));
  }

  ~CanBridgeNode() override
  {
    send_stop_once_on_shutdown();
    if (sock_fd_ >= 0) {
      close(sock_fd_);
      sock_fd_ = -1;
    }
  }

private:

  const MitRanges & ranges_for(const uint8_t motor_id) const
  {
    return (motor_id == rs03_id_) ? MIT_RS03 : MIT_RS02;
  }

  bool open_socketcan(const std::string & channel)
  {
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, channel.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "ioctl(SIOCGIFINDEX) failed for %s: %s", channel.c_str(), std::strerror(errno));
      return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind() failed for %s: %s", channel.c_str(), std::strerror(errno));
      return false;
    }

    const int flags = fcntl(sock_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      RCLCPP_ERROR(get_logger(), "fcntl(O_NONBLOCK) failed: %s", std::strerror(errno));
      return false;
    }
    return true;
  }

  bool send_ext_frame(const uint32_t arb_id, const std::array<uint8_t, 8> & data, const uint8_t dlc = 8)
  {
    if (sock_fd_ < 0) {
      return false;
    }
    struct can_frame frame {};
    frame.can_id = (arb_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data.data(), dlc);
    const ssize_t n = write(sock_fd_, &frame, sizeof(frame));
    return n == static_cast<ssize_t>(sizeof(frame));
  }

  std::optional<struct can_frame> recv_frame()
  {
    struct can_frame frame {};
    const ssize_t n = read(sock_fd_, &frame, sizeof(frame));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return std::nullopt;
      }
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "read() failed: %s", std::strerror(errno));
      return std::nullopt;
    }
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      return std::nullopt;
    }
    return frame;
  }

  struct CachedCommand
  {
    msgs::msg::MotorCMD cmd {};
    bool valid {false};
    bool has_sent {false};
    bool has_received_cmd {false};
    bool timeout_home_active {false};
    bool home_traj_active {false};
    float home_traj_start_q {0.0F};
    float home_traj_goal_q {0.0F};
    float home_traj_duration_sec {kHomeReturnDurationSec};
    std::chrono::steady_clock::time_point home_traj_start_tp {};
    std::chrono::steady_clock::time_point last_sent {};
    std::chrono::steady_clock::time_point last_received_cmd {};
  };

  struct TxPolicy
  {
    double tx_hz_default {500.0};
    std::unordered_map<int, double> tx_hz_by_motor {};
  };

  struct HomePolicy
  {
    double q_des_default {0.0};
    double qd_des_default {0.0};
    double kp_default {30.0};
    double kd_default {5.0};
    double tau_ff_default {0.0};
    std::unordered_map<int, double> q_des_by_motor {};
    std::unordered_map<int, double> kp_by_motor {};
    std::unordered_map<int, double> kd_by_motor {};
    std::unordered_map<int, double> q_limit_min_by_motor {};
    std::unordered_map<int, double> q_limit_max_by_motor {};
  };

  bool parse_numeric_by_motor_json(
    const std::string & text,
    std::unordered_map<int, double> & out,
    std::string & error) const
  {
    error.clear();
    out.clear();

    const auto first = std::find_if_not(
      text.begin(), text.end(),
      [](char c) {return std::isspace(static_cast<unsigned char>(c)) != 0;});
    if (first == text.end()) {
      return true;
    }
    const auto last = std::find_if_not(
      text.rbegin(), text.rend(),
      [](char c) {return std::isspace(static_cast<unsigned char>(c)) != 0;}).base();
    const std::string trimmed(first, last);
    if (trimmed == "{}") {
      return true;
    }
    if (trimmed.size() < 2 || trimmed.front() != '{' || trimmed.back() != '}') {
      error = "expected object string like {\"1\":0.0,\"2\":1.5}";
      return false;
    }

    const std::string inner = trimmed.substr(1, trimmed.size() - 2);
    static const std::regex entry_re(
      "\"([0-9]+)\"\\s*:\\s*([-+]?(?:[0-9]*\\.?[0-9]+)(?:[eE][-+]?[0-9]+)?)");

    std::string leftover;
    std::size_t cursor = 0;
    auto it = std::sregex_iterator(inner.begin(), inner.end(), entry_re);
    const auto end = std::sregex_iterator();
    for (; it != end; ++it) {
      const auto & match = *it;
      const auto start = static_cast<std::size_t>(match.position());
      const auto len = static_cast<std::size_t>(match.length());
      if (start > cursor) {
        leftover.append(inner, cursor, start - cursor);
      }
      cursor = start + len;

      try {
        const int motor_id = std::stoi(match[1].str());
        const double value = std::stod(match[2].str());
        if (motor_id < 0) {
          error = "motor_id must be >= 0";
          return false;
        }
        out[motor_id] = value;
      } catch (...) {
        error = "failed to parse numeric-by-motor json";
        return false;
      }
    }
    if (cursor < inner.size()) {
      leftover.append(inner, cursor, inner.size() - cursor);
    }

    leftover.erase(
      std::remove_if(
        leftover.begin(), leftover.end(),
        [](char c) {
          return std::isspace(static_cast<unsigned char>(c)) != 0 || c == ',';
        }),
      leftover.end());
    if (!leftover.empty()) {
      error = "invalid object content";
      return false;
    }
    return true;
  }

  bool parse_tx_hz_by_motor_json(
    const std::string & text,
    std::unordered_map<int, double> & out,
    std::string & error) const
  {
    if (!parse_numeric_by_motor_json(text, out, error)) {
      return false;
    }
    for (const auto & kv : out) {
      if (kv.second <= 0.0) {
        error = "motor_id must be >= 0 and hz must be > 0";
        return false;
      }
    }
    return true;
  }

  void apply_tx_policy(
    const double tx_hz_default,
    const std::unordered_map<int, double> & tx_hz_by_motor,
    const bool emit_log)
  {
    tx_policy_.tx_hz_default = tx_hz_default;
    tx_policy_.tx_hz_by_motor = tx_hz_by_motor;
    if (emit_log) {
      RCLCPP_INFO(
        get_logger(),
        "tx policy updated: tx_hz_default=%.3f per_motor=%zu",
        tx_policy_.tx_hz_default,
        tx_policy_.tx_hz_by_motor.size());
    }
  }

  void apply_home_policy(
    const double q_des_default,
    const double qd_des_default,
    const double kp_default,
    const double kd_default,
    const double tau_ff_default,
    const std::unordered_map<int, double> & q_des_by_motor,
    const std::unordered_map<int, double> & kp_by_motor,
    const std::unordered_map<int, double> & kd_by_motor,
    const std::unordered_map<int, double> & q_limit_min_by_motor,
    const std::unordered_map<int, double> & q_limit_max_by_motor,
    const bool emit_log)
  {
    home_policy_.q_des_default = q_des_default;
    home_policy_.qd_des_default = qd_des_default;
    home_policy_.kp_default = std::max(0.0, kp_default);
    home_policy_.kd_default = std::max(0.0, kd_default);
    home_policy_.tau_ff_default = tau_ff_default;
    home_policy_.q_des_by_motor = q_des_by_motor;
    home_policy_.kp_by_motor = kp_by_motor;
    home_policy_.kd_by_motor = kd_by_motor;
    home_policy_.q_limit_min_by_motor = q_limit_min_by_motor;
    home_policy_.q_limit_max_by_motor = q_limit_max_by_motor;

    for (auto & kv : home_policy_.kp_by_motor) {
      if (kv.second < 0.0) {
        RCLCPP_WARN(
          get_logger(),
          "home_kp_by_motor_json[%d]=%.3f is negative; clamped to 0.0",
          kv.first,
          kv.second);
        kv.second = 0.0;
      }
    }
    for (auto & kv : home_policy_.kd_by_motor) {
      if (kv.second < 0.0) {
        RCLCPP_WARN(
          get_logger(),
          "home_kd_by_motor_json[%d]=%.3f is negative; clamped to 0.0",
          kv.first,
          kv.second);
        kv.second = 0.0;
      }
    }

    if (emit_log) {
      RCLCPP_INFO(
        get_logger(),
        "home policy updated: default(q_des=%.3f qd_des=%.3f kp=%.3f kd=%.3f tau_ff=%.3f), "
        "per_motor(q_des=%zu kp=%zu kd=%zu qmin=%zu qmax=%zu)",
        home_policy_.q_des_default,
        home_policy_.qd_des_default,
        home_policy_.kp_default,
        home_policy_.kd_default,
        home_policy_.tau_ff_default,
        home_policy_.q_des_by_motor.size(),
        home_policy_.kp_by_motor.size(),
        home_policy_.kd_by_motor.size(),
        home_policy_.q_limit_min_by_motor.size(),
        home_policy_.q_limit_max_by_motor.size());
    }
  }

  double value_for_motor(
    const std::unordered_map<int, double> & by_motor,
    const uint8_t motor_id,
    const double default_value) const
  {
    const auto it = by_motor.find(static_cast<int>(motor_id));
    if (it == by_motor.end()) {
      return default_value;
    }
    return it->second;
  }

  std::pair<float, float> position_limits_for_motor(const uint8_t motor_id) const
  {
    const MitRanges & r = ranges_for(motor_id);
    const double qmin_cfg = value_for_motor(
      home_policy_.q_limit_min_by_motor, motor_id, static_cast<double>(r.p_min));
    const double qmax_cfg = value_for_motor(
      home_policy_.q_limit_max_by_motor, motor_id, static_cast<double>(r.p_max));

    float q_min = static_cast<float>(std::max(static_cast<double>(r.p_min), qmin_cfg));
    float q_max = static_cast<float>(std::min(static_cast<double>(r.p_max), qmax_cfg));
    if (q_min > q_max) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "motor_id=%u invalid q-limit config (min=%.3f max=%.3f): using MIT range [%.3f, %.3f]",
        motor_id,
        qmin_cfg,
        qmax_cfg,
        static_cast<double>(r.p_min),
        static_cast<double>(r.p_max));
      q_min = r.p_min;
      q_max = r.p_max;
    }
    return {q_min, q_max};
  }

  float wrap_or_clamp_current_q_for_home(
    const float current_q,
    const float q_min,
    const float q_max) const
  {
    const float period = 2.0F * static_cast<float>(M_PI);
    const float span = q_max - q_min;
    // For 2*pi windows (e.g. [-pi, pi]), wrap only when the reported state is far out of range.
    // If it is just slightly outside (quantization/noise near boundary), clamp to keep sign continuity.
    if (std::fabs(span - period) <= 1e-3F) {
      if (current_q >= q_min && current_q <= q_max) {
        return current_q;
      }

      constexpr float kNearBoundarySnapRad = 0.2F;
      const float below = (current_q < q_min) ? (q_min - current_q) : 0.0F;
      const float above = (current_q > q_max) ? (current_q - q_max) : 0.0F;
      const float out_of_range = std::max(below, above);
      if (out_of_range <= kNearBoundarySnapRad) {
        return clamp(current_q, q_min, q_max);
      }

      float wrapped = q_min + std::fmod(current_q - q_min, period);
      if (wrapped < q_min) {
        wrapped += period;
      }
      if (wrapped > q_max) {
        wrapped -= period;
      }
      return clamp(wrapped, q_min, q_max);
    }
    return clamp(current_q, q_min, q_max);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result {};
    result.successful = true;

    double next_tx_hz_default = tx_policy_.tx_hz_default;
    std::string next_tx_hz_by_motor_json = tx_hz_by_motor_json_;
    bool tx_default_changed = false;
    bool tx_json_changed = false;
    double next_home_q_des = home_q_des_;
    double next_home_qd_des = home_qd_des_;
    double next_home_kp = home_kp_;
    double next_home_kd = home_kd_;
    double next_home_tau_ff = home_tau_ff_;
    std::string next_home_q_des_by_motor_json = home_q_des_by_motor_json_;
    std::string next_home_kp_by_motor_json = home_kp_by_motor_json_;
    std::string next_home_kd_by_motor_json = home_kd_by_motor_json_;
    std::string next_q_limit_min_by_motor_json = q_limit_min_by_motor_json_;
    std::string next_q_limit_max_by_motor_json = q_limit_max_by_motor_json_;
    bool home_scalar_changed = false;
    bool home_map_changed = false;

    auto param_to_double = [](const rclcpp::Parameter & param, double & out) -> bool {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        out = param.as_double();
        return true;
      }
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        out = static_cast<double>(param.as_int());
        return true;
      }
      return false;
    };

    for (const auto & param : params) {
      const std::string & name = param.get_name();
      if (name == "tx_hz_default" || name == "default_tx_hz") {
        double hz = 0.0;
        if (!param_to_double(param, hz)) {
          result.successful = false;
          result.reason = name + " must be numeric";
          return result;
        }
        if (hz <= 0.0) {
          result.successful = false;
          result.reason = name + " must be > 0";
          return result;
        }
        next_tx_hz_default = hz;
        tx_default_changed = true;
        continue;
      }

      if (name == "tx_hz_by_motor_json") {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "tx_hz_by_motor_json must be string";
          return result;
        }
        next_tx_hz_by_motor_json = param.as_string();
        tx_json_changed = true;
        continue;
      }

      if (
        name == "home_q_des" || name == "home_qd_des" || name == "home_kp" ||
        name == "home_kd" || name == "home_tau_ff")
      {
        double value = 0.0;
        if (!param_to_double(param, value)) {
          result.successful = false;
          result.reason = name + " must be numeric";
          return result;
        }
        if ((name == "home_kp" || name == "home_kd") && value < 0.0) {
          result.successful = false;
          result.reason = name + " must be >= 0";
          return result;
        }
        if (name == "home_q_des") {
          next_home_q_des = value;
        } else if (name == "home_qd_des") {
          next_home_qd_des = value;
        } else if (name == "home_kp") {
          next_home_kp = value;
        } else if (name == "home_kd") {
          next_home_kd = value;
        } else {
          next_home_tau_ff = value;
        }
        home_scalar_changed = true;
        continue;
      }

      if (
        name == "home_q_des_by_motor_json" || name == "home_kp_by_motor_json" ||
        name == "home_kd_by_motor_json" || name == "q_limit_min_by_motor_json" ||
        name == "q_limit_max_by_motor_json")
      {
        if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = name + " must be string";
          return result;
        }
        const std::string value = param.as_string();
        if (name == "home_q_des_by_motor_json") {
          next_home_q_des_by_motor_json = value;
        } else if (name == "home_kp_by_motor_json") {
          next_home_kp_by_motor_json = value;
        } else if (name == "home_kd_by_motor_json") {
          next_home_kd_by_motor_json = value;
        } else if (name == "q_limit_min_by_motor_json") {
          next_q_limit_min_by_motor_json = value;
        } else {
          next_q_limit_max_by_motor_json = value;
        }
        home_map_changed = true;
        continue;
      }
    }

    const bool tx_changed = tx_default_changed || tx_json_changed;
    const bool home_changed = home_scalar_changed || home_map_changed;
    if (!tx_changed && !home_changed) {
      return result;
    }

    std::unordered_map<int, double> parsed_by_motor = tx_policy_.tx_hz_by_motor;
    if (tx_json_changed) {
      std::string parse_error;
      if (!parse_tx_hz_by_motor_json(next_tx_hz_by_motor_json, parsed_by_motor, parse_error)) {
        result.successful = false;
        result.reason = "tx_hz_by_motor_json parse failed: " + parse_error;
        return result;
      }
    }

    std::unordered_map<int, double> parsed_home_q_des_by_motor = home_policy_.q_des_by_motor;
    std::unordered_map<int, double> parsed_home_kp_by_motor = home_policy_.kp_by_motor;
    std::unordered_map<int, double> parsed_home_kd_by_motor = home_policy_.kd_by_motor;
    std::unordered_map<int, double> parsed_q_limit_min_by_motor = home_policy_.q_limit_min_by_motor;
    std::unordered_map<int, double> parsed_q_limit_max_by_motor = home_policy_.q_limit_max_by_motor;
    if (home_map_changed) {
      std::string parse_error;
      if (!parse_numeric_by_motor_json(
          next_home_q_des_by_motor_json, parsed_home_q_des_by_motor, parse_error))
      {
        result.successful = false;
        result.reason = "home_q_des_by_motor_json parse failed: " + parse_error;
        return result;
      }
      if (!parse_numeric_by_motor_json(
          next_home_kp_by_motor_json, parsed_home_kp_by_motor, parse_error))
      {
        result.successful = false;
        result.reason = "home_kp_by_motor_json parse failed: " + parse_error;
        return result;
      }
      if (!parse_numeric_by_motor_json(
          next_home_kd_by_motor_json, parsed_home_kd_by_motor, parse_error))
      {
        result.successful = false;
        result.reason = "home_kd_by_motor_json parse failed: " + parse_error;
        return result;
      }
      if (!parse_numeric_by_motor_json(
          next_q_limit_min_by_motor_json, parsed_q_limit_min_by_motor, parse_error))
      {
        result.successful = false;
        result.reason = "q_limit_min_by_motor_json parse failed: " + parse_error;
        return result;
      }
      if (!parse_numeric_by_motor_json(
          next_q_limit_max_by_motor_json, parsed_q_limit_max_by_motor, parse_error))
      {
        result.successful = false;
        result.reason = "q_limit_max_by_motor_json parse failed: " + parse_error;
        return result;
      }

      for (const auto & kv : parsed_home_kp_by_motor) {
        if (kv.second < 0.0) {
          result.successful = false;
          result.reason = "home_kp_by_motor_json must be >= 0";
          return result;
        }
      }
      for (const auto & kv : parsed_home_kd_by_motor) {
        if (kv.second < 0.0) {
          result.successful = false;
          result.reason = "home_kd_by_motor_json must be >= 0";
          return result;
        }
      }
      for (const auto & kv : parsed_q_limit_min_by_motor) {
        const auto it = parsed_q_limit_max_by_motor.find(kv.first);
        if (it != parsed_q_limit_max_by_motor.end() && kv.second > it->second) {
          result.successful = false;
          result.reason = "q_limit_min_by_motor_json must be <= q_limit_max_by_motor_json";
          return result;
        }
      }
    }

    if (tx_changed) {
      apply_tx_policy(next_tx_hz_default, parsed_by_motor, true);
      if (tx_default_changed) {
        tx_hz_default_ = next_tx_hz_default;
        default_tx_hz_ = next_tx_hz_default;  // Keep legacy alias in sync.
      }
      if (tx_json_changed) {
        tx_hz_by_motor_json_ = next_tx_hz_by_motor_json;
      }
    }

    if (home_changed) {
      apply_home_policy(
        next_home_q_des,
        next_home_qd_des,
        next_home_kp,
        next_home_kd,
        next_home_tau_ff,
        parsed_home_q_des_by_motor,
        parsed_home_kp_by_motor,
        parsed_home_kd_by_motor,
        parsed_q_limit_min_by_motor,
        parsed_q_limit_max_by_motor,
        true);
      home_q_des_ = next_home_q_des;
      home_qd_des_ = next_home_qd_des;
      home_kp_ = next_home_kp;
      home_kd_ = next_home_kd;
      home_tau_ff_ = next_home_tau_ff;
      if (home_map_changed) {
        home_q_des_by_motor_json_ = next_home_q_des_by_motor_json;
        home_kp_by_motor_json_ = next_home_kp_by_motor_json;
        home_kd_by_motor_json_ = next_home_kd_by_motor_json;
        q_limit_min_by_motor_json_ = next_q_limit_min_by_motor_json;
        q_limit_max_by_motor_json_ = next_q_limit_max_by_motor_json;
      }
    }
    return result;
  }

  double tx_hz_for_motor(const uint8_t motor_id) const
  {
    const auto it = tx_policy_.tx_hz_by_motor.find(static_cast<int>(motor_id));
    if (it != tx_policy_.tx_hz_by_motor.end() && it->second > 0.0) {
      return it->second;
    }
    return (tx_policy_.tx_hz_default > 0.0) ? tx_policy_.tx_hz_default : default_tx_hz_;
  }

  bool send_mit_command(
    const msgs::msg::MotorCMD & cmd,
    const bool allow_periodic_lift_for_2pi_limit = false)
  {
    const uint8_t motor_id = cmd.motor_id;
    const MitRanges & r = ranges_for(motor_id);
    const auto limits = position_limits_for_motor(motor_id);
    const float q_des_in_window = clamp(cmd.q_des, limits.first, limits.second);
    if (std::fabs(q_des_in_window - cmd.q_des) > 1e-6F) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "motor_id=%u q_des clamp applied: %.3f -> %.3f (limit=[%.3f, %.3f])",
        motor_id,
        static_cast<double>(cmd.q_des),
        static_cast<double>(q_des_in_window),
        static_cast<double>(limits.first),
        static_cast<double>(limits.second));
    }

    float q_des_for_encode = q_des_in_window;
    const float period = 2.0F * static_cast<float>(M_PI);
    const float span = limits.second - limits.first;
    if (allow_periodic_lift_for_2pi_limit && std::fabs(span - period) <= 1e-3F) {
      const auto rt_it = runtime_by_motor_.find(motor_id);
      if (rt_it != runtime_by_motor_.end() && rt_it->second.has_state) {
        const float q_des_lifted = nearest_periodic_target(
          q_des_in_window,
          rt_it->second.last_state.q,
          period,
          r.p_min,
          r.p_max);
        q_des_for_encode = clamp(q_des_lifted, r.p_min, r.p_max);
        if (std::fabs(q_des_for_encode - q_des_in_window) > 1e-4F) {
          RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "motor_id=%u periodic home lift applied: %.3f -> %.3f (state_q=%.3f)",
            motor_id,
            static_cast<double>(q_des_in_window),
            static_cast<double>(q_des_for_encode),
            static_cast<double>(rt_it->second.last_state.q));
        }
      }
    }

    const uint16_t t_u16 = float_to_u16(cmd.tau_ff, r.t_min, r.t_max);
    const uint16_t q_u16 = float_to_u16(q_des_for_encode, r.p_min, r.p_max);
    const uint16_t qd_u16 = float_to_u16(cmd.qd_des, r.v_min, r.v_max);
    const uint16_t kp_u16 = float_to_u16(cmd.kp, r.kp_min, r.kp_max);
    const uint16_t kd_u16 = float_to_u16(cmd.kd, r.kd_min, r.kd_max);

    const uint32_t arb = pack_ext_id(TYPE01_OPERATION_CONTROL, t_u16, motor_id);
    std::array<uint8_t, 8> data {};
    data[0] = static_cast<uint8_t>((q_u16 >> 8) & 0xFF);
    data[1] = static_cast<uint8_t>(q_u16 & 0xFF);
    data[2] = static_cast<uint8_t>((qd_u16 >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(qd_u16 & 0xFF);
    data[4] = static_cast<uint8_t>((kp_u16 >> 8) & 0xFF);
    data[5] = static_cast<uint8_t>(kp_u16 & 0xFF);
    data[6] = static_cast<uint8_t>((kd_u16 >> 8) & 0xFF);
    data[7] = static_cast<uint8_t>(kd_u16 & 0xFF);

    return send_ext_frame(arb, data);
  }

  bool send_enable_frame(const uint8_t motor_id)
  {
    std::array<uint8_t, 8> data {};
    const uint32_t arb = pack_ext_id(TYPE03_ENABLE, host_id_, motor_id);
    return send_ext_frame(arb, data);
  }

  bool send_stop_frame(const uint8_t motor_id, const bool clear_fault = false)
  {
    std::array<uint8_t, 8> data {};
    data[0] = clear_fault ? 1U : 0U;
    const uint32_t arb = pack_ext_id(TYPE04_STOP, host_id_, motor_id);
    return send_ext_frame(arb, data);
  }

  void send_stop_once_on_shutdown()
  {
    if (sock_fd_ < 0) {
      return;
    }

    std::set<uint8_t> target_motor_ids = discovered_motor_ids_;
    for (const auto & kv : latest_cmd_by_motor_) {
      target_motor_ids.insert(kv.first);
    }
    for (const auto & kv : runtime_by_motor_) {
      target_motor_ids.insert(kv.first);
    }

    if (target_motor_ids.empty()) {
      RCLCPP_WARN(get_logger(), "shutdown Type04 skipped: no known motor id");
      return;
    }

    int ok = 0;
    int fail = 0;
    for (const uint8_t motor_id : target_motor_ids) {
      if (send_stop_frame(motor_id, false)) {
        ++ok;
      } else {
        ++fail;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    RCLCPP_INFO(
      get_logger(),
      "shutdown Type04 sent once: targets=%zu ok=%d fail=%d",
      target_motor_ids.size(),
      ok,
      fail);
  }

  void send_enable_once_from_scan()
  {
    if (discovered_motor_ids_.empty()) {
      RCLCPP_WARN(get_logger(), "startup Type03 skipped: no discovered id from Type00 scan");
      return;
    }

    int ok = 0;
    int fail = 0;
    for (const uint8_t motor_id : discovered_motor_ids_) {
      MotorRuntime & rt = runtime_by_motor_[motor_id];
      if (send_enable_frame(motor_id)) {
        rt.enable_sent = true;
        rt.last_enable_sent = std::chrono::steady_clock::now();
        ++ok;
      } else {
        ++fail;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    RCLCPP_INFO(
      get_logger(),
      "startup Type03 sent once: targets=%zu ok=%d fail=%d",
      discovered_motor_ids_.size(),
      ok,
      fail);
  }

  struct MotorRuntime
  {
    bool enable_sent {false};
    bool ready_for_control {false};  // Type03 was sent and Type02 feedback observed afterwards.
    bool has_state {false};
    msgs::msg::MotorState last_state {};
    std::chrono::steady_clock::time_point last_enable_sent {};
  };

  bool ensure_motor_ready_for_control(const uint8_t motor_id)
  {
    MotorRuntime & rt = runtime_by_motor_[motor_id];
    if (rt.ready_for_control) {
      return true;
    }

      RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "waiting TYPE02 before applying /motor_cmd_array for motor_id=%u", motor_id);
    return false;
  }

  msgs::msg::MotorCMD make_home_command(const uint8_t motor_id) const
  {
    msgs::msg::MotorCMD home {};
    home.motor_id = motor_id;
    home.q_des = home_q_des_for_motor(motor_id);
    home.qd_des = static_cast<float>(home_policy_.qd_des_default);
    home.kp = home_kp_for_motor(motor_id);
    home.kd = home_kd_for_motor(motor_id);
    home.tau_ff = static_cast<float>(home_policy_.tau_ff_default);
    return home;
  }

  float home_q_des_for_motor(const uint8_t motor_id) const
  {
    return static_cast<float>(
      value_for_motor(home_policy_.q_des_by_motor, motor_id, home_policy_.q_des_default));
  }

  float home_kp_for_motor(const uint8_t motor_id) const
  {
    return static_cast<float>(
      value_for_motor(home_policy_.kp_by_motor, motor_id, home_policy_.kp_default));
  }

  float home_kd_for_motor(const uint8_t motor_id) const
  {
    return static_cast<float>(
      value_for_motor(home_policy_.kd_by_motor, motor_id, home_policy_.kd_default));
  }

  float aligned_home_q_des(const uint8_t motor_id, const float current_q) const
  {
    const auto limits = position_limits_for_motor(motor_id);
    const float q_base = home_q_des_for_motor(motor_id);
    return nearest_periodic_target(
      q_base,
      current_q,
      2.0F * static_cast<float>(M_PI),
      limits.first,
      limits.second);
  }

  void start_home_trajectory(
    const uint8_t motor_id,
    const float current_q,
    const std::chrono::steady_clock::time_point & now_tp,
    const char * reason)
  {
    CachedCommand & cached = latest_cmd_by_motor_[motor_id];
    const auto limits = position_limits_for_motor(motor_id);
    const float q_base = home_q_des_for_motor(motor_id);
    const float q_start = wrap_or_clamp_current_q_for_home(current_q, limits.first, limits.second);
    const float q_goal = aligned_home_q_des(motor_id, q_start);
    const float abs_dq = std::fabs(q_goal - q_start);

    cached.home_traj_start_q = q_start;
    cached.home_traj_goal_q = q_goal;
    cached.home_traj_duration_sec = kHomeReturnDurationSec;
    cached.home_traj_start_tp = now_tp;
    cached.home_traj_active = abs_dq > kHomeEpsilon;

    if (cached.home_traj_active) {
      RCLCPP_INFO(
        get_logger(),
        "motor_id=%u home trajectory started (%s, q_base=%.3f q_start=%.3f q_goal=%.3f duration=%.3f s)",
        motor_id,
        reason,
        static_cast<double>(q_base),
        static_cast<double>(q_start),
        static_cast<double>(q_goal),
        static_cast<double>(cached.home_traj_duration_sec));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "motor_id=%u home trajectory skipped (%s, |dq|<=%.1e, q_base=%.3f q=%.3f)",
        motor_id,
        reason,
        static_cast<double>(kHomeEpsilon),
        static_cast<double>(q_base),
        static_cast<double>(q_goal));
    }
  }

  float sample_home_trajectory_q_des(
    CachedCommand & cached,
    const std::chrono::steady_clock::time_point & now_tp) const
  {
    if (!cached.home_traj_active || cached.home_traj_duration_sec <= 0.0F) {
      return cached.home_traj_goal_q;
    }

    const double elapsed_sec = std::chrono::duration<double>(now_tp - cached.home_traj_start_tp).count();
    const float alpha = clamp(
      static_cast<float>(elapsed_sec / static_cast<double>(cached.home_traj_duration_sec)),
      0.0F,
      1.0F);
    const float q_des = cached.home_traj_start_q + alpha * (cached.home_traj_goal_q - cached.home_traj_start_q);
    if (alpha >= 1.0F) {
      cached.home_traj_active = false;
      return cached.home_traj_goal_q;
    }
    return q_des;
  }

  void start_home_stream_for_ready_motor(const uint8_t motor_id, const float current_q)
  {
    CachedCommand & cached = latest_cmd_by_motor_[motor_id];
    if (cached.has_received_cmd) {
      return;
    }

    cached.cmd = make_home_command(motor_id);
    const auto now_tp = std::chrono::steady_clock::now();
    start_home_trajectory(motor_id, current_q, now_tp, "pre-home");
    cached.cmd.q_des = cached.home_traj_goal_q;
    cached.cmd.stamp = to_builtin_time(now());
    cached.valid = true;
    cached.has_sent = false;
    cached.timeout_home_active = false;
    RCLCPP_INFO(
      get_logger(),
      "motor_id=%u home Type01 stream armed after ready (current_q_raw=%.3f q_start=%.3f q_goal=%.3f duration=%.3f s)",
      motor_id,
      static_cast<double>(current_q),
      static_cast<double>(cached.home_traj_start_q),
      static_cast<double>(cached.home_traj_goal_q),
      static_cast<double>(cached.home_traj_duration_sec));
  }

  void on_cmd_array(const msgs::msg::MotorCMDArray::SharedPtr msg)
  {
    if (msg->commands.empty()) {
      return;
    }
    const auto now_tp = std::chrono::steady_clock::now();
    for (const auto & cmd : msg->commands) {
      CachedCommand & cached = latest_cmd_by_motor_[cmd.motor_id];
      cached.cmd = cmd;
      cached.valid = true;
      cached.has_received_cmd = true;
      cached.last_received_cmd = now_tp;
      cached.timeout_home_active = false;
      cached.home_traj_active = false;
    }
  }

  void dispatch_cached_commands()
  {
    const auto now_tp = std::chrono::steady_clock::now();
    for (auto & kv : latest_cmd_by_motor_) {
      const uint8_t motor_id = kv.first;
      CachedCommand & cached = kv.second;
      if (!cached.valid) {
        continue;
      }

      const bool is_pre_home_stream = !cached.has_received_cmd;
      if (!is_pre_home_stream && !ensure_motor_ready_for_control(motor_id)) {
        continue;  // Hold external /motor_cmd_array until Type02 is observed.
      }

      const double tx_hz = tx_hz_for_motor(motor_id);
      if (tx_hz <= 0.0) {
        continue;
      }

      const auto period = std::chrono::duration<double>(1.0 / tx_hz);
      if (cached.has_sent) {
        const auto elapsed = now_tp - cached.last_sent;
        if (elapsed < period) {
          continue;
        }
      }

      bool cmd_timed_out = false;
      if (cmd_timeout_ms_ > 0 && cached.has_received_cmd) {
        const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now_tp - cached.last_received_cmd).count();
        cmd_timed_out = age_ms >= cmd_timeout_ms_;
      }

      msgs::msg::MotorCMD outgoing = cached.cmd;
      if (is_pre_home_stream) {
        outgoing = make_home_command(motor_id);
        outgoing.q_des = sample_home_trajectory_q_des(cached, now_tp);
        outgoing.stamp = to_builtin_time(now());
      } else if (cmd_timed_out) {
        outgoing = make_home_command(motor_id);
        const MotorRuntime & rt = runtime_by_motor_[motor_id];
        if (!cached.timeout_home_active) {
          cached.timeout_home_active = true;
          if (rt.has_state) {
            start_home_trajectory(motor_id, rt.last_state.q, now_tp, "timeout-home");
          } else {
            const auto limits = position_limits_for_motor(motor_id);
            const float q_home = clamp(home_q_des_for_motor(motor_id), limits.first, limits.second);
            cached.home_traj_active = false;
            cached.home_traj_start_q = q_home;
            cached.home_traj_goal_q = q_home;
            cached.home_traj_duration_sec = kHomeReturnDurationSec;
            cached.home_traj_start_tp = now_tp;
          }
          RCLCPP_WARN(
            get_logger(),
            "motor_id=%u cmd timeout (%d ms): switching to home trajectory "
            "(q_start=%.3f q_goal=%.3f duration=%.3f s)",
            motor_id,
            cmd_timeout_ms_,
            static_cast<double>(cached.home_traj_start_q),
            static_cast<double>(cached.home_traj_goal_q),
            static_cast<double>(cached.home_traj_duration_sec));
        }
        outgoing.q_des = sample_home_trajectory_q_des(cached, now_tp);
        outgoing.stamp = to_builtin_time(now());
      } else if (cached.timeout_home_active) {
        cached.timeout_home_active = false;
        cached.home_traj_active = false;
        RCLCPP_INFO(
          get_logger(),
          "motor_id=%u cmd restored: leaving home timeout mode (q_goal=%.3f duration=%.3f s)",
          motor_id,
          static_cast<double>(cached.home_traj_goal_q),
          static_cast<double>(cached.home_traj_duration_sec));
      }

      const bool allow_periodic_lift_for_2pi_limit = is_pre_home_stream || cmd_timed_out;
      if (!send_mit_command(outgoing, allow_periodic_lift_for_2pi_limit)) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "failed to send TYPE01 command frame");
      }
      cached.has_sent = true;
      cached.last_sent = now_tp;
    }
  }

  void scan_motor_ids()
  {
    if (scan_min_id_ > scan_max_id_) {
      std::swap(scan_min_id_, scan_max_id_);
    }

    std::array<uint8_t, 8> zeros {};
    for (int id = scan_min_id_; id <= scan_max_id_; ++id) {
      const uint32_t arb = pack_ext_id(TYPE00_GET_DEVICE_ID, host_id_, static_cast<uint8_t>(id));
      (void)send_ext_frame(arb, zeros);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(scan_wait_ms_);
    std::set<int> found;
    while (std::chrono::steady_clock::now() < deadline) {
      auto frame_opt = recv_frame();
      if (!frame_opt.has_value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      const auto & frame = frame_opt.value();
      if ((frame.can_id & CAN_EFF_FLAG) == 0) {
        continue;
      }
      const uint32_t arb = frame.can_id & CAN_EFF_MASK;
      const uint8_t type = comm_type_from_arb(arb);
      if (type != TYPE00_GET_DEVICE_ID) {
        continue;
      }
      int motor_id = -1;
      const uint8_t id1 = data1_from_arb(arb);
      const uint8_t id2 = static_cast<uint8_t>(data2_from_arb(arb) & 0xFF);
      // Manual form: Type00 reply uses bit7~0 = 0xFE and bit15~8 = motor id.
      if (id1 == 0xFE) {
        motor_id = static_cast<int>(id2);
      } else if (id2 == 0xFE) {
        // Be tolerant to swapped encodings.
        motor_id = static_cast<int>(id1);
      } else {
        // Fallback for legacy format.
        motor_id = static_cast<int>(id1);
      }
      if (motor_id >= scan_min_id_ && motor_id <= scan_max_id_) {
        found.insert(motor_id);
      }
    }

    discovered_motor_ids_.clear();
    for (const int id : found) {
      discovered_motor_ids_.insert(static_cast<uint8_t>(id));
    }

    if (discovered_motor_ids_.empty()) {
      RCLCPP_WARN(
        get_logger(), "Type00 scan found no motor in id range [%d, %d]", scan_min_id_, scan_max_id_);
      return;
    }

    std::string ids;
    for (const uint8_t id : discovered_motor_ids_) {
      if (!ids.empty()) {
        ids += ",";
      }
      ids += std::to_string(static_cast<int>(id));
    }
    RCLCPP_INFO(get_logger(), "Type00 scan discovered motor ids: [%s]", ids.c_str());
  }

  void poll_can_frames()
  {
    // Per tick, keep only the last sample/event per motor_id.
    std::map<uint8_t, msgs::msg::MotorState> state_batch_by_motor;
    std::map<uint8_t, msgs::msg::MotorError> error_batch_by_motor;

    int processed = 0;
    while (processed < rx_max_frames_per_tick_) {
      auto frame_opt = recv_frame();
      if (!frame_opt.has_value()) {
        break;
      }
      ++processed;
      const auto & frame = frame_opt.value();

      if ((frame.can_id & CAN_EFF_FLAG) == 0) {
        continue;
      }
      if (frame.can_dlc != 8) {
        continue;
      }
      const uint32_t arb = frame.can_id & CAN_EFF_MASK;
      if (comm_type_from_arb(arb) != TYPE02_FEEDBACK) {
        continue;  // Type02 only
      }

      const uint16_t data2 = data2_from_arb(arb);
      const uint8_t motor_id = static_cast<uint8_t>(data2 & 0xFF);
      const uint8_t flags = static_cast<uint8_t>((data2 >> 8) & 0xFF);
      const uint8_t mode_status = static_cast<uint8_t>((flags >> 6) & 0x03);
      const uint8_t fault_bits = static_cast<uint8_t>(flags & 0x3F);

      const uint16_t q_raw = be_u16(frame.data[0], frame.data[1]);
      const uint16_t qd_raw = be_u16(frame.data[2], frame.data[3]);
      const uint16_t tau_raw = be_u16(frame.data[4], frame.data[5]);
      const uint16_t temp_raw = be_u16(frame.data[6], frame.data[7]);
      const MitRanges & r = ranges_for(motor_id);
      const auto stamp = to_builtin_time(now());

      msgs::msg::MotorState st {};
      st.stamp = stamp;
      st.motor_id = motor_id;
      st.q = u16_to_float(q_raw, r.p_min, r.p_max);
      st.qd = u16_to_float(qd_raw, r.v_min, r.v_max);
      st.tau = u16_to_float(tau_raw, r.t_min, r.t_max);
      st.temp_c = static_cast<float>(temp_raw) / 10.0F;
      state_batch_by_motor[motor_id] = st;

      MotorRuntime & rt = runtime_by_motor_[motor_id];
      rt.has_state = true;
      rt.last_state = st;
      if (!rt.ready_for_control && rt.enable_sent) {
        if (mode_status == 2U) {
          rt.ready_for_control = true;
          RCLCPP_INFO(get_logger(), "motor_id=%u ready: TYPE02 run-mode observed after TYPE03", motor_id);
          start_home_stream_for_ready_motor(motor_id, st.q);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "motor_id=%u waiting run mode TYPE02 after TYPE03 (mode_status=%u)",
            motor_id,
            mode_status);
        }
      }

      const uint8_t prev_fault = last_fault_bits_[motor_id];
      const bool changed = (prev_fault != fault_bits);
      last_fault_bits_[motor_id] = fault_bits;
      if (!changed) {
        continue;
      }

      msgs::msg::MotorError err {};
      err.stamp = stamp;
      err.motor_id = motor_id;
      err.mode_status = mode_status;
      err.prev_fault_bits = prev_fault;
      err.fault_bits = fault_bits;
      if (prev_fault == 0 && fault_bits != 0) {
        err.event = msgs::msg::MotorError::EVENT_RAISED;
      } else if (prev_fault != 0 && fault_bits == 0) {
        err.event = msgs::msg::MotorError::EVENT_CLEARED;
      } else {
        err.event = msgs::msg::MotorError::EVENT_CHANGED;
      }
      error_batch_by_motor[motor_id] = err;
    }

    if (!state_batch_by_motor.empty()) {
      msgs::msg::MotorStateArray batch {};
      batch.stamp = to_builtin_time(now());
      batch.states.reserve(state_batch_by_motor.size());
      for (const auto & kv : state_batch_by_motor) {
        batch.states.push_back(kv.second);
      }
      state_array_pub_->publish(batch);
    }

    if (!error_batch_by_motor.empty()) {
      msgs::msg::MotorErrorArray batch {};
      batch.stamp = to_builtin_time(now());
      batch.errors.reserve(error_batch_by_motor.size());
      for (const auto & kv : error_batch_by_motor) {
        batch.errors.push_back(kv.second);
      }
      error_array_pub_->publish(batch);
    }

    // Rx is event-driven; Tx is rate-controlled per motor using latest cached command.
    dispatch_cached_commands();
  }

  int sock_fd_{-1};
  std::string channel_;
  uint8_t host_id_{0xFD};
  uint8_t rs03_id_{2};
  int scan_min_id_{1};
  int scan_max_id_{10};
  int scan_wait_ms_{200};
  int rx_max_frames_per_tick_{100};
  double default_tx_hz_{500.0};
  double tx_hz_default_{500.0};
  std::string tx_hz_by_motor_json_{"{}"};
  int cmd_timeout_ms_{100};
  double home_q_des_{0.0};
  std::string home_q_des_by_motor_json_{kDefaultHomeQDesByMotorJson};
  double home_qd_des_{0.0};
  double home_kp_{30.0};
  std::string home_kp_by_motor_json_{kDefaultHomeKpByMotorJson};
  double home_kd_{5.0};
  std::string home_kd_by_motor_json_{kDefaultHomeKdByMotorJson};
  double home_tau_ff_{0.0};
  std::string q_limit_min_by_motor_json_{kDefaultQLimitMinByMotorJson};
  std::string q_limit_max_by_motor_json_{kDefaultQLimitMaxByMotorJson};

  std::unordered_map<uint8_t, uint8_t> last_fault_bits_;
  std::unordered_map<uint8_t, CachedCommand> latest_cmd_by_motor_;
  std::unordered_map<uint8_t, MotorRuntime> runtime_by_motor_;
  std::set<uint8_t> discovered_motor_ids_;
  TxPolicy tx_policy_;
  HomePolicy home_policy_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rclcpp::Publisher<msgs::msg::MotorStateArray>::SharedPtr state_array_pub_;
  rclcpp::Publisher<msgs::msg::MotorErrorArray>::SharedPtr error_array_pub_;
  rclcpp::Subscription<msgs::msg::MotorCMDArray>::SharedPtr cmd_array_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
