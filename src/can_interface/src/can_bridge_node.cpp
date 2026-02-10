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
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
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

#include "msgs/msg/motor_cmd.hpp"
#include "msgs/msg/motor_error.hpp"
#include "msgs/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr uint8_t TYPE00_GET_DEVICE_ID = 0x00;
constexpr uint8_t TYPE01_OPERATION_CONTROL = 0x01;
constexpr uint8_t TYPE02_FEEDBACK = 0x02;
constexpr uint8_t TYPE03_ENABLE = 0x03;
constexpr uint8_t TYPE04_STOP = 0x04;
constexpr uint8_t TYPE_MASK = 0x1F;

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
    rx_max_frames_per_tick_ = declare_parameter<int>("rx_max_frames_per_tick", 128);
    default_tx_hz_ = declare_parameter<double>("default_tx_hz", 500.0);
    gate_reload_ms_ = declare_parameter<int>("gate_reload_ms", 50);
    enable_retry_ms_ = declare_parameter<int>("enable_retry_ms", 100);
    stop_on_shutdown_ = declare_parameter<bool>("stop_on_shutdown", true);
    home_on_shutdown_ = declare_parameter<bool>("home_on_shutdown", true);
    stop_clear_fault_ = declare_parameter<bool>("stop_clear_fault", false);
    shutdown_home_duration_ms_ = declare_parameter<int>("shutdown_home_duration_ms", 400);
    shutdown_home_hz_ = declare_parameter<double>("shutdown_home_hz", 200.0);
    home_q_des_ = declare_parameter<double>("home_q_des", 0.0);
    home_qd_des_ = declare_parameter<double>("home_qd_des", 0.0);
    home_kp_ = declare_parameter<double>("home_kp", 8.0);
    home_kd_ = declare_parameter<double>("home_kd", 0.6);
    home_tau_ff_ = declare_parameter<double>("home_tau_ff", 0.0);
    control_gate_state_file_ = declare_parameter<std::string>(
      "control_gate_state_file",
      default_gate_state_path());
    if (default_tx_hz_ <= 0.0) {
      default_tx_hz_ = 500.0;
    }
    if (gate_reload_ms_ <= 0) {
      gate_reload_ms_ = 50;
    }
    if (enable_retry_ms_ <= 0) {
      enable_retry_ms_ = 100;
    }
    if (shutdown_home_duration_ms_ < 0) {
      shutdown_home_duration_ms_ = 0;
    }
    if (shutdown_home_hz_ <= 0.0) {
      shutdown_home_hz_ = 200.0;
    }

    if (!open_socketcan(channel_)) {
      throw std::runtime_error("failed to open socketcan channel: " + channel_);
    }

    const auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
    const auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();

    state_pub_ = create_publisher<msgs::msg::MotorState>("/motor_state", qos_best_effort);
    error_pub_ = create_publisher<msgs::msg::MotorError>("/motor_error", qos_reliable);
    cmd_sub_ = create_subscription<msgs::msg::MotorCMD>(
      "/motor_cmd",
      qos_best_effort,
      std::bind(&CanBridgeNode::on_cmd, this, std::placeholders::_1));

    scan_motor_ids();
    reload_gate_config(true);
    RCLCPP_INFO(
      get_logger(),
      "driver parameter write(Type18/Type22) is disabled in can_bridge_node; use maintenance tools only");

    io_timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&CanBridgeNode::poll_can_frames, this));
  }

  ~CanBridgeNode() override
  {
    safe_shutdown_sequence();
    if (sock_fd_ >= 0) {
      close(sock_fd_);
      sock_fd_ = -1;
    }
  }

private:
  static std::string default_gate_state_path()
  {
    const char * env = std::getenv("IDLE_CONTROL_GATE_STATE");
    if (env != nullptr && env[0] != '\0') {
      return std::string(env);
    }
    return "/tmp/idle_control_gate_state.json";
  }

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
    std::chrono::steady_clock::time_point last_sent {};
  };

  struct GateConfig
  {
    bool dirty {false};
    bool initialized {false};
    double tx_hz_default {500.0};
    std::unordered_map<int, double> tx_hz_by_motor {};
    std::chrono::steady_clock::time_point loaded_at {};
  };

  void reload_gate_config(const bool force = false)
  {
    const auto now_tp = std::chrono::steady_clock::now();
    if (!force && gate_.initialized) {
      const auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - gate_.loaded_at).count();
      if (age_ms < gate_reload_ms_) {
        return;
      }
    }
    gate_.initialized = true;
    gate_.loaded_at = now_tp;

    std::ifstream in(control_gate_state_file_);
    if (!in.good()) {
      gate_.dirty = false;
      gate_.tx_hz_default = default_tx_hz_;
      gate_.tx_hz_by_motor.clear();
      return;
    }

    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    static const std::regex dirty_re("\"control_param_dirty\"\\s*:\\s*(true|false)");
    static const std::regex tx_default_re("\"tx_hz_default\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+)");
    static const std::regex tx_map_block_re("\"tx_hz_by_motor\"\\s*:\\s*\\{([^}]*)\\}");
    static const std::regex tx_map_entry_re("\"([0-9]+)\"\\s*:\\s*([-+]?[0-9]*\\.?[0-9]+)");

    gate_.dirty = false;
    gate_.tx_hz_default = default_tx_hz_;
    gate_.tx_hz_by_motor.clear();

    std::smatch m;
    if (std::regex_search(text, m, dirty_re) && m.size() >= 2) {
      gate_.dirty = (m[1] == "true");
    }
    if (std::regex_search(text, m, tx_default_re) && m.size() >= 2) {
      try {
        const double hz = std::stod(m[1].str());
        if (hz > 0.0) {
          gate_.tx_hz_default = hz;
        }
      } catch (...) {
      }
    }
    if (std::regex_search(text, m, tx_map_block_re) && m.size() >= 2) {
      const std::string body = m[1].str();
      auto it = std::sregex_iterator(body.begin(), body.end(), tx_map_entry_re);
      const auto end = std::sregex_iterator();
      for (; it != end; ++it) {
        const std::smatch em = *it;
        if (em.size() < 3) {
          continue;
        }
        try {
          const int motor_id = std::stoi(em[1].str());
          const double hz = std::stod(em[2].str());
          if (motor_id >= 0 && hz > 0.0) {
            gate_.tx_hz_by_motor[motor_id] = hz;
          }
        } catch (...) {
        }
      }
    }
  }

  bool control_param_dirty()
  {
    reload_gate_config();
    return gate_.dirty;
  }

  double tx_hz_for_motor(const uint8_t motor_id) const
  {
    const auto it = gate_.tx_hz_by_motor.find(static_cast<int>(motor_id));
    if (it != gate_.tx_hz_by_motor.end() && it->second > 0.0) {
      return it->second;
    }
    return (gate_.tx_hz_default > 0.0) ? gate_.tx_hz_default : default_tx_hz_;
  }

  bool send_mit_command(const msgs::msg::MotorCMD & cmd)
  {
    const uint8_t motor_id = cmd.motor_id;
    const MitRanges & r = ranges_for(motor_id);
    const uint16_t t_u16 = float_to_u16(cmd.tau_ff, r.t_min, r.t_max);
    const uint16_t q_u16 = float_to_u16(cmd.q_des, r.p_min, r.p_max);
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

  bool send_stop_frame(const uint8_t motor_id, const bool clear_fault)
  {
    std::array<uint8_t, 8> data {};
    data[0] = clear_fault ? 1U : 0U;
    const uint32_t arb = pack_ext_id(TYPE04_STOP, host_id_, motor_id);
    return send_ext_frame(arb, data);
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

    const auto now_tp = std::chrono::steady_clock::now();
    bool due = !rt.enable_sent;
    if (!due) {
      const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - rt.last_enable_sent).count();
      due = elapsed_ms >= enable_retry_ms_;
    }
    if (due) {
      if (!send_enable_frame(motor_id)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "failed to send TYPE03 enable for motor_id=%u", motor_id);
      } else {
        rt.enable_sent = true;
        rt.last_enable_sent = now_tp;
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "waiting TYPE02 after TYPE03 before TYPE01 for motor_id=%u", motor_id);
    return false;
  }

  std::vector<uint8_t> shutdown_target_motors() const
  {
    std::set<uint8_t> ids;
    for (const auto & kv : latest_cmd_by_motor_) {
      if (kv.second.valid) {
        ids.insert(kv.first);
      }
    }
    for (const auto & kv : runtime_by_motor_) {
      ids.insert(kv.first);
    }
    for (const uint8_t id : discovered_motor_ids_) {
      ids.insert(id);
    }
    return std::vector<uint8_t>(ids.begin(), ids.end());
  }

  void safe_shutdown_sequence()
  {
    if (shutdown_done_) {
      return;
    }
    shutdown_done_ = true;
    if (sock_fd_ < 0 || !stop_on_shutdown_) {
      return;
    }

    const std::vector<uint8_t> ids = shutdown_target_motors();
    if (ids.empty()) {
      return;
    }

    if (home_on_shutdown_ && shutdown_home_duration_ms_ > 0 && shutdown_home_hz_ > 0.0) {
      const double duration_s = static_cast<double>(shutdown_home_duration_ms_) / 1000.0;
      const int cycles = std::max(1, static_cast<int>(std::ceil(duration_s * shutdown_home_hz_)));
      const auto period = std::chrono::duration<double>(1.0 / shutdown_home_hz_);
      auto next_tp = std::chrono::steady_clock::now();
      msgs::msg::MotorCMD home {};
      home.q_des = static_cast<float>(home_q_des_);
      home.qd_des = static_cast<float>(home_qd_des_);
      home.kp = static_cast<float>(home_kp_);
      home.kd = static_cast<float>(home_kd_);
      home.tau_ff = static_cast<float>(home_tau_ff_);

      RCLCPP_INFO(
        get_logger(),
        "shutdown: moving to home for %d ms @ %.1f Hz before TYPE04 stop",
        shutdown_home_duration_ms_,
        shutdown_home_hz_);

      for (int cycle = 0; cycle < cycles; ++cycle) {
        for (const uint8_t id : ids) {
          (void)send_enable_frame(id);
          home.stamp = now().to_msg();
          home.motor_id = id;
          if (!send_mit_command(home)) {
            RCLCPP_WARN(get_logger(), "shutdown home TYPE01 send failed for motor_id=%u", id);
          }
        }
        next_tp += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next_tp);
      }
    }

    for (const uint8_t id : ids) {
      if (!send_stop_frame(id, stop_clear_fault_)) {
        RCLCPP_WARN(get_logger(), "TYPE04 stop send failed for motor_id=%u", id);
      }
    }
    RCLCPP_INFO(get_logger(), "shutdown: TYPE04 stop sent to %zu motor(s)", ids.size());
  }

  void on_cmd(const msgs::msg::MotorCMD::SharedPtr msg)
  {
    if (control_param_dirty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "control params are dirty: command blocked until saved");
      return;
    }

    CachedCommand & cached = latest_cmd_by_motor_[msg->motor_id];
    cached.cmd = *msg;
    cached.valid = true;
  }

  void dispatch_cached_commands()
  {
    reload_gate_config();
    if (gate_.dirty) {
      return;
    }

    const auto now_tp = std::chrono::steady_clock::now();
    for (auto & kv : latest_cmd_by_motor_) {
      const uint8_t motor_id = kv.first;
      CachedCommand & cached = kv.second;
      if (!cached.valid) {
        continue;
      }
      if (!ensure_motor_ready_for_control(motor_id)) {
        continue;  // Enforce TYPE03 -> TYPE02 -> TYPE01 order.
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

      if (!send_mit_command(cached.cmd)) {
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
      if (type != TYPE00_GET_DEVICE_ID && type != TYPE02_FEEDBACK) {
        continue;
      }
      int motor_id = -1;
      if (type == TYPE00_GET_DEVICE_ID) {
        motor_id = static_cast<int>(data1_from_arb(arb));
      } else if (type == TYPE02_FEEDBACK) {
        motor_id = static_cast<int>(data2_from_arb(arb) & 0xFF);
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
    for (int i = 0; i < rx_max_frames_per_tick_; ++i) {
      auto frame_opt = recv_frame();
      if (!frame_opt.has_value()) {
        break;
      }
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

      msgs::msg::MotorState st {};
      st.stamp = now().to_msg();
      st.motor_id = motor_id;
      st.q = u16_to_float(q_raw, r.p_min, r.p_max);
      st.qd = u16_to_float(qd_raw, r.v_min, r.v_max);
      st.tau = u16_to_float(tau_raw, r.t_min, r.t_max);
      st.temp_c = static_cast<float>(temp_raw) / 10.0F;
      state_pub_->publish(st);

      MotorRuntime & rt = runtime_by_motor_[motor_id];
      rt.has_state = true;
      rt.last_state = st;
      if (rt.enable_sent && !rt.ready_for_control) {
        rt.ready_for_control = true;
        RCLCPP_INFO(get_logger(), "motor_id=%u ready: TYPE03 -> TYPE02 observed", motor_id);
      }

      const uint8_t prev_fault = last_fault_bits_[motor_id];
      const bool changed = (prev_fault != fault_bits);
      last_fault_bits_[motor_id] = fault_bits;
      if (!changed) {
        continue;
      }

      msgs::msg::MotorError err {};
      err.stamp = st.stamp;
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
      error_pub_->publish(err);
    }

    // Rx is event-driven; Tx is rate-controlled per motor using latest cached command.
    dispatch_cached_commands();
  }

  int sock_fd_{-1};
  std::string channel_;
  std::string control_gate_state_file_;
  uint8_t host_id_{0xFD};
  uint8_t rs03_id_{2};
  int scan_min_id_{1};
  int scan_max_id_{10};
  int scan_wait_ms_{200};
  int rx_max_frames_per_tick_{128};
  double default_tx_hz_{500.0};
  int gate_reload_ms_{50};
  int enable_retry_ms_{100};
  bool stop_on_shutdown_{true};
  bool home_on_shutdown_{true};
  bool stop_clear_fault_{false};
  int shutdown_home_duration_ms_{400};
  double shutdown_home_hz_{200.0};
  double home_q_des_{0.0};
  double home_qd_des_{0.0};
  double home_kp_{8.0};
  double home_kd_{0.6};
  double home_tau_ff_{0.0};
  bool shutdown_done_{false};

  std::unordered_map<uint8_t, uint8_t> last_fault_bits_;
  std::unordered_map<uint8_t, CachedCommand> latest_cmd_by_motor_;
  std::unordered_map<uint8_t, MotorRuntime> runtime_by_motor_;
  std::set<uint8_t> discovered_motor_ids_;
  GateConfig gate_;
  rclcpp::Publisher<msgs::msg::MotorState>::SharedPtr state_pub_;
  rclcpp::Publisher<msgs::msg::MotorError>::SharedPtr error_pub_;
  rclcpp::Subscription<msgs::msg::MotorCMD>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
