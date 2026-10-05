#include <memory>
#include <set>
#include <vector>
#include <regex>

#include <crazyflie_cpp/Crazyflie.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "std_srvs/srv/empty.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
#include "crazyflie_interfaces/msg/status.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/connection_statistics_array.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using crazyflie_interfaces::srv::StartTrajectory;
using crazyflie_interfaces::srv::Takeoff;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;
using crazyflie_interfaces::srv::UploadTrajectory;
using crazyflie_interfaces::srv::NotifySetpointsStop;
using crazyflie_interfaces::srv::Arm;
using std_srvs::srv::Empty;

using motion_capture_tracking_interfaces::msg::NamedPoseArray;
using crazyflie_interfaces::msg::FullState;

#ifdef ROS_DISTRO_HUMBLE
inline auto get_service_qos() { return rmw_qos_profile_services_default; }
#else
inline auto get_service_qos() { return rclcpp::ServicesQoS(); }
#endif

// Note on logging: we use a single logger with string prefixes
// A better way would be to use named child loggers, but these do not
// report to /rosout in humble, see https://github.com/ros2/rclpy/issues/1131
// Once we do not support humble anymore, consider switching to child loggers

// Helper class to convert crazyflie_cpp logging messages to ROS logging messages
class CrazyflieLogger : public Logger
{
public:
  CrazyflieLogger(rclcpp::Logger logger, const std::string& prefix)
      : Logger()
      , logger_(logger)
      , prefix_(prefix)
  {
  }

  virtual ~CrazyflieLogger() {}

  virtual void info(const std::string &msg)
  {
    RCLCPP_INFO(logger_, "%s %s", prefix_.c_str(), msg.c_str());
  }

  virtual void warning(const std::string &msg)
  {
    RCLCPP_WARN(logger_, "%s %s", prefix_.c_str(),  msg.c_str());
  }

  virtual void error(const std::string &msg)
  {
    RCLCPP_ERROR(logger_, "%s %s", prefix_.c_str(), msg.c_str());
  }
private:
  rclcpp::Logger logger_;
  std::string prefix_;
};

std::set<std::string> extract_names(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    const std::string &pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

// ROS wrapper for a single Crazyflie object
class CrazyflieROS
{
private:
  struct logPose {
    float x;
    float y;
    float z;
    int32_t quatCompressed;
  } __attribute__((packed));

  struct logScan {
    uint16_t front;
    uint16_t left;
    uint16_t back;
    uint16_t right;
  } __attribute__((packed));

  struct logOdom {
    int16_t x;
    int16_t y;
    int16_t z;
    int32_t quatCompressed;
    int16_t vx;
    int16_t vy;
    int16_t vz;
    //int16_t rateRoll;  
    //int16_t ratePitch;
    //int16_t rateYaw;
  } __attribute__((packed));


  struct logStatus {
    // general status
    uint16_t supervisorInfo; // supervisor.info
    // battery related
    // Note that using BQ-deck/Bolt one can actually have two batteries at the same time.
    // vbat refers to the battery directly connected to the CF board and might not reflect
    // the "external" battery on BQ/Bolt builds
    uint16_t vbatMV;  // pm.vbatMV
    uint8_t pmState;  // pm.state
    // radio related
    uint8_t rssi;     // radio.rssi
    uint16_t numRxBc; // radio.numRxBc
    uint16_t numRxUc; // radio.numRxUc
  } __attribute__((packed));

public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& cf_type,
    const std::string& name,
    rclcpp::Node* node,
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd,
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv,
    const CrazyflieBroadcaster* cfbc,
    bool enable_parameters = true)
    : logger_(node->get_logger())
    , cf_logger_(logger_, "[" + name + "]")
    , cf_(
      link_uri,
      cf_logger_,
      std::bind(&CrazyflieROS::on_console, this, std::placeholders::_1))
    , name_(name)
    , node_(node)
    , tf_broadcaster_(node)
    , last_on_latency_(std::chrono::steady_clock::now())
    , cfbc_(cfbc)
    , previous_numRxBc(0)
    , previous_numRxUc(0)
    , previous_stats_unicast_()
    , previous_stats_broadcast_()
    , last_latency_in_ms_(0)
    , first_status_msg_(true)
  {
    auto sub_opt_cf_cmd = rclcpp::SubscriptionOptions();
    sub_opt_cf_cmd.callback_group = callback_group_cf_cmd;

    // Services
    auto service_qos = rmw_qos_profile_services_default;


    service_emergency_ = node->create_service<Empty>(name + "/emergency", std::bind(&CrazyflieROS::emergency, this, _1, _2),  get_service_qos(), callback_group_cf_srv);
    service_start_trajectory_ = node->create_service<StartTrajectory>(name + "/start_trajectory", std::bind(&CrazyflieROS::start_trajectory, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_takeoff_ = node->create_service<Takeoff>(name + "/takeoff", std::bind(&CrazyflieROS::takeoff, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_land_ = node->create_service<Land>(name + "/land", std::bind(&CrazyflieROS::land, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_go_to_ = node->create_service<GoTo>(name + "/go_to", std::bind(&CrazyflieROS::go_to, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_upload_trajectory_ = node->create_service<UploadTrajectory>(name + "/upload_trajectory", std::bind(&CrazyflieROS::upload_trajectory, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_notify_setpoints_stop_ = node->create_service<NotifySetpointsStop>(name + "/notify_setpoints_stop", std::bind(&CrazyflieROS::notify_setpoints_stop, this, _1, _2), get_service_qos(), callback_group_cf_srv);
    service_arm_ = node->create_service<Arm>(name + "/arm", std::bind(&CrazyflieROS::arm, this, _1, _2), get_service_qos(), callback_group_cf_srv);

    // Topics

    subscription_cmd_vel_legacy_ = node->create_subscription<geometry_msgs::msg::Twist>(name + "/cmd_vel_legacy", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_vel_legacy_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_full_state_ = node->create_subscription<crazyflie_interfaces::msg::FullState>(name + "/cmd_full_state", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_full_state_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_position_ = node->create_subscription<crazyflie_interfaces::msg::Position>(name + "/cmd_position", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_position_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_velocity_world_ = node->create_subscription<crazyflie_interfaces::msg::VelocityWorld>(name + "/cmd_velocity_world", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_velocity_world_changed, this, _1), sub_opt_cf_cmd);
    subscription_cmd_hover_ = node->create_subscription<crazyflie_interfaces::msg::Hover>(name + "/cmd_hover", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_hover_changed, this, _1), sub_opt_cf_cmd);

    publisher_robot_description_ = node->create_publisher<std_msgs::msg::String>(name + "/robot_description",
      rclcpp::QoS(1).transient_local());
    {
      auto msg = std::make_unique<std_msgs::msg::String>();
      auto robot_desc = node->get_parameter("robot_description").get_parameter_value().get<std::string>();
      msg->data = std::regex_replace(robot_desc, std::regex("\\$NAME"), name);
      publisher_robot_description_->publish(std::move(msg));
    }

    // spinning timer
    // used to process all incoming radio messages
    spin_timer_ =
      node->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&CrazyflieROS::spin_once, this), callback_group_cf_srv);

    // link statistics
    warning_freq_ = node->get_parameter("warnings.frequency").get_parameter_value().get<float>();
    max_latency_ = node->get_parameter("warnings.communication.max_unicast_latency").get_parameter_value().get<float>();
    min_ack_rate_ = node->get_parameter("warnings.communication.min_unicast_ack_rate").get_parameter_value().get<float>();
    min_unicast_receive_rate_ = node->get_parameter("warnings.communication.min_unicast_receive_rate").get_parameter_value().get<float>();
    min_broadcast_receive_rate_ = node->get_parameter("warnings.communication.min_broadcast_receive_rate").get_parameter_value().get<float>();
    publish_stats_ = node->get_parameter("warnings.communication.publish_stats").get_parameter_value().get<bool>();
    if (publish_stats_) {
      publisher_connection_stats_ = node->create_publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>(name + "/connection_statistics", 10);
    }

    // Packet tracing for ONE drone, OFF unless asked for:
    //   ros2 launch crazyflie launch.py --ros-args -p debug.trace_cf:=cf1
    // Prints, to stderr, every non-null ack at the link layer and the dispatch
    // decision for every packet in Crazyflie::processPacket -- which is how we
    // tell apart the three remaining explanations of the "link alive, log data
    // dead" stall (CLAUDE.md): log packets never arriving, arriving but failing
    // crtpLogDataResponse::valid(), or arriving on the wrong connection.
    //
    // Read at CONNECT only, deliberately: this rig's same-process
    // /parameter_events never loop back (see the DDS gotcha in CLAUDE.md), so a
    // runtime toggle would silently do nothing. The default is spelled out to
    // avoid the declare_parameter(name, {}) overload trap, also in CLAUDE.md.
    if (!node->has_parameter("telemetry_watchdog_s")) {
      node->declare_parameter<double>("telemetry_watchdog_s", 0.0);
    }
    telemetry_watchdog_s_ = node->get_parameter("telemetry_watchdog_s").as_double();
    if (telemetry_watchdog_s_ > 0.0) {
      RCLCPP_INFO(logger_, "[%s] telemetry watchdog armed at %.1f s", name_.c_str(),
                  telemetry_watchdog_s_);
    }

    if (!node->has_parameter("debug.trace_cf")) {
      node->declare_parameter<std::string>("debug.trace_cf", std::string(""));
    }
    // Accepts "all", or a comma-separated list: debug.trace_cf:=cf1,cf3,cf5.
    // WHICH drone stalls is random per session, so tracing a single guess is a
    // bet; tracing every drone costs one summary line per second each, which is
    // why the trace is summary-based rather than per-packet.
    {
      const std::string want = node->get_parameter("debug.trace_cf").as_string();
      bool on = (want == "all" || want == "*");
      if (!on && !want.empty()) {
        size_t pos = 0;
        while (!on && pos <= want.size()) {
          size_t comma = want.find(',', pos);
          if (comma == std::string::npos) comma = want.size();
          std::string one = want.substr(pos, comma - pos);
          // trim spaces so "cf1, cf3" works as well as "cf1,cf3"
          while (!one.empty() && isspace((unsigned char)one.front())) one.erase(one.begin());
          while (!one.empty() && isspace((unsigned char)one.back())) one.pop_back();
          if (one == name_) on = true;
          pos = comma + 1;
        }
      }
      if (on) {
        RCLCPP_WARN(logger_, "[%s] PACKET TRACE ENABLED (1 Hz summaries on stderr)", name_.c_str());
        cf_.setTrace(true);
      }
    }

    if (warning_freq_ >= 0.0) {
      cf_.setLatencyCallback(std::bind(&CrazyflieROS::on_latency, this, std::placeholders::_1));
      link_statistics_timer_ =
        node->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0/warning_freq_)),
        std::bind(&CrazyflieROS::on_link_statistics_timer, this), callback_group_cf_srv);

    }

    // KEEP-ALIVE -- a guaranteed unicast floor per drone.
    // The firmware deletes every log block by itself (log.c, logRunBlock:
    // `if (!crtpIsConnected()) { logReset(); crtpReset(); }`) when the DRONE has
    // received no CRTP packet for RADIO_ACTIVITY_TIMEOUT_MS = 1000 ms
    // (radiolink.c). The host is never told, and the nRF keeps acking polls, so
    // the link still measures perfect -- the "link alive, log data dead" stall.
    // MEASURED: a healthy drone here already receives ~170 packets/s (the link
    // library's auto-pings refresh the same tick), so this is insurance against
    // that behaviour changing, NOT the fix -- a connection starved for a whole
    // second starves this timer's packet too. The recovery below is the fix.
    // NOT on callback_group_cf_srv: that group is mutually exclusive and the
    // watchdog's rebuild runs there, so a rebuild would silence the keep-alive
    // for the one drone that can least afford it.
    keepalive_hz_ = node->get_parameter("keepalive_frequency").as_double();
    if (keepalive_hz_ > 0.0) {
      keepalive_timer_ = node->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0 / keepalive_hz_)),
        [this]() { cf_.triggerLatencyMeasurement(); }, callback_group_cf_cmd);
    }

    auto start = std::chrono::system_clock::now();

    cf_.logReset();

    auto node_parameters_iface = node->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
        node_parameters_iface->get_parameter_overrides();

    // declares lambda, to be used as local function, which re-declares specified parameters for other nodes to query
    auto declare_param = [&parameter_overrides, node](const std::string& param)
    {
      // rclcpp::ParameterValue value(parameter_overridesparam]);
      node->declare_parameter(param, parameter_overrides.at(param));
    };
    declare_param("robots." + name + ".uri");
    declare_param("robots." + name + ".initial_position");

    // declares a lambda, to be used as local function
    auto update_map = [&parameter_overrides](std::map<std::string, rclcpp::ParameterValue>& map, const std::string& pattern)
    {
      for (const auto &i : parameter_overrides) {
        if (i.first.find(pattern) == 0) {
          size_t start = pattern.size() + 1;
          const auto group_and_name = i.first.substr(start);
          map[group_and_name] = i.second;
        }
      }
    };

    auto update_value = [&parameter_overrides](rclcpp::ParameterValue& value, const std::string& pattern)
    {
      for (const auto &i : parameter_overrides) {
        if (i.first.find(pattern) == 0) {
          value = i.second;
        }
      }
    };

    if (enable_parameters) {
      bool query_all_values_on_connect = node->get_parameter("firmware_params.query_all_values_on_connect").get_parameter_value().get<bool>();

      int numParams = 0;
      RCLCPP_INFO(logger_, "[%s] Requesting parameters...", name_.c_str());
      cf_.requestParamToc(/*forceNoCache*/false, /*requestValues*/query_all_values_on_connect);
      for (auto iter = cf_.paramsBegin(); iter != cf_.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = name + ".params." + entry.group + "." + entry.name;
        switch (entry.type)
        {
        case Crazyflie::ParamTypeUint8:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<uint8_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt8:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int8_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeUint16:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<uint16_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt16:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int16_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeUint32:
          if (query_all_values_on_connect) {
            node->declare_parameter<int64_t>(paramName, cf_.getParam<uint32_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt32:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int32_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeFloat:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<float>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_DOUBLE);
          }
          break;
        default:
          RCLCPP_WARN(logger_, "[%s] Unknown param type for %s/%s", name_.c_str(), entry.group.c_str(), entry.name.c_str());
          break;
        }
        // If there is no such parameter in all, add it
        std::string allParamName = "all.params." + entry.group + "." + entry.name;
        if (!node->has_parameter(allParamName)) {
          if (entry.type == Crazyflie::ParamTypeFloat) {
            node->declare_parameter(allParamName, rclcpp::PARAMETER_DOUBLE);
          } else {
            node->declare_parameter(allParamName, rclcpp::PARAMETER_INTEGER);
          }
        }
        ++numParams;
      }
      auto end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds1 = end1 - start;
      RCLCPP_INFO(logger_, "[%s] reqParamTOC: %f s (%d params)", name_.c_str(), elapsedSeconds1.count(), numParams);
      
      // Set parameters as specified in the configuration files, as in the following order
      // 1.) check all/firmware_params
      // 2.) check robot_types/<type_name>/firmware_params
      // 3.) check robots/<robot_name>/firmware_params
      // where the higher order is used if defined on multiple levels.

      // <group>.<name> -> value map
      std::map<std::string, rclcpp::ParameterValue> set_param_map;

      // check global settings/firmware_params
      update_map(set_param_map, "all.firmware_params");
      // check robot_types/<type_name>/firmware_params
      update_map(set_param_map, "robot_types." + cf_type + ".firmware_params");
      // check robots/<robot_name>/firmware_params
      update_map(set_param_map, "robots." + name_ + ".firmware_params");

      // Update parameters
      for (const auto&i : set_param_map) {
        std::string paramName = name + ".params." + std::regex_replace(i.first, std::regex("\\."), ".");
        change_parameter(rclcpp::Parameter(paramName, i.second));
      }

      // Light the bottom Color LED deck GREEN on connect, if present, as a
      // quick visual check that the deck + param path work at launch.
      // colorLedBot.wrgb8888 is a uint32 packed as 0xWWRRGGBB (white/red/green/blue).
      // Change colours at runtime with scripts/led.sh or:
      //   ros2 run crazyflie_examples color_led <color>
      // Guarded on the TOC entry so a drone without the deck is silently skipped
      // (getParamTocEntry returns nullptr rather than throwing).
      {
        const uint32_t kConnectLedColor = 0x0000FF00;  // green; edit to taste
        auto ledEntry = cf_.getParamTocEntry("colorLedBot", "wrgb8888");
        if (ledEntry) {
          cf_.setParam<uint32_t>(ledEntry->id, kConnectLedColor);
          RCLCPP_INFO(logger_, "[%s] Color LED deck: set colorLedBot.wrgb8888 to 0x%08X (green) on connect", name_.c_str(), kConnectLedColor);
        } else {
          RCLCPP_INFO(logger_, "[%s] No Color LED deck detected (colorLedBot.wrgb8888 absent); skipping LED set", name_.c_str());
        }
      }
    }

    // Reference Frame
    {
      rclcpp::ParameterValue reference_frame_value("world");

      // Get logging configuration as specified in the configuration files, as in the following order
      // 1.) check all/reference_frame
      // 2.) check robot_types/<type_name>/reference_frame
      // 3.) check robots/<robot_name>/reference_frame
      // where the higher order is used if defined on multiple levels.
      update_value(reference_frame_value, "all.reference_frame");
      // check robot_types/<type_name>/reference_frame
      update_value(reference_frame_value, "robot_types." + cf_type + ".reference_frame");
      // check robots/<robot_name>/reference_frame
      update_value(reference_frame_value, "robots." + name_ + ".reference_frame");

      // extract reference frame 
      reference_frame_ = reference_frame_value.get<std::string>();
     
      RCLCPP_INFO(logger_, "[%s] ref frame: %s", name_.c_str(), reference_frame_.c_str());
      
    }

    // Logging
    {
      // <group>.<name> -> value map
      std::map<std::string, rclcpp::ParameterValue> log_config_map;

      // Get logging configuration as specified in the configuration files, as in the following order
      // 1.) check all/firmware_logging
      // 2.) check robot_types/<type_name>/firmware_logging
      // 3.) check robots/<robot_name>/firmware_logging
      // where the higher order is used if defined on multiple levels.
      update_map(log_config_map, "all.firmware_logging");
      // check robot_types/<type_name>/firmware_logging
      update_map(log_config_map, "robot_types." + cf_type + ".firmware_logging");
      // check robots/<robot_name>/firmware_logging
      update_map(log_config_map, "robots." + name_ + ".firmware_logging");

      // check if logging is enabled for this drone
      bool logging_enabled = log_config_map["enabled"].get<bool>();
      if (logging_enabled) {
        cf_.requestLogToc(/*forceNoCache*/);

        for (const auto&i : log_config_map) {
          // check if any of the default topics are enabled
          if (i.first.find("default_topics.pose") == 0) {
            int freq = log_config_map["default_topics.pose.frequency"].get<int>();
            RCLCPP_INFO(logger_, "[%s] Logging to /pose at %d Hz", name_.c_str(), freq);

            publisher_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(name + "/pose", 10);

            std::function<void(uint32_t, const logPose*)> cb = std::bind(&CrazyflieROS::on_logging_pose, this, std::placeholders::_1, std::placeholders::_2);

            const std::list<std::pair<std::string, std::string>> vars({
                {"stateEstimate", "x"},
                {"stateEstimate", "y"},
                {"stateEstimate", "z"},
                {"stateEstimateZ", "quat"}
            });
            period_pose_ = uint8_t(100.0f / (float)freq);
            // Registered as a BUILDER (which creates it now) rather than just
            // constructed: a stalled drone has DROPPED its blocks, so recovery
            // has to create them again - see rebuild_log_blocks().
            add_log_builder([this, vars, cb]() mutable {
              log_block_pose_.reset(new LogBlock<logPose>(&cf_, vars, cb, recovery_timeout_ms_));
              log_block_pose_->start(period_pose_);
            });
          }
          else if (i.first.find("default_topics.scan") == 0) {
            int freq = log_config_map["default_topics.scan.frequency"].get<int>();
            RCLCPP_INFO(logger_, "[%s] Logging to /scan at %d Hz", name_.c_str(), freq);

            publisher_scan_ = node->create_publisher<sensor_msgs::msg::LaserScan>(name + "/scan", 10);

            std::function<void(uint32_t, const logScan*)> cb = std::bind(&CrazyflieROS::on_logging_scan, this, std::placeholders::_1, std::placeholders::_2);

            const std::list<std::pair<std::string, std::string>> vars({
                {"range", "front"},
                {"range", "left"},
                {"range", "back"},
                {"range", "right"}
            });
            period_scan_ = uint8_t(100.0f / (float)freq);
            add_log_builder([this, vars, cb]() mutable {
              log_block_scan_.reset(new LogBlock<logScan>(&cf_, vars, cb, recovery_timeout_ms_));
              log_block_scan_->start(period_scan_);
            });
          }
          else if (i.first.find("default_topics.odom") == 0) {
            int freq = log_config_map["default_topics.odom.frequency"].get<int>();
            RCLCPP_INFO(logger_, "[%s] Logging to /odom at %d Hz", name_.c_str(), freq);

            publisher_odom_ = node->create_publisher<nav_msgs::msg::Odometry>(name + "/odom", 10);

            std::function<void(uint32_t, const logOdom*)> cb = std::bind(&CrazyflieROS::on_logging_odom, this, std::placeholders::_1, std::placeholders::_2);

            const std::list<std::pair<std::string, std::string>> vars({
                {"stateEstimateZ", "x"},
                {"stateEstimateZ", "y"},
                {"stateEstimateZ", "z"},
                {"stateEstimateZ", "quat"},
                {"stateEstimateZ", "vx"},
                {"stateEstimateZ", "vy"},
                {"stateEstimateZ", "vz"},
                //{"stateEstimateZ", "rateRoll"},
                //{"stateEstimateZ", "ratePitch"},
                //{"stateEstimateZ", "rateYaw"}
            });
            period_odom_ = uint8_t(100.0f / (float)freq);
            add_log_builder([this, vars, cb]() mutable {
              log_block_odom_.reset(new LogBlock<logOdom>(&cf_, vars, cb, recovery_timeout_ms_));
              log_block_odom_->start(period_odom_);
            });
            
          }
          else if (i.first.find("default_topics.status") == 0) {
            int freq = log_config_map["default_topics.status.frequency"].get<int>();
            RCLCPP_INFO(logger_, "[%s] Logging to /status at %d Hz", name_.c_str(), freq);

            publisher_status_ = node->create_publisher<crazyflie_interfaces::msg::Status>(name + "/status", 10);

            std::function<void(uint32_t, const logStatus*)> cb = std::bind(&CrazyflieROS::on_logging_status, this, std::placeholders::_1, std::placeholders::_2);

            std::list<std::pair<std::string, std::string> > logvars({
              // general status
              {"supervisor", "info"},
              // battery related
              {"pm", "vbatMV"},
              {"pm", "state"},
              // radio related
              {"radio", "rssi"}
            });

            // check if this firmware version has radio.numRx{Bc,Uc}
            status_has_radio_stats_ = false;
            for (auto iter = cf_.logVariablesBegin(); iter != cf_.logVariablesEnd(); ++iter) {
              auto entry = *iter;
              if (entry.group == "radio" && entry.name == "numRxBc") {
                logvars.push_back({"radio", "numRxBc"});
                logvars.push_back({"radio", "numRxUc"});
                status_has_radio_stats_ = true;
                break;
              }
            }

            // older firmware -> use other 16-bit variables
            if (!status_has_radio_stats_) {
                RCLCPP_WARN(logger_, "[%s] Older firmware. status/num_rx_broadcast and status/num_rx_unicast are set to zero.", name_.c_str());
                logvars.push_back({"pm", "vbatMV"});
                logvars.push_back({"pm", "vbatMV"});
            }

            period_status_ = uint8_t(100.0f / (float)freq);
            add_log_builder([this, logvars, cb]() mutable {
              log_block_status_.reset(new LogBlock<logStatus>(&cf_, logvars, cb, recovery_timeout_ms_));
              log_block_status_->start(period_status_);
            });
          }
          else if (i.first.find("custom_topics") == 0
                   && i.first.rfind(".vars") != std::string::npos) {
            std::string topic_name = i.first.substr(14, i.first.size() - 14 - 5);

            int freq = log_config_map["custom_topics." + topic_name + ".frequency"].get<int>();
            auto vars = log_config_map["custom_topics." + topic_name + ".vars"].get<std::vector<std::string>>();
            
            RCLCPP_INFO(logger_, "[%s] Logging to %s at %d Hz", name_.c_str(), topic_name.c_str(), freq);

            publishers_generic_.emplace_back(node->create_publisher<crazyflie_interfaces::msg::LogDataGeneric>(name + "/" + topic_name, 10));

            std::function<void(uint32_t, const std::vector<float>*, void* userData)> cb = std::bind(
              &CrazyflieROS::on_logging_custom,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3);

            // publishers_generic_ is a std::list, so this address stays valid
            // however many more custom topics are added after it.
            void* user_data = (void*)&publishers_generic_.back();
            const uint8_t period = uint8_t(100.0f / (float)freq);
            add_log_builder([this, vars, cb, user_data, period]() mutable {
              log_blocks_generic_.emplace_back(new LogBlockGeneric(
                &cf_, vars, user_data, cb, recovery_timeout_ms_));
              periods_generic_.push_back(period);
              log_blocks_generic_.back()->start(period);
            });
          }
        }
      }
    }

    RCLCPP_INFO(logger_, "[%s] Requesting memories...", name_.c_str());
    cf_.requestMemoryToc();
  }

  ~CrazyflieROS()
  {
    shutting_down_ = true;
    // Connection indicator: turn the Color LED deck off on clean disconnect
    // (best effort - on a hard link loss there is no link left to command).
    // setParam only ENQUEUES the packet; destroying cf_ closes the connection
    // and drops the send queue (Connection::close -> removeConnection), so we
    // must wait for the radio ack before returning. Bounded at 500 ms so a
    // dead link can never wedge shutdown.
    try {
      auto ledEntry = cf_.getParamTocEntry("colorLedBot", "wrgb8888");
      if (ledEntry) {
        const size_t acksBefore = cf_.connectionStats().ack_count;
        cf_.setParam<uint32_t>(ledEntry->id, 0x00000000);
        const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (cf_.connectionStats().ack_count <= acksBefore &&
               std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (cf_.connectionStats().ack_count > acksBefore) {
          // small grace so the radio finishes the exchange for our packet
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          fprintf(stderr, "[%s] Color LED deck: off-command acked on disconnect\n", name_.c_str());
        } else {
          fprintf(stderr, "[%s] Color LED deck: off-command NOT acked (link down?); LED state unknown\n", name_.c_str());
        }
      }
    } catch (...) {
      // link may already be gone; LED state is then unknowable - ignore
    }
  }

  void spin_once()
  {
    // process all packets from the receive queue
    cf_.processAllPackets();
  }

  std::string broadcastUri() const
  {
    return cf_.broadcastUri();
  }

  //: Feed the telemetry watchdog's airborne guard from the server's /poses
  //: handler - the only place that sees altitude for every drone however it was
  //: commanded, broadcast /all/takeoff included.
  //: Did trajectory `id` fail to upload to this drone? See upload_trajectory().
  bool trajectory_is_bad(uint8_t id) const { return bad_trajectories_.count(id) > 0; }

  void note_mocap_z(float z)
  {
    last_mocap_z_ = z;
    last_mocap_z_valid_ = true;
    if (z <= 0.10f) {
      commanded_flight_ = false;        // provably on the floor again
    }
  }

  uint8_t id() const
  {
    return cf_.address() & 0xFF;
  }

  const Crazyflie::ParamTocEntry* paramTocEntry(const std::string& group, const std::string& name)
  {
    return cf_.getParamTocEntry(group, name);
  }

  const std::string& name() const
  {
    return name_;
  }

  void change_parameter(const rclcpp::Parameter& p)
  {
    std::string prefix = name_ + ".params.";
    if (p.get_name().find(prefix) != 0) {
      RCLCPP_ERROR(
              logger_,
              "[%s] Incorrect parameter update request for param \"%s\"", name_.c_str(), p.get_name().c_str());
      return;
    }
    size_t pos = p.get_name().find(".", prefix.size());
    std::string group(p.get_name().begin() + prefix.size(), p.get_name().begin() + pos);
    std::string name(p.get_name().begin() + pos + 1, p.get_name().end());

    RCLCPP_INFO(
        logger_,
        "[%s] Update parameter \"%s.%s\" to %s",
        name_.c_str(),
        group.c_str(),
        name.c_str(),
        p.value_to_string().c_str());

    auto entry = cf_.getParamTocEntry(group, name);
    if (entry) {
      switch (entry->type)
      {
      case Crazyflie::ParamTypeUint8:
        cf_.setParam<uint8_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt8:
        cf_.setParam<int8_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeUint16:
        cf_.setParam<uint16_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt16:
        cf_.setParam<int16_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeUint32:
        cf_.setParam<uint32_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt32:
        cf_.setParam<int32_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeFloat:
        if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
          cf_.setParam<float>(entry->id, (float)p.as_int());
        } else {
          cf_.setParam<float>(entry->id, p.as_double());
        }

        break;
      }
    } else {
      RCLCPP_ERROR(logger_, "[%s] Could not find param %s/%s", name_.c_str(), group.c_str(), name.c_str());
    }
  }

private:

  void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg)
  { 
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;
    cf_.sendFullStateSetpoint(
    x, y, z,
    vx, vy, vz,
    ax, ay, az,
    qx, qy, qz, qw,
    rollRate, pitchRate, yawRate);

  }

  void cmd_position_changed(const crazyflie_interfaces::msg::Position::SharedPtr msg) {
    float x = msg->x;
    float y = msg->y;
    float z = msg->z;
    float yaw = msg->yaw;
    cf_.sendPositionSetpoint(x, y, z, yaw);
  }

    void cmd_velocity_world_changed(const crazyflie_interfaces::msg::VelocityWorld::SharedPtr msg) {
    float vx = msg->vel.x;
    float vy = msg->vel.y;
    float vz = msg->vel.z;
    float yaw_rate = msg->yaw_rate;
    cf_.sendVelocityWorldSetpoint(vx, vy, vz, yaw_rate);
  }

  void cmd_hover_changed(const crazyflie_interfaces::msg::Hover::SharedPtr msg) {
    float vx = msg->vx;
    float vy = msg->vy;
    float yawRate = -1.0 * msg->yaw_rate * 180.0 / M_PI; // Convert from radians to degrees
    float z = msg->z_distance;
    cf_.sendHoverSetpoint(vx, vy, yawRate, z);
  }

  void cmd_vel_legacy_changed(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float roll = msg->linear.y;
    float pitch = - (msg->linear.x);
    float yawrate = msg->angular.z;
    uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);
    // RCLCPP_INFO(logger_, "roll: %f, pitch: %f, yaw: %f, thrust: %u", roll, pitch, yawrate, (unsigned int)thrust);
    cf_.sendSetpoint(roll, pitch, yawrate, thrust);
  }

  void on_console(const char *msg)
  {
    message_buffer_ += msg;
    size_t pos = message_buffer_.find('\n');
    if (pos != std::string::npos)
    {
      message_buffer_[pos] = 0;
      RCLCPP_INFO(logger_, "[%s] %s", name_.c_str(), message_buffer_.c_str());
      message_buffer_.erase(0, pos + 1);
    }
  }

  void emergency(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] emergency()", name_.c_str());
    cf_.emergencyStop();
  }

  void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
                        std::shared_ptr<StartTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] start_trajectory(id=%d, timescale=%f, reversed=%d, relative=%d, group_mask=%d)",
      name_.c_str(),
      request->trajectory_id,
      request->timescale,
      request->reversed,
      request->relative,
      request->group_mask);
    if (bad_trajectories_.count(request->trajectory_id)) {
      RCLCPP_FATAL(logger_, "[%s] REFUSING to start trajectory %d: its upload did "
                   "not complete. Re-upload it before flying.",
                   name_.c_str(), request->trajectory_id);
      return;
    }
    cf_.startTrajectory(request->trajectory_id,
      request->timescale,
      request->reversed,
      request->relative,
      request->group_mask);
  }

  void takeoff(const std::shared_ptr<Takeoff::Request> request,
               std::shared_ptr<Takeoff::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] takeoff(height=%f m, duration=%f s, group_mask=%d)", 
                name_.c_str(),
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    cf_.takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
  }

  void land(const std::shared_ptr<Land::Request> request,
            std::shared_ptr<Land::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] land(height=%f m, duration=%f s, group_mask=%d)",
                name_.c_str(),
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    cf_.land(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
  }

  void go_to(const std::shared_ptr<GoTo::Request> request,
             std::shared_ptr<GoTo::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, relative=%d, group_mask=%d)",
                name_.c_str(),
                request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->relative,
                request->group_mask);
    cf_.goTo(request->goal.x, request->goal.y, request->goal.z, request->yaw, 
              rclcpp::Duration(request->duration).seconds(),
              request->relative, request->group_mask);
  }

  void upload_trajectory(const std::shared_ptr<UploadTrajectory::Request> request,
                        std::shared_ptr<UploadTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] upload_trajectory(id=%d, offset=%d)",
                name_.c_str(),
                request->trajectory_id,
                request->piece_offset);

    std::vector<Crazyflie::poly4d> pieces(request->pieces.size());
    for (size_t i = 0; i < pieces.size(); ++i)
    {
      if (   request->pieces[i].poly_x.size() != 8 
          || request->pieces[i].poly_y.size() != 8
          || request->pieces[i].poly_z.size() != 8
          || request->pieces[i].poly_yaw.size() != 8)
      {
        RCLCPP_FATAL(logger_, "[%s] Wrong number of pieces!", name_.c_str());
        return;
      }
      pieces[i].duration = rclcpp::Duration(request->pieces[i].duration).seconds();
      for (size_t j = 0; j < 8; ++j)
      {
        pieces[i].p[0][j] = request->pieces[i].poly_x[j];
        pieces[i].p[1][j] = request->pieces[i].poly_y[j];
        pieces[i].p[2][j] = request->pieces[i].poly_z[j];
        pieces[i].p[3][j] = request->pieces[i].poly_yaw[j];
      }
    }
    // The upload can now give up instead of blocking forever (see
    // Crazyflie::uploadTrajectory). It MUST NOT escape into rclcpp: this
    // callback group is mutually exclusive and shared with this drone's
    // land/takeoff/arm/emergency services and the telemetry watchdog.
    // UploadTrajectory.srv carries no success field, so the caller cannot be
    // told -- hence FATAL, and the flag that start_trajectory checks.
    try {
      cf_.uploadTrajectory(request->trajectory_id, request->piece_offset, pieces);
      bad_trajectories_.erase(request->trajectory_id);
      response->success = true;
    } catch (const std::exception& e) {
      bad_trajectories_.insert(request->trajectory_id);
      response->success = false;
      response->message = std::string(name_) + ": " + e.what();
      RCLCPP_FATAL(logger_, "[%s] TRAJECTORY %d UPLOAD FAILED (%s) - DO NOT FLY IT; "
                   "start_trajectory will refuse this id until it uploads cleanly",
                   name_.c_str(), request->trajectory_id, e.what());
    }
  }

  void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                         std::shared_ptr<NotifySetpointsStop::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] notify_setpoints_stop(remain_valid_millisecs%d, group_mask=%d)",
                name_.c_str(),
                request->remain_valid_millisecs,
                request->group_mask);

    cf_.notifySetpointsStop(request->remain_valid_millisecs);
  }

  void arm(const std::shared_ptr<Arm::Request> request,
                         std::shared_ptr<Arm::Response> response)
  {
    RCLCPP_INFO(logger_, "[%s] arm(%d)",
                name_.c_str(),
                request->arm);

    cf_.sendArmingRequest(request->arm);
  }

  void on_logging_pose(uint32_t time_in_ms, const logPose* data) {
    last_log_rx_ = std::chrono::steady_clock::now();   // telemetry watchdog
    if (shutting_down_) return;  // publishers may already be destroyed during teardown
    if (publisher_pose_) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = reference_frame_;

      msg.pose.position.x = data->x;
      msg.pose.position.y = data->y;
      msg.pose.position.z = data->z;

      float q[4];
      quatdecompress(data->quatCompressed, q);
      msg.pose.orientation.x = q[0];
      msg.pose.orientation.y = q[1];
      msg.pose.orientation.z = q[2];
      msg.pose.orientation.w = q[3];

      publisher_pose_->publish(msg);

      // send a transform for this pose
      geometry_msgs::msg::TransformStamped msg2;
      msg2.header = msg.header;
      msg2.child_frame_id = name_;
      msg2.transform.translation.x = data->x;
      msg2.transform.translation.y = data->y;
      msg2.transform.translation.z = data->z;
      msg2.transform.rotation.x = q[0];
      msg2.transform.rotation.y = q[1];
      msg2.transform.rotation.z = q[2];
      msg2.transform.rotation.w = q[3];
      tf_broadcaster_.sendTransform(msg2);
    }
  }

  void on_logging_scan(uint32_t time_in_ms, const logScan* data) {
    last_log_rx_ = std::chrono::steady_clock::now();   // telemetry watchdog
    if (shutting_down_) return;  // publishers may already be destroyed during teardown
    if (publisher_scan_) {
      
      const float max_range = 3.49;
      float front_range = data->front / 1000.0f;
      if (front_range > max_range) front_range = std::numeric_limits<float>::infinity();
      float left_range = data->left / 1000.0f;
      if (left_range > max_range) left_range = std::numeric_limits<float>::infinity();
      float back_range = data->back / 1000.0f;
      if (back_range > max_range) back_range = std::numeric_limits<float>::infinity();
      float right_range = data->right / 1000.0f;
      if (right_range > max_range) right_range = std::numeric_limits<float>::infinity();

      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = name_;
      msg.range_min = 0.01;
      msg.range_max = max_range;
      msg.ranges.push_back(back_range);
      msg.ranges.push_back(right_range);
      msg.ranges.push_back(front_range);
      msg.ranges.push_back(left_range);
      msg.angle_min = -0.5 * 2 * M_PI;
      msg.angle_max = 0.25 * 2 * M_PI;
      msg.angle_increment = 1.0 * M_PI / 2;

      publisher_scan_->publish(msg);
    }
  }

  void on_logging_odom(uint32_t time_in_ms, const logOdom* data) {
    last_log_rx_ = std::chrono::steady_clock::now();   // telemetry watchdog
    if (shutting_down_) return;  // publishers may already be destroyed during teardown
    if (publisher_odom_) {
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = name_;
      msg.pose.pose.position.x = data->x / 1000.0f;
      msg.pose.pose.position.y = data->y / 1000.0f;
      msg.pose.pose.position.z = data->z / 1000.0f;

      float q[4];
      quatdecompress(data->quatCompressed, q);
      msg.pose.pose.orientation.x = q[0];
      msg.pose.pose.orientation.y = q[1];
      msg.pose.pose.orientation.z = q[2];
      msg.pose.pose.orientation.w = q[3];

      msg.twist.twist.linear.x = data->vx / 1000.0f;
      msg.twist.twist.linear.y = data->vy / 1000.0f;
      msg.twist.twist.linear.z = data->vz / 1000.0f;
      //msg.twist.twist.angular.x = data->rateRoll / 1000.0f;
      //msg.twist.twist.angular.y = data->ratePitch / 1000.0f;
      //msg.twist.twist.angular.z = data->rateYaw / 1000.0f;

      publisher_odom_->publish(msg);
    }

  }

  void on_logging_status(uint32_t time_in_ms, const logStatus* data) {
    last_log_rx_ = std::chrono::steady_clock::now();   // telemetry watchdog
    if (shutting_down_) return;  // publishers may already be destroyed during teardown
    if (publisher_status_) {
      
      crazyflie_interfaces::msg::Status msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = name_;
      msg.supervisor_info = data->supervisorInfo;
      msg.battery_voltage = data->vbatMV / 1000.0f;
      msg.pm_state = data->pmState;
      msg.rssi = data->rssi;
      if (status_has_radio_stats_) {
        if (first_status_msg_) {
          previous_numRxBc = data->numRxBc;
          previous_numRxUc = data->numRxUc;
          previous_stats_unicast_ = cf_.connectionStats();
          previous_stats_broadcast_ = cfbc_->connectionStats();
          first_status_msg_ = false;
          return;
        }
        int32_t deltaRxBc = data->numRxBc - previous_numRxBc;
        int32_t deltaRxUc = data->numRxUc - previous_numRxUc;
        // handle overflow
        if (deltaRxBc < 0) {
          deltaRxBc += std::numeric_limits<uint16_t>::max();
        }
        if (deltaRxUc < 0) {
          deltaRxUc += std::numeric_limits<uint16_t>::max();
        }
        msg.num_rx_broadcast = deltaRxBc;
        msg.num_rx_unicast = deltaRxUc;
        previous_numRxBc = data->numRxBc;
        previous_numRxUc = data->numRxUc;
      } else {
        msg.num_rx_broadcast = 0;
        msg.num_rx_unicast = 0;
      }

      // connection sent stats (unicast)
      const auto statsUc = cf_.connectionStats();
      size_t deltaTxUc = statsUc.sent_count - previous_stats_unicast_.sent_count;
      msg.num_tx_unicast = deltaTxUc;
      previous_stats_unicast_ = statsUc;

      // connection sent stats (broadcast)
      const auto statsBc = cfbc_->connectionStats();
      size_t deltaTxBc = statsBc.sent_count - previous_stats_broadcast_.sent_count;
      msg.num_tx_broadcast = deltaTxBc;
      previous_stats_broadcast_ = statsBc;

      msg.latency_unicast = last_latency_in_ms_;

      publisher_status_->publish(msg);

      // warnings
      if (msg.num_rx_unicast > msg.num_tx_unicast * 1.05 /*allow some slack*/) {
        RCLCPP_WARN(logger_, "[%s] Unexpected number of unicast packets. Sent: %d. Received: %d", name_.c_str(), msg.num_tx_unicast, msg.num_rx_unicast);
      }
      if (msg.num_tx_unicast > 0) {
        float unicast_receive_rate = msg.num_rx_unicast / (float)msg.num_tx_unicast;
        if (unicast_receive_rate < min_unicast_receive_rate_) {
          RCLCPP_WARN(logger_, "[%s] Low unicast receive rate (%.2f < %.2f). Sent: %d. Received: %d", name_.c_str(), unicast_receive_rate, min_unicast_receive_rate_, msg.num_tx_unicast, msg.num_rx_unicast);
        }
      }

      if (msg.num_rx_broadcast > msg.num_tx_broadcast * 1.05 /*allow some slack*/) {
        RCLCPP_WARN(logger_, "[%s] Unexpected number of broadcast packets. Sent: %d. Received: %d", name_.c_str(), msg.num_tx_broadcast, msg.num_rx_broadcast);
      }
      if (msg.num_tx_broadcast > 0) {
        float broadcast_receive_rate = msg.num_rx_broadcast / (float)msg.num_tx_broadcast;
        if (broadcast_receive_rate < min_broadcast_receive_rate_) {
          RCLCPP_WARN(logger_, "[%s] Low broadcast receive rate (%.2f < %.2f). Sent: %d. Received: %d", name_.c_str(), broadcast_receive_rate, min_broadcast_receive_rate_, msg.num_tx_broadcast, msg.num_rx_broadcast);
        }
      }
    }
  }

  void on_logging_custom(uint32_t time_in_ms, const std::vector<float>* values, void* userData) {
    last_log_rx_ = std::chrono::steady_clock::now();   // telemetry watchdog
    if (shutting_down_) return;  // publishers may already be destroyed during teardown

    auto pub = reinterpret_cast<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr*>(userData);

    crazyflie_interfaces::msg::LogDataGeneric msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = reference_frame_;
    msg.timestamp = time_in_ms;
    msg.values = *values;

    (*pub)->publish(msg);
  }

  //: Telemetry watchdog. MEASURED 2026-10-05 with packet tracing: a drone can stop
  //: sending log data on its connection while the link stays perfectly alive -- the
  //: link trace reads log=0 with ~200 nulls/s, i.e. the drone answers every poll
  //: with "nothing to send". Commands, flight and a FRESH connection to the same
  //: drone all keep working, and only recreating the log blocks (which a server
  //: restart does) brings telemetry back. See CLAUDE.md, "link alive, log data dead".
  //: Deliberately fail-safe: anything other than "mocap positively says this drone
  //: is on the floor" counts as maybe-airborne. Telemetry is dead exactly when we
  //: would want to ask the drone, so /cfX/status cannot be trusted here, and the
  //: shows take off with BROADCAST /all/takeoff, which no per-drone command handler
  //: ever observes - mocap altitude is the only signal that sees both cases.
  bool may_be_airborne() const
  {
    if (!last_mocap_z_valid_) return true;
    if (last_mocap_z_ > 0.10f) return true;
    if (commanded_flight_) return true;
    return false;
  }

  //: Create a log block AND remember how, so it can be created again later.
  void add_log_builder(std::function<void()> build)
  {
    build();                                  // exactly what the code did before
    log_block_builders_.push_back(std::move(build));
  }

  //: Tier 1 - stop/start the blocks the drone is believed to still hold.
  void restart_log_blocks()
  {
    if (log_block_pose_   && period_pose_)   { log_block_pose_->stop();   log_block_pose_->start(period_pose_); }
    if (log_block_scan_   && period_scan_)   { log_block_scan_->stop();   log_block_scan_->start(period_scan_); }
    if (log_block_odom_   && period_odom_)   { log_block_odom_->stop();   log_block_odom_->start(period_odom_); }
    if (log_block_status_ && period_status_) { log_block_status_->stop(); log_block_status_->start(period_status_); }
    size_t i = 0;
    for (auto& b : log_blocks_generic_) {
      if (b && i < periods_generic_.size() && periods_generic_[i]) { b->stop(); b->start(periods_generic_[i]); }
      ++i;
    }
  }

  //: Tier 2 - the drone has dropped its blocks, so create them from scratch.
  //
  //  MEASURED 2026-10-05: at a stall, tier 1 fails with "Could not start log
  //  block!" - the drone ANSWERS the control request and refuses it, i.e. the
  //  block id it is being asked to start no longer exists on the aircraft.
  //  That is the whole explanation of the stall: no block, nothing to send,
  //  so every poll is answered with a null and the link looks perfectly
  //  healthy. This is the logging half of what a server restart does.
  void rebuild_log_blocks()
  {
    // Destroy first: ~LogBlock frees its id, so the rebuild reuses the same
    // ids and the drone's own log state is wiped by logReset() underneath.
    // Abandon rather than stop(): the blocks are already gone from the drone,
    // so the stop handshake can only burn its retries, and logReset() wipes the
    // drone's log state anyway.
    if (log_block_pose_)   log_block_pose_->abandon();
    if (log_block_scan_)   log_block_scan_->abandon();
    if (log_block_odom_)   log_block_odom_->abandon();
    if (log_block_status_) log_block_status_->abandon();
    for (auto& b : log_blocks_generic_) { if (b) b->abandon(); }
    log_block_pose_.reset();
    log_block_scan_.reset();
    log_block_odom_.reset();
    log_block_status_.reset();
    log_blocks_generic_.clear();
    periods_generic_.clear();                 // the builders push these back
    // Bail early if the drone will not even answer a reset: every wait on this
    // path is bounded, because it runs in a timer sharing a mutually-exclusive
    // callback group with this drone's land/emergency services.
    if (!cf_.logReset(RECOVERY_TIMEOUT_MS, 2)) {
      throw std::runtime_error("no reply to logReset");
    }
    recovery_timeout_ms_ = RECOVERY_TIMEOUT_MS;   // builders pass it to the blocks
    try {
      for (auto& build : log_block_builders_) {
        build();
      }
    } catch (...) {
      recovery_timeout_ms_ = 0;
      throw;
    }
    recovery_timeout_ms_ = 0;
  }

  void check_telemetry_watchdog()
  {
    if (telemetry_watchdog_s_ <= 0.0 || recovering_) return;
    const auto now = std::chrono::steady_clock::now();
    const double quiet = std::chrono::duration<double>(now - last_log_rx_).count();
    // A rebuild that failed part-way leaves blocks missing while others still
    // deliver, so `quiet` never grows: retry on the flag instead, on the same
    // cadence rather than on every tick of this timer.
    const bool retry_due = rebuild_pending_ &&
        std::chrono::duration<double>(now - last_rebuild_attempt_).count() >= telemetry_watchdog_s_;
    if (quiet < telemetry_watchdog_s_ && !retry_due) return;
    const double since_latency = std::chrono::duration<double>(now - last_latency_rx_).count();
    if (since_latency > 2.0) return;    // an ordinary dead link: not ours to fix
    if (may_be_airborne()) {
      RCLCPP_ERROR(logger_, "[%s] NO TELEMETRY for %.1f s though the link is alive, and "
                   "this drone may be AIRBORNE (mocap z %s%.2f m) - NOT touching its "
                   "connection. Land it; recovery runs once it is down.", name_.c_str(),
                   quiet, last_mocap_z_valid_ ? "" : "unknown, last ", last_mocap_z_);
      last_log_rx_ = now;               // once per window, not once per second
      return;
    }
    recovering_ = true;
    last_rebuild_attempt_ = now;
    // After a part-built rebuild, stop/start would "succeed" on the blocks that
    // survived and never recreate the missing ones, so skip straight to tier 2.
    bool recovered = false;
    if (retry_due) {
      RCLCPP_WARN(logger_, "[%s] retrying the half-finished log-block rebuild",
                  name_.c_str());
    } else {
      RCLCPP_WARN(logger_, "[%s] NO TELEMETRY for %.1f s though the link is alive "
                  "(latency %.1f s ago) - restarting its log blocks in place",
                  name_.c_str(), quiet, since_latency);
      try {
        restart_log_blocks();
        RCLCPP_WARN(logger_, "[%s] log blocks restarted; watching for data", name_.c_str());
        recovered = true;
      } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "[%s] stop/start refused (%s) - the drone no longer holds "
                    "these blocks; recreating them", name_.c_str(), e.what());
      }
    }
    if (!recovered) {
      try {
        rebuild_log_blocks();
        RCLCPP_WARN(logger_, "[%s] log blocks RECREATED (logReset + create + start); "
                    "watching for data", name_.c_str());
        rebuild_pending_ = false;
      } catch (const std::exception& e) {
        // A half-built block set can still deliver SOME data, which keeps the
        // no-telemetry trigger from ever firing again -- so the retry must be
        // driven by this flag, not by silence.
        rebuild_pending_ = true;
        RCLCPP_ERROR(logger_, "[%s] log-block rebuild FAILED (%s) - retrying in %.0f s",
                     name_.c_str(), e.what(), telemetry_watchdog_s_);
      }
    }
    last_log_rx_ = std::chrono::steady_clock::now();
    recovering_ = false;
  }

  void on_link_statistics_timer()
  {
    check_telemetry_watchdog();
    cf_.triggerLatencyMeasurement();

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_on_latency_;
    if (elapsed.count() > 1.0 / warning_freq_) {
      RCLCPP_WARN(logger_, "[%s] last latency update: %f s", name_.c_str(), elapsed.count());
    }

    auto stats = cf_.connectionStatsDelta();

    if (stats.ack_count > 0) {
      float ack_rate = stats.sent_count / stats.ack_count;
      if (ack_rate < min_ack_rate_) {
        RCLCPP_WARN(logger_, "[%s] Ack rate: %.1f %%", name_.c_str(), ack_rate * 100);
      }
    }

    if (publish_stats_) {
      crazyflie_interfaces::msg::ConnectionStatisticsArray msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = reference_frame_;
      msg.stats.resize(1);

      msg.stats[0].uri = cf_.uri();
      msg.stats[0].sent_count = stats.sent_count;
      msg.stats[0].sent_ping_count = stats.sent_ping_count;
      msg.stats[0].receive_count = stats.receive_count;
      msg.stats[0].enqueued_count = stats.enqueued_count;
      msg.stats[0].ack_count = stats.ack_count;

      publisher_connection_stats_->publish(msg);
    }
  }

  void on_latency(uint64_t latency_in_us)
  {
    last_latency_rx_ = std::chrono::steady_clock::now();   // telemetry watchdog
    if (latency_in_us / 1000.0 > max_latency_) {
      RCLCPP_WARN(logger_, "[%s] High latency: %.1f ms", name_.c_str(), latency_in_us / 1000.0);
    }
    last_on_latency_ = std::chrono::steady_clock::now();
    last_latency_in_ms_ = (uint16_t)(latency_in_us / 1000.0);
  }

private:
  rclcpp::Logger logger_;
  CrazyflieLogger cf_logger_;
  // set at destructor entry: log-data packets drained during cf_ teardown must
  // not be published - their ROS publishers are already destroyed (segfault)
  std::atomic<bool> shutting_down_{false};

  Crazyflie cf_;
  std::string message_buffer_;
  std::string name_;

  rclcpp::Node* node_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Service<Empty>::SharedPtr service_emergency_;
  rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
  rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
  rclcpp::Service<Land>::SharedPtr service_land_;
  rclcpp::Service<GoTo>::SharedPtr service_go_to_;
  rclcpp::Service<UploadTrajectory>::SharedPtr service_upload_trajectory_;
  rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
  rclcpp::Service<Arm>::SharedPtr service_arm_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_legacy_;
  rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr subscription_cmd_position_;
  rclcpp::Subscription<crazyflie_interfaces::msg::VelocityWorld>::SharedPtr subscription_cmd_velocity_world_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Hover>::SharedPtr subscription_cmd_hover_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_robot_description_;

  // logging
  std::string reference_frame_;

  std::unique_ptr<LogBlock<logPose>> log_block_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

  std::unique_ptr<LogBlock<logScan>> log_block_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan_;

  std::unique_ptr<LogBlock<logOdom>> log_block_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;

  std::unique_ptr<LogBlock<logStatus>> log_block_status_;
  bool status_has_radio_stats_;
  rclcpp::Publisher<crazyflie_interfaces::msg::Status>::SharedPtr publisher_status_;
  uint16_t previous_numRxBc;
  uint16_t previous_numRxUc;
  bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_unicast_;
  bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_broadcast_;
  bool first_status_msg_ = true;
  const CrazyflieBroadcaster* cfbc_;

  std::list<std::unique_ptr<LogBlockGeneric>> log_blocks_generic_;
  //: Per-request bound used ONLY while rebuild_log_blocks() runs; 0 at connect,
  //  where an unbounded wait is the documented behaviour.
  //: 300 ms was too tight -- MEASURED 2026-10-05, a log-block start answered
  //  later than that, so the rebuild aborted half-finished and the drone sat
  //  degraded until the NEXT stall. A drone that answers logReset answers these.
  static constexpr unsigned int RECOVERY_TIMEOUT_MS = 1000;
  unsigned int recovery_timeout_ms_{0};
  //: How to create every log block again from nothing - see rebuild_log_blocks().
  std::vector<std::function<void()>> log_block_builders_;
  //: Telemetry watchdog state - see check_telemetry_watchdog().
  double telemetry_watchdog_s_{0.0};
  bool recovering_{false};
  bool rebuild_pending_{false};
  //: Trajectory ids whose upload did not complete - start_trajectory refuses them.
  std::set<uint8_t> bad_trajectories_;
  bool commanded_flight_{false};
  bool last_mocap_z_valid_{false};
  float last_mocap_z_{0.0f};
  uint8_t period_pose_{0}, period_scan_{0}, period_odom_{0}, period_status_{0};
  std::vector<uint8_t> periods_generic_;
  std::chrono::steady_clock::time_point last_rebuild_attempt_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_log_rx_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_latency_rx_{std::chrono::steady_clock::now()};
  std::list<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr> publishers_generic_;

  // multithreading
  rclcpp::CallbackGroup::SharedPtr callback_group_cf_;
  rclcpp::TimerBase::SharedPtr spin_timer_;

  // link statistics
  rclcpp::TimerBase::SharedPtr link_statistics_timer_;
  rclcpp::TimerBase::SharedPtr keepalive_timer_;
  double keepalive_hz_{0.0};
  std::chrono::time_point<std::chrono::steady_clock> last_on_latency_;
  uint16_t last_latency_in_ms_;
  float warning_freq_;
  float max_latency_;
  float min_ack_rate_;
  float min_unicast_receive_rate_;
  float min_broadcast_receive_rate_;
  bool publish_stats_;
  rclcpp::Publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;
};

class CrazyflieServer : public rclcpp::Node
{
public:
  CrazyflieServer()
      : Node("crazyflie_server")
      , logger_(get_logger())
  {
    // Create callback groups (each group can run in a separate thread)
    callback_group_mocap_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_mocap = rclcpp::SubscriptionOptions();
    sub_opt_mocap.callback_group = callback_group_mocap_;

    callback_group_all_cmd_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_all_cmd = rclcpp::SubscriptionOptions();
    sub_opt_all_cmd.callback_group = callback_group_all_cmd_;

    callback_group_all_srv_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_cf_cmd_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_cf_srv_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // declare global params
    this->declare_parameter("all.broadcasts.num_repeats", 15);
    this->declare_parameter("all.broadcasts.delay_between_repeats_ms", 1);
    this->declare_parameter("firmware_params.query_all_values_on_connect", false);

    broadcasts_num_repeats_ = this->get_parameter("all.broadcasts.num_repeats").get_parameter_value().get<int>();
    broadcasts_delay_between_repeats_ms_ = this->get_parameter("all.broadcasts.delay_between_repeats_ms").get_parameter_value().get<int>();
    mocap_enabled_ = false;

    this->declare_parameter("robot_description", "");

    // Warnings
    // See the keep-alive comment in CrazyflieROS: below ~2 Hz the drone's own
    // 1 s radio-activity timeout starts deleting its log blocks.
    this->declare_parameter("keepalive_frequency", 4.0);
    this->declare_parameter("warnings.frequency", 1.0);
    float freq = this->get_parameter("warnings.frequency").get_parameter_value().get<float>();
    if (freq >= 0.0) {
      watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/freq)), std::bind(&CrazyflieServer::on_watchdog_timer, this), callback_group_all_srv_);
    }
    this->declare_parameter("warnings.motion_capture.warning_if_rate_outside", std::vector<double>({80.0, 120.0}));
    auto rate_range = this->get_parameter("warnings.motion_capture.warning_if_rate_outside").get_parameter_value().get<std::vector<double>>();
    mocap_min_rate_ = rate_range[0];
    mocap_max_rate_ = rate_range[1];

    this->declare_parameter("warnings.communication.max_unicast_latency", 10.0);
    this->declare_parameter("warnings.communication.min_unicast_ack_rate", 0.9);
    this->declare_parameter("warnings.communication.min_unicast_receive_rate", 0.9);
    this->declare_parameter("warnings.communication.min_broadcast_receive_rate", 0.9);
    this->declare_parameter("warnings.communication.publish_stats", false);

    publish_stats_ = this->get_parameter("warnings.communication.publish_stats").get_parameter_value().get<bool>();
    if (publish_stats_) {
      publisher_connection_stats_ = this->create_publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>("all/connection_statistics", 10);
    }

    // load crazyflies from params
    auto node_parameters_iface = this->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
        node_parameters_iface->get_parameter_overrides();

    auto cf_names = extract_names(parameter_overrides, "robots");
    for (const auto &name : cf_names) {
      bool enabled = parameter_overrides.at("robots." + name + ".enabled").get<bool>();
      if (enabled) {
        // Lookup type
        std::string cf_type = parameter_overrides.at("robots." + name + ".type").get<std::string>();
        // Find the connection setting for the given type
        const auto con = parameter_overrides.find("robot_types." + cf_type + ".connection");
        std::string constr = "crazyflie";
        if (con != parameter_overrides.end()) {
          constr = con->second.get<std::string>();
        }
        // Find the mocap setting
        const auto mocap_en = parameter_overrides.find("robot_types." + cf_type + ".motion_capture.enabled");
        if (mocap_en != parameter_overrides.end()) {
          if (mocap_en->second.get<bool>()) {
            mocap_enabled_ = true;
          }
        }

        // if it is a Crazyflie, try to connect
        if (constr == "crazyflie") {
          std::string uri = parameter_overrides.at("robots." + name + ".uri").get<std::string>();
          auto broadcastUri = Crazyflie::broadcastUriFromUnicastUri(uri);
          if (broadcaster_.count(broadcastUri) == 0) {
            broadcaster_.emplace(broadcastUri, std::make_unique<CrazyflieBroadcaster>(broadcastUri));
          }

          crazyflies_.emplace(name, std::make_unique<CrazyflieROS>(
            uri,
            cf_type,
            name,
            this,
            callback_group_cf_cmd_,
            callback_group_cf_srv_,
            broadcaster_.at(broadcastUri).get()));

          update_name_to_id_map(name, crazyflies_[name]->id());
        }
        else if (constr == "none") {
          // we still might want to track this object, so update our map
          uint8_t id = parameter_overrides.at("robots." + name + ".id").get<uint8_t>();
          update_name_to_id_map(name, id);
        } else {
          RCLCPP_INFO(logger_, "[all] Unknown connection type %s", constr.c_str());
        }
      }
    }

    this->declare_parameter("poses_qos_deadline", 100.0f);
    double poses_qos_deadline = this->get_parameter("poses_qos_deadline").get_parameter_value().get<double>();

    rclcpp::SensorDataQoS sensor_data_qos;
    sensor_data_qos.keep_last(1);
    sensor_data_qos.deadline(rclcpp::Duration(0/*s*/, 1e9/poses_qos_deadline /*ns*/));
    sub_poses_ = this->create_subscription<NamedPoseArray>(
      "poses", sensor_data_qos, std::bind(&CrazyflieServer::posesChanged, this, _1), sub_opt_mocap);

    // support for changing firmware params at runtime ("<cf>.params.*" and
    // "all.params.*") via the standard set_parameters service (ros2 param set).
    //
    // NOTE: upstream used a ParameterEventHandler on /parameter_events here.
    // On this rig the node's own parameter events are never delivered back to
    // itself (same-process DDS loopback failure: events from other processes
    // arrive, the node's own do not), so that handler never fired and runtime
    // param changes silently never reached the drones. This on-set callback
    // runs synchronously inside the set_parameters service call instead — no
    // pub/sub round-trip involved. Registered at the end of construction so
    // the startup parameter declarations do not re-push values already set.
    on_set_parameters_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CrazyflieServer::on_set_parameters, this, _1));

    // topics for "all"
    subscription_cmd_full_state_ = this->create_subscription<crazyflie_interfaces::msg::FullState>("all/cmd_full_state", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieServer::cmd_full_state_changed, this, _1), sub_opt_all_cmd);

    // services for "all"
    service_start_trajectory_ = this->create_service<StartTrajectory>("all/start_trajectory", std::bind(&CrazyflieServer::start_trajectory, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_takeoff_ = this->create_service<Takeoff>("all/takeoff", std::bind(&CrazyflieServer::takeoff, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_land_ = this->create_service<Land>("all/land", std::bind(&CrazyflieServer::land, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_go_to_ = this->create_service<GoTo>("all/go_to", std::bind(&CrazyflieServer::go_to, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_notify_setpoints_stop_ = this->create_service<NotifySetpointsStop>("all/notify_setpoints_stop", std::bind(&CrazyflieServer::notify_setpoints_stop, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    service_arm_ = this->create_service<Arm>("all/arm", std::bind(&CrazyflieServer::arm, this, _1, _2), get_service_qos(), callback_group_all_srv_);
    
    // This is the last service to announce and can be used to check if the server is fully available
    service_emergency_ = this->create_service<Empty>("all/emergency", std::bind(&CrazyflieServer::emergency, this, _1, _2), get_service_qos(), callback_group_all_srv_);
  }


private:
  void emergency(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] emergency()");
    for (int i = 0; i < broadcasts_num_repeats_; ++i)
    {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->emergencyStop();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
            std::shared_ptr<StartTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] start_trajectory(id=%d, timescale=%f, reversed=%d, group_mask=%d)",
                request->trajectory_id,
                request->timescale,
                request->reversed,
                request->group_mask);

    // REFUSE if this trajectory failed to upload to ANY drone. The per-drone
    // handler's check is not enough: the shows start trajectories over this
    // BROADCAST, which never touches a CrazyflieROS. On 2026-10-05 that gap let
    // a show fly cf5 through four trajectories it had never received (its
    // unicast receive rate was 0.00 -- "Sent: 1487. Received: 0") and the drone
    // flew whatever was in that memory slot until it was e-stopped.
    // Refusing for EVERYONE is the safe side: the formation holds position
    // instead of one drone flying garbage next to four flying the figure.
    std::string offenders;
    for (const auto& cf : crazyflies_) {
      if (cf.second && cf.second->trajectory_is_bad(request->trajectory_id)) {
        offenders += (offenders.empty() ? "" : ", ") + cf.first;
      }
    }
    if (!offenders.empty()) {
      RCLCPP_FATAL(logger_, "[all] REFUSING to start trajectory %d: its upload "
                   "FAILED on %s. Nothing will move. Re-upload before flying.",
                   request->trajectory_id, offenders.c_str());
      return;
    }

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->startTrajectory(request->trajectory_id,
                            request->timescale,
                            request->reversed,
                            request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void takeoff(const std::shared_ptr<Takeoff::Request> request,
                        std::shared_ptr<Takeoff::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] takeoff(height=%f m, duration=%f s, group_mask=%d)",
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto& bc : broadcaster_) {
        auto& cfbc = bc.second;
        cfbc->takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void land(const std::shared_ptr<Land::Request> request,
           std::shared_ptr<Land::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] land(height=%f m, duration=%f s, group_mask=%d)",
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto& bc : broadcaster_) {
        auto& cfbc = bc.second;
        cfbc->land(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void go_to(const std::shared_ptr<GoTo::Request> request,
            std::shared_ptr<GoTo::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, group_mask=%d)",
                request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->goTo(request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                         std::shared_ptr<NotifySetpointsStop::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] notify_setpoints_stop(remain_valid_millisecs%d, group_mask=%d)",
                request->remain_valid_millisecs,
                request->group_mask);

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->notifySetpointsStop(request->remain_valid_millisecs);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void arm(const std::shared_ptr<Arm::Request> request,
                         std::shared_ptr<Arm::Response> response)
  {
    RCLCPP_INFO(logger_, "[all] arm(%d)",
                request->arm);

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendArmingRequest(request->arm);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg)
  { 
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;

    for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendFullStateSetpoint(
          x, y, z,
          vx, vy, vz,
          ax, ay, az,
          qx, qy, qz, qw,
          rollRate, pitchRate, yawRate);
    }

  }

  void posesChanged(const NamedPoseArray::SharedPtr msg)
  {
    mocap_data_received_timepoints_.emplace_back(std::chrono::steady_clock::now());

    // Here, we send all the poses to all CFs
    // In Crazyswarm1, we only sent the poses of the same group (i.e. channel)


    // split the message into parts that require position update and pose update
    std::vector<CrazyflieBroadcaster::externalPosition> data_position;
    std::vector<CrazyflieBroadcaster::externalPose> data_pose;

    for (const auto& pose : msg->poses) {
      {
        // Feed the per-drone telemetry watchdog its airborne guard. This is the
        // only place that sees altitude for every drone regardless of how it was
        // commanded (broadcast /all/takeoff included).
        const auto cfit = crazyflies_.find(pose.name);
        if (cfit != crazyflies_.end() && cfit->second) {
          cfit->second->note_mocap_z((float)pose.pose.position.z);
        }
      }
      const auto iter = name_to_id_.find(pose.name);
      if (iter != name_to_id_.end()) {
        uint8_t id = iter->second;
        if (isnan(pose.pose.orientation.w)) {
          data_position.push_back({id, 
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z});
        } else {
          data_pose.push_back({id, 
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z,
            (float)pose.pose.orientation.x, (float)pose.pose.orientation.y, (float)pose.pose.orientation.z, (float)pose.pose.orientation.w});
        }
      }
    }

    // send position only updates to the swarm
    if (data_position.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPositions(data_position);
      }
    }

    // send pose only updates to the swarm
    if (data_pose.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPoses(data_pose);
      }
    }
  }

  // Runs synchronously inside every set_parameters service call (ros2 param
  // set, crazyflie_py setParam, ...) and pushes "<cf>.params.*" /
  // "all.params.*" changes to the drones over the radio. Replaces the
  // upstream /parameter_events subscription, whose self-events never arrived
  // (see registration site). Always returns success: the radio push is
  // best-effort and must not reject the ROS-side parameter change.
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter> &params)
  {
    for (const auto &p : params) {
      try {
        apply_firmware_param(p);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "[all] Failed to apply param %s: %s",
                     p.get_name().c_str(), e.what());
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void apply_firmware_param(const rclcpp::Parameter &p)
  {
    size_t params_pos = p.get_name().find(".params.");
    if (params_pos == std::string::npos) {
      return;
    }
    std::string cfname(p.get_name().begin(), p.get_name().begin() + params_pos);
    size_t prefixsize = params_pos + 8;
    if (cfname == "all") {
      size_t pos = p.get_name().find(".", prefixsize);
      std::string group(p.get_name().begin() + prefixsize, p.get_name().begin() + pos);
      std::string name(p.get_name().begin() + pos + 1, p.get_name().end());

      RCLCPP_INFO(
          logger_,
          "[all] Update parameter \"%s.%s\" to %s",
          group.c_str(),
          name.c_str(),
          p.value_to_string().c_str());

      for (auto& cf : crazyflies_) {
        const auto entry = cf.second->paramTocEntry(group, name);
        if (entry) {
          switch (entry->type)
          {
          case Crazyflie::ParamTypeUint8:
            broadcast_set_param<uint8_t>(group, name, p.as_int());
            break;
          case Crazyflie::ParamTypeInt8:
            broadcast_set_param<int8_t>(group, name, p.as_int());
            break;
          case Crazyflie::ParamTypeUint16:
            broadcast_set_param<uint16_t>(group, name, p.as_int());
            break;
          case Crazyflie::ParamTypeInt16:
            broadcast_set_param<int16_t>(group, name, p.as_int());
            break;
          case Crazyflie::ParamTypeUint32:
            broadcast_set_param<uint32_t>(group, name, p.as_int());
            break;
          case Crazyflie::ParamTypeInt32:
            broadcast_set_param<int32_t>(group, name, p.as_int());
            break;
          case Crazyflie::ParamTypeFloat:
            if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
              broadcast_set_param<float>(group, name, (float)p.as_int());
            } else {
              broadcast_set_param<float>(group, name, p.as_double());
            }
            break;
          }
          break;
        }
      }
    } else {
      auto iter = crazyflies_.find(cfname);
      if (iter != crazyflies_.end()) {
        iter->second->change_parameter(p);
      }
    }
  }

  void on_watchdog_timer()
  {
    auto now = std::chrono::steady_clock::now();

    // motion capture
    // a) check if the rate was within specified bounds
    if (mocap_data_received_timepoints_.size() >= 2) {
      double mean_rate = 0;
      double min_rate = std::numeric_limits<double>::max();
      double max_rate = 0;
      int num_rates_wrong = 0;
      for (size_t i = 0; i < mocap_data_received_timepoints_.size() - 1; ++i) {
        std::chrono::duration<double> diff = mocap_data_received_timepoints_[i+1] - mocap_data_received_timepoints_[i];
        double rate = 1.0 / diff.count();
        mean_rate += rate;
        min_rate = std::min(min_rate, rate);
        max_rate = std::max(max_rate, rate);
        if (rate <= mocap_min_rate_ || rate >= mocap_max_rate_) {
          num_rates_wrong++;
        }
      }
      mean_rate /= (mocap_data_received_timepoints_.size() - 1);

      if (num_rates_wrong > 0) {
        RCLCPP_WARN(logger_, "[all] Motion capture rate off (#: %d, Avg: %.1f, Min: %.1f, Max: %.1f)", num_rates_wrong, mean_rate, min_rate, max_rate);
      }
    } else if (mocap_enabled_) {
      // b) warn if no data was received
      RCLCPP_WARN(logger_, "[all] Motion capture did not receive data!");
    }

    mocap_data_received_timepoints_.clear();

    if (publish_stats_) {

      crazyflie_interfaces::msg::ConnectionStatisticsArray msg;
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "world"; // this is across broadcasters, which is not directly associated with single CFs; hence keep "world" here
      msg.stats.resize(broadcaster_.size());

      size_t i = 0;
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;

        auto stats = cfbc->connectionStatsDelta();

        msg.stats[i].uri = cfbc->uri();
        msg.stats[i].sent_count = stats.sent_count;
        msg.stats[i].sent_ping_count = stats.sent_ping_count;
        msg.stats[i].receive_count = stats.receive_count;
        msg.stats[i].enqueued_count = stats.enqueued_count;
        msg.stats[i].ack_count = stats.ack_count;
        ++i;
      }
      publisher_connection_stats_->publish(msg);
    }
  }

  template<class T>
  void broadcast_set_param(
    const std::string& group,
    const std::string& name,
    const T& value)
  {
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->setParam<T>(group.c_str(), name.c_str(), value);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void update_name_to_id_map(const std::string& name, uint8_t id)
  {
    const auto iter = name_to_id_.find(name);
    if (iter != name_to_id_.end()) {
      RCLCPP_WARN(logger_, "[all] At least two objects with the same id (%d, %s, %s)", id, name.c_str(), iter->first.c_str());
    } else {
      name_to_id_.insert(std::make_pair(name, id));
    }
  }

  private:
    rclcpp::Logger logger_;

    // subscribers
    rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
    rclcpp::Subscription<NamedPoseArray>::SharedPtr sub_poses_;

    // services
    rclcpp::Service<Empty>::SharedPtr service_emergency_;
    rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
    rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
    rclcpp::Service<Land>::SharedPtr service_land_;
    rclcpp::Service<GoTo>::SharedPtr service_go_to_;
    rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
    rclcpp::Service<Arm>::SharedPtr service_arm_;

    std::map<std::string, std::unique_ptr<CrazyflieROS>> crazyflies_;



    // broadcastUri -> broadcast object
    std::map<std::string, std::unique_ptr<CrazyflieBroadcaster>> broadcaster_;

    // maps CF name -> CF id
    std::map<std::string, uint8_t> name_to_id_;

    // global params
    int broadcasts_num_repeats_;
    int broadcasts_delay_between_repeats_ms_;

    // parameter updates
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;

    // sanity checks
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    bool mocap_enabled_;
    float mocap_min_rate_;
    float mocap_max_rate_;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> mocap_data_received_timepoints_;
    bool publish_stats_;
    rclcpp::Publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;

    // multithreading
    rclcpp::CallbackGroup::SharedPtr callback_group_mocap_;
    rclcpp::CallbackGroup::SharedPtr callback_group_all_cmd_;
    rclcpp::CallbackGroup::SharedPtr callback_group_all_srv_;
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd_;
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv_;
  };

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);


  auto node = std::make_shared<CrazyflieServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
