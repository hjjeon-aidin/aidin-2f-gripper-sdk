// aidin_gripper_driver - ROS2 driver node for the AIDIN BLDC gripper.
//
// Wraps the aidin::Gripper C++ SDK (self-contained Modbus RTU framing,
// no external Modbus library) and exposes:
//   * /aidin_gripper/state             (aidin_gripper_msgs/msg/GripperState)
//   * /aidin_gripper/activate          (std_srvs/srv/Trigger)
//   * /aidin_gripper/deactivate        (std_srvs/srv/Trigger)
//   * /aidin_gripper/home              (std_srvs/srv/Trigger)
//   * /aidin_gripper/full_init         (std_srvs/srv/Trigger)
//   * /aidin_gripper/stop              (std_srvs/srv/Trigger)
//   * /aidin_gripper/fault_clear       (std_srvs/srv/Trigger)
//   * /aidin_gripper/move_to           (aidin_gripper_msgs/srv/MoveTo)
//   * /aidin_gripper/emergency_release (aidin_gripper_msgs/srv/EmergencyRelease)
//   * /aidin_gripper/dev_cmd           (aidin_gripper_msgs/srv/DevCmd) — see below
//   * /aidin_gripper/tactile           (aidin_gripper_msgs/msg/TactileState) — publish_tactile:=true
//   * /aidin_gripper/tactile_zero      (std_srvs/srv/Trigger)
#include "aidin/gripper.hpp"
#include "aidin/registers.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "aidin_gripper_msgs/msg/gripper_state.hpp"
#include "aidin_gripper_msgs/msg/tactile_state.hpp"
#include "aidin_gripper_msgs/srv/dev_cmd.hpp"
#include "aidin_gripper_msgs/srv/move_to.hpp"
#include "aidin_gripper_msgs/srv/emergency_release.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// Set to 0 (or build with -DAIDIN_ENABLE_DEV_CMD=OFF, see CMakeLists) to
// compile the developer-command service (~/dev_cmd) out of the node
// entirely. ~/fault_clear stays available — clearing the latched fault is a
// normal operational action, not a factory one.
#ifndef AIDIN_ENABLE_DEV_CMD
#define AIDIN_ENABLE_DEV_CMD 1
#endif

using namespace std::chrono_literals;
using GripperStateMsg     = aidin_gripper_msgs::msg::GripperState;
using TactileStateMsg     = aidin_gripper_msgs::msg::TactileState;
using TactileTipMsg       = aidin_gripper_msgs::msg::TactileTip;
using DevCmdSrv           = aidin_gripper_msgs::srv::DevCmd;
using MoveToSrv           = aidin_gripper_msgs::srv::MoveTo;
using EmergencyReleaseSrv = aidin_gripper_msgs::srv::EmergencyRelease;
using TriggerSrv          = std_srvs::srv::Trigger;

class AidinGripperNode : public rclcpp::Node {
public:
    AidinGripperNode() : Node("aidin_gripper_driver") {
        // ----- Parameters -----
        port_         = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_     = declare_parameter<int>("baudrate", 115200);
        slave_id_     = declare_parameter<int>("slave_id", 1);
        parity_       = declare_parameter<std::string>("parity", "N");
        publish_rate_ = declare_parameter<double>("publish_rate_hz", 50.0);
        auto_activate_ = declare_parameter<bool>("auto_activate", true);
        auto_home_    = declare_parameter<bool>("auto_home", false);
        publish_tactile_ = declare_parameter<bool>("publish_tactile", false);

        // ----- Connect to the gripper -----
        try {
            gripper_.connect(port_, baudrate_,
                             parity_.empty() ? 'N' : parity_[0],
                             slave_id_);
            RCLCPP_INFO(get_logger(),
                "Connected to AIDIN gripper on %s @ %d %c81 (slave %d)",
                port_.c_str(), baudrate_, parity_[0], slave_id_);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(get_logger(), "Connect failed: %s", e.what());
            throw;
        }

        // Best-effort startup: activate + (optionally) home.
        if (auto_activate_) {
            try {
                gripper_.activate();
                RCLCPP_INFO(get_logger(), "Auto-activated.");
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "auto_activate failed: %s", e.what());
            }
        }
        if (auto_home_) {
            try {
                gripper_.home();
                RCLCPP_INFO(get_logger(), "Auto-homed.");
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "auto_home failed: %s", e.what());
            }
        }

        // ----- ROS interfaces -----
        state_pub_ = create_publisher<GripperStateMsg>(
            "~/state", rclcpp::SensorDataQoS());
        if (publish_tactile_) {
            tactile_pub_ = create_publisher<TactileStateMsg>(
                "~/tactile", rclcpp::SensorDataQoS());
            try {
                const uint16_t sensor = gripper_.readRegister(aidin::reg::CFG_SENSOR_TYPE);
                if (sensor != aidin::reg::SENSOR_TACTILE) {
                    RCLCPP_WARN(get_logger(),
                        "publish_tactile is set but CFG_SENSOR_TYPE=%u (tactile is 2) — "
                        "~/tactile will report LINK_NOT_DETECTED", static_cast<unsigned>(sensor));
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "CFG_SENSOR_TYPE read failed: %s", e.what());
            }
        }

        srv_activate_   = create_service<TriggerSrv>("~/activate",
            std::bind(&AidinGripperNode::onActivate, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_deactivate_ = create_service<TriggerSrv>("~/deactivate",
            std::bind(&AidinGripperNode::onDeactivate, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_home_       = create_service<TriggerSrv>("~/home",
            std::bind(&AidinGripperNode::onHome, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_full_init_  = create_service<TriggerSrv>("~/full_init",
            std::bind(&AidinGripperNode::onFullInit, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_stop_       = create_service<TriggerSrv>("~/stop",
            std::bind(&AidinGripperNode::onStop, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_fault_clear_ = create_service<TriggerSrv>("~/fault_clear",
            std::bind(&AidinGripperNode::onFaultClear, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_move_       = create_service<MoveToSrv>("~/move_to",
            std::bind(&AidinGripperNode::onMoveTo, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_release_    = create_service<EmergencyReleaseSrv>("~/emergency_release",
            std::bind(&AidinGripperNode::onEmergencyRelease, this,
                      std::placeholders::_1, std::placeholders::_2));
        srv_tactile_zero_ = create_service<TriggerSrv>("~/tactile_zero",
            std::bind(&AidinGripperNode::onTactileZero, this,
                      std::placeholders::_1, std::placeholders::_2));
#if AIDIN_ENABLE_DEV_CMD
        srv_dev_cmd_    = create_service<DevCmdSrv>("~/dev_cmd",
            std::bind(&AidinGripperNode::onDevCmd, this,
                      std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(get_logger(),
            "~/dev_cmd enabled (build with -DAIDIN_ENABLE_DEV_CMD=OFF to remove)");
#endif

        const auto period = std::chrono::microseconds(
            static_cast<int64_t>(1.0e6 / publish_rate_));
        publish_timer_ = create_wall_timer(period,
            std::bind(&AidinGripperNode::publishState, this));
    }

    ~AidinGripperNode() override {
        try { gripper_.disconnect(); } catch (...) {}
    }

private:
    // ---------------- Publishing ----------------
    void publishState() {
        aidin::GripperState s;
        {
            std::lock_guard<std::mutex> lk(io_mutex_);
            try {
                s = gripper_.readState();
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "readState failed: %s", e.what());
                return;
            }
        }
        GripperStateMsg msg;
        msg.header.stamp    = now();
        msg.header.frame_id = "aidin_gripper";
        msg.activated       = s.activated;
        msg.homed           = s.homed;
        msg.go_to_active    = s.goToActive;
        msg.stage           = static_cast<uint8_t>(s.stage);
        msg.object_state    = static_cast<uint8_t>(s.object);
        msg.fault           = s.fault;
        msg.latched_fault   = s.latchedFault;
        msg.position_echo   = s.positionEcho;
        msg.position        = s.position;
        msg.current         = s.current;
        msg.speed           = s.speed;
        msg.voltage         = s.voltage;
        state_pub_->publish(msg);

        if (tactile_pub_) publishTactile();
    }

    void publishTactile() {
        aidin::TactilePair tips;
        {
            std::lock_guard<std::mutex> lk(io_mutex_);
            try {
                tips = gripper_.readTactile();
            } catch (const std::exception& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "readTactile failed: %s", e.what());
                return;
            }
        }
        TactileStateMsg msg;
        msg.header.stamp    = now();
        msg.header.frame_id = "aidin_gripper";
        msg.right           = toMsg(tips[0]);
        msg.left            = toMsg(tips[1]);
        tactile_pub_->publish(msg);
    }

    static TactileTipMsg toMsg(const aidin::TactileTip& t) {
        TactileTipMsg m;
        m.link      = static_cast<uint8_t>(t.link());
        m.valid     = t.valid;
        m.contact   = t.contact;
        m.saturated = t.saturated;
        m.pos_x     = t.posX;
        m.pos_y     = t.posY;
        m.fx        = t.fx;
        m.fy        = t.fy;
        m.fz        = t.fz;
        m.rate_hz   = t.rateHz;
        m.status    = t.status;
        return m;
    }

    // ---------------- Service helpers ----------------
    template<typename ReqT, typename RespT, typename Fn>
    void runCommand(const ReqT& /*req*/, RespT& resp, const char* name, Fn fn) {
        std::lock_guard<std::mutex> lk(io_mutex_);
        try {
            fn();
            resp.success = true;
            resp.message = std::string(name) + " OK";
        } catch (const std::exception& e) {
            resp.success = false;
            resp.message = std::string(name) + " failed: " + e.what();
            RCLCPP_ERROR(get_logger(), "%s", resp.message.c_str());
        }
    }

    void onActivate(const std::shared_ptr<TriggerSrv::Request> req,
                    std::shared_ptr<TriggerSrv::Response> resp) {
        runCommand(*req, *resp, "activate", [&]{ gripper_.activate(); });
    }
    void onDeactivate(const std::shared_ptr<TriggerSrv::Request> req,
                      std::shared_ptr<TriggerSrv::Response> resp) {
        runCommand(*req, *resp, "deactivate", [&]{ gripper_.deactivate(); });
    }
    void onHome(const std::shared_ptr<TriggerSrv::Request> req,
                std::shared_ptr<TriggerSrv::Response> resp) {
        runCommand(*req, *resp, "home", [&]{ gripper_.home(); });
    }
    void onFullInit(const std::shared_ptr<TriggerSrv::Request> req,
                    std::shared_ptr<TriggerSrv::Response> resp) {
        // Full initialization (protocol v1.12.0, ACTION=0xA5): activate +
        // stroke calibration. Fingers sweep the full stroke twice.
        runCommand(*req, *resp, "full_init", [&]{ gripper_.fullInit(); });
    }
    void onStop(const std::shared_ptr<TriggerSrv::Request> req,
                std::shared_ptr<TriggerSrv::Response> resp) {
        runCommand(*req, *resp, "stop", [&]{ gripper_.stop(); });
    }
    void onTactileZero(const std::shared_ptr<TriggerSrv::Request> req,
                       std::shared_ptr<TriggerSrv::Response> resp) {
        // Sensor no-load zero on both tips (protocol v1.17.0+). Pads must be
        // untouched; each tip reads LINK_WAITING ~1.25 s, then LINK_LIVE.
        runCommand(*req, *resp, "tactile_zero", [&]{ gripper_.tactileZero(); });
    }

    void onFaultClear(const std::shared_ptr<TriggerSrv::Request> req,
                      std::shared_ptr<TriggerSrv::Response> resp) {
        runCommand(*req, *resp, "fault_clear", [&]{
            gripper_.writeRegister(aidin::reg::DEV_CMD, aidin::reg::DEV_CMD_FAULT_CLEAR);
            std::this_thread::sleep_for(50ms);
            const uint16_t ds = gripper_.readRegister(aidin::reg::DEV_STATUS);
            if (ds != aidin::reg::DEV_STATUS_OK) {
                throw aidin::GripperError("DEV_STATUS=" + std::to_string(ds));
            }
        });
    }

#if AIDIN_ENABLE_DEV_CMD
    void onDevCmd(const std::shared_ptr<DevCmdSrv::Request> req,
                  std::shared_ptr<DevCmdSrv::Response> resp) {
        std::lock_guard<std::mutex> lk(io_mutex_);
        // 주의: 완료까지 이 핸들러가 블로킹된다 (단일 스레드 executor 에서는
        // 그동안 ~/state 발행도 멈춘다) — 개발/공장 명령 전용이라 허용.
        try {
            gripper_.writeRegister(aidin::reg::DEV_CMD, req->cmd);
            const double timeout_s = req->timeout_sec > 0.0f ? req->timeout_sec : 5.0;
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(static_cast<int64_t>(timeout_s * 1000.0));
            uint16_t ds = aidin::reg::DEV_STATUS_RUNNING;
            while (std::chrono::steady_clock::now() < deadline) {
                ds = gripper_.readRegister(aidin::reg::DEV_STATUS);
                if (ds == aidin::reg::DEV_STATUS_OK ||
                    ds == aidin::reg::DEV_STATUS_ERROR) break;
                std::this_thread::sleep_for(50ms);
            }
            resp->dev_status = static_cast<uint8_t>(ds);
            resp->success    = (ds == aidin::reg::DEV_STATUS_OK);
            resp->message    = resp->success ? "dev_cmd OK"
                             : (ds == aidin::reg::DEV_STATUS_ERROR ? "dev_cmd ERROR"
                                                                   : "dev_cmd timeout");
        } catch (const std::exception& e) {
            resp->success = false;
            resp->message = std::string("dev_cmd failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
        }
    }
#endif

    void onMoveTo(const std::shared_ptr<MoveToSrv::Request> req,
                  std::shared_ptr<MoveToSrv::Response> resp) {
        std::lock_guard<std::mutex> lk(io_mutex_);
        try {
            const auto timeout = std::chrono::milliseconds(
                static_cast<int64_t>(req->timeout_sec * 1000.0));
            gripper_.moveTo(req->position, req->speed, req->force,
                            req->blocking, timeout);
            const auto s = gripper_.readState();
            resp->success            = true;
            resp->message            = "move_to OK";
            resp->final_position     = s.position;
            resp->final_object_state = static_cast<uint8_t>(s.object);
        } catch (const std::exception& e) {
            resp->success = false;
            resp->message = std::string("move_to failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
        }
    }

    void onEmergencyRelease(const std::shared_ptr<EmergencyReleaseSrv::Request> req,
                            std::shared_ptr<EmergencyReleaseSrv::Response> resp) {
        std::lock_guard<std::mutex> lk(io_mutex_);
        try {
            gripper_.emergencyRelease(req->close_direction);
            resp->success = true;
            resp->message = "emergency_release latched (call activate to clear)";
        } catch (const std::exception& e) {
            resp->success = false;
            resp->message = std::string("emergency_release failed: ") + e.what();
            RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
        }
    }

    // ---------------- State ----------------
    aidin::Gripper gripper_;
    std::mutex     io_mutex_;

    std::string port_, parity_;
    int         baudrate_{}, slave_id_{};
    double      publish_rate_{};
    bool        auto_activate_{}, auto_home_{}, publish_tactile_{};

    rclcpp::Publisher<GripperStateMsg>::SharedPtr      state_pub_;
    rclcpp::Publisher<TactileStateMsg>::SharedPtr      tactile_pub_;
    rclcpp::Service<TriggerSrv>::SharedPtr             srv_activate_;
    rclcpp::Service<TriggerSrv>::SharedPtr             srv_deactivate_;
    rclcpp::Service<TriggerSrv>::SharedPtr             srv_home_;
    rclcpp::Service<TriggerSrv>::SharedPtr             srv_full_init_;
    rclcpp::Service<TriggerSrv>::SharedPtr             srv_stop_;
    rclcpp::Service<TriggerSrv>::SharedPtr             srv_fault_clear_;
    rclcpp::Service<MoveToSrv>::SharedPtr              srv_move_;
    rclcpp::Service<EmergencyReleaseSrv>::SharedPtr    srv_release_;
    rclcpp::Service<TriggerSrv>::SharedPtr             srv_tactile_zero_;
#if AIDIN_ENABLE_DEV_CMD
    rclcpp::Service<DevCmdSrv>::SharedPtr              srv_dev_cmd_;
#endif
    rclcpp::TimerBase::SharedPtr                       publish_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<AidinGripperNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("aidin_gripper_driver"),
                     "Fatal: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
