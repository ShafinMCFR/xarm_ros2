/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
           Vinman <vinman.cub@gmail.com>
 ============================================================================*/

#include "xarm_controller/hardware/uf_robot_system_hardware.h"
#include <cmath>
#include <chrono>

#define SERVICE_CALL_FAILED 999
#define SERVICE_IS_PERSISTENT_BUT_INVALID 998
#define ROBOT_IS_DISCONNECTED -1
#define WAIT_SERVICE_TIMEOUT 996
#define VELO_DURATION 1

namespace uf_robot_hardware
{
    static rclcpp::Logger LOGGER = rclcpp::get_logger("UFACTORY.RobotHW");

    void UFRobotSystemHardware::_init_ufactory_driver(void)
    {
        rclcpp::NodeOptions node_options;
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = rclcpp::Node::make_shared("ufactory_driver", node_options);


        std::thread th([this]() -> void {
            rclcpp::spin(node_);
            rclcpp::shutdown();
        });
        th.detach();

        robot_ip_ = "";
        auto it = info_.hardware_parameters.find("robot_ip");
        if (it != info_.hardware_parameters.end()) {
            robot_ip_ = it->second.substr(1);
        }
        if (robot_ip_ == "") {
            RCLCPP_ERROR(LOGGER, "[%s] No param named 'robot_ip'", robot_ip_.c_str());
            rclcpp::shutdown();
            exit(1);
        }

        std::string hw_ns = "xarm";
        it = info_.hardware_parameters.find("hw_ns");
        if (it != info_.hardware_parameters.end()) {
            hw_ns = it->second;
        }
        node_->set_parameter(rclcpp::Parameter("hw_ns", hw_ns));

        std::string prefix = "";
        it = info_.hardware_parameters.find("prefix");
        if (it != info_.hardware_parameters.end()) {
            prefix = it->second.substr(1);
        }
        if (prefix != "") {
            LOGGER = rclcpp::get_logger("UFACTORY." + prefix + "RobotHW");
        }
        node_->set_parameter(rclcpp::Parameter("prefix", prefix));

        std::string report_type = "normal";
        it = info_.hardware_parameters.find("report_type");
        if (it != info_.hardware_parameters.end()) {
            report_type = it->second;
        }
        node_->set_parameter(rclcpp::Parameter("report_type", report_type));

        std::string robot_type = "xarm";
        it = info_.hardware_parameters.find("robot_type");
        if (it != info_.hardware_parameters.end()) {
            robot_type = it->second;
        }

        RCLCPP_INFO(LOGGER, "[%s] namespace: %s", robot_ip_.c_str(), node_->get_namespace());
        RCLCPP_INFO(LOGGER, "[%s] robot_type: %s, hw_ns: %s, prefix: %s, report_type: %s", 
            robot_ip_.c_str(), robot_type.c_str(), hw_ns.c_str(), prefix.c_str(), report_type.c_str());

        int dof = 7;
        it = info_.hardware_parameters.find("dof");
        if (it != info_.hardware_parameters.end()) {
            dof = atoi(it->second.c_str());
        }
        node_->set_parameter(rclcpp::Parameter("dof", dof));

        int default_gripper_baud = 2000000;
        it = info_.hardware_parameters.find("default_gripper_baud");
        if (it != info_.hardware_parameters.end()) {
            default_gripper_baud = atoi(it->second.c_str());
        }
        node_->set_parameter(rclcpp::Parameter("default_gripper_baud", default_gripper_baud));

        bool baud_checkset = true;
        it = info_.hardware_parameters.find("baud_checkset");
        if (it != info_.hardware_parameters.end()) {
            baud_checkset = (it->second == "True" || it->second == "true");
        }
        node_->set_parameter(rclcpp::Parameter("baud_checkset", baud_checkset));

        bool add_gripper = true;
        it = info_.hardware_parameters.find("add_gripper");
        if (it != info_.hardware_parameters.end()) {
            add_gripper = (it->second == "True" || it->second == "true");
        }
        
        if (robot_type == "lite") add_gripper = false;
        node_->set_parameter(rclcpp::Parameter("add_gripper", add_gripper));

        bool add_bio_gripper = true;
        it = info_.hardware_parameters.find("add_bio_gripper");
        if (it != info_.hardware_parameters.end()) {
            add_bio_gripper = (it->second == "True" || it->second == "true");
        }
        
        if (robot_type == "lite") add_bio_gripper = false;
        node_->set_parameter(rclcpp::Parameter("add_bio_gripper", add_bio_gripper));

        it = info_.hardware_parameters.find("velocity_control");
        if (it != info_.hardware_parameters.end()) {
            velocity_control_ = (it->second == "True" || it->second == "true");
        }
        RCLCPP_INFO(LOGGER, "[%s] dof: %d, velocity_control: %d, add_gripper: %d, add_bio_gripper: %d, baud_checkset: %d, default_gripper_baud: %d", 
            robot_ip_.c_str(), dof, velocity_control_, add_gripper, add_bio_gripper, baud_checkset, default_gripper_baud);
        
        // 20250318, disable xarm_driver publish joint_states
        xarm_driver_.init(node_, robot_ip_, true);
        // 20250318, get joint_states msg reference from xarm_driver
        joint_state_msg_ = xarm_driver_.get_joint_states();
    }

    CallbackReturn UFRobotSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        info_ = info;
        velocity_control_ = false;
        read_code_ = 0;
        write_code_ = 0;

        initialized_ = false;
        fault_latched_ = false;
        have_feedback_ = false;
        read_ready_ = false;

        read_cnts_ = 0;
        read_max_time_ = 0;
        read_total_time_ = 0;
        read_failed_cnts_ = 0;
        memset(cmds_float_, 0, sizeof(cmds_float_));
        memset(prev_cmds_float_, 0, sizeof(prev_cmds_float_));

        position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        position_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        velocity_cmds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
            bool has_pos_cmd_interface = false;
            for (auto i = 0u; i < joint.command_interfaces.size(); ++i) {
                if (joint.command_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_cmd_interface = true;
                    break;
                }
            }
            if (!has_pos_cmd_interface) {
                RCLCPP_ERROR(LOGGER, "[%s] Joint '%s' has %ld command interfaces found, but not found %s command interface",
                    robot_ip_.c_str(), joint.name.c_str(), joint.command_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }

            bool has_pos_state_interface = false;
            for (auto i = 0u; i < joint.state_interfaces.size(); ++i) {
                if (joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION) {
                    has_pos_state_interface = true;
                    break;
                }
            }
            if (!has_pos_state_interface) {
                RCLCPP_ERROR(LOGGER, "[%s] Joint '%s' has %ld state interfaces found, but not found %s state interface",
                    robot_ip_.c_str(), joint.name.c_str(), joint.state_interfaces.size(), hardware_interface::HW_IF_POSITION
                );
                return CallbackReturn::ERROR;
            }
        }

        if (info_.joints.empty() || info_.joints.size() > 7) return CallbackReturn::ERROR;
        _init_ufactory_driver();
        RCLCPP_INFO(LOGGER, "[%s] System Sucessfully configured!", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> UFRobotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> UFRobotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (uint i = 0; i < info_.joints.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_cmds_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_cmds_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn UFRobotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
    {
        if (fault_latched_) return CallbackReturn::ERROR;
        sensor_msgs::msg::JointState sample;
        read_code_ = _read_feedback(sample);
        if (read_code_ != 0 || !_xarm_is_ready_read() || !_accept_feedback(sample)) {
            _fault("activation feedback", read_code_ != 0 ? read_code_ : UXBUS_STATE::ERR_PARAM);
            return CallbackReturn::ERROR;
        }
        // Seed position commands from a successful physical read, never zero placeholders.
        for (size_t i = 0; i < position_states_.size(); ++i) {
            position_cmds_[i] = position_states_[i];
            velocity_cmds_[i] = 0.0;
        }
        const int code = _activate_driver();
        if (code != 0) {
            _fault("activation", code);
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(LOGGER, "[%s] Hardware activated from measured joint positions", robot_ip_.c_str());
        return CallbackReturn::SUCCESS;
    }

    int UFRobotSystemHardware::_activate_driver()
    {
        // Existing controller faults must be resolved explicitly, not cleared here.
        int code = xarm_driver_.arm->motion_enable(true);
        if (code == 0) code = xarm_driver_.arm->set_mode(
            velocity_control_ ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
        if (code == 0) code = xarm_driver_.arm->set_state(XARM_STATE::START);
        if (code != 0) return code;
        // Reports arrive on another thread. This wait is outside the update loop.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!_xarm_is_ready_write()) {
            if (std::chrono::steady_clock::now() >= deadline) return UXBUS_STATE::STATE_NOT_READY;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return 0;
    }

    CallbackReturn UFRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
    {
        if (xarm_driver_.arm != nullptr) xarm_driver_.arm->set_mode(XARM_MODE::POSE);
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type UFRobotSystemHardware::_fault(const char *operation, int code)
    {
        if (!fault_latched_) {
            RCLCPP_ERROR(LOGGER, "[%s] %s failed (code %d); command transmission disabled. Restart required.",
                         robot_ip_.c_str(), operation, code);
        }
        fault_latched_ = true;
        have_feedback_ = false;
        initialized_ = false;
        return hardware_interface::return_type::ERROR;
    }

    bool UFRobotSystemHardware::_accept_feedback(const sensor_msgs::msg::JointState &sample)
    {
        const auto count = info_.joints.size();
        if (sample.name.size() != count || sample.position.size() != count ||
            sample.velocity.size() != count) return false;
        for (size_t i = 0; i < count; ++i) {
            if (sample.name[i] != info_.joints[i].name || !std::isfinite(sample.position[i]) ||
                !std::isfinite(sample.velocity[i])) return false;
        }
        for (size_t i = 0; i < count; ++i) {
            position_states_[i] = sample.position[i];
            velocity_states_[i] = sample.velocity[i];
            if (!initialized_) {
                position_cmds_[i] = position_states_[i];
                velocity_cmds_[i] = 0.0;
            }
        }
        have_feedback_ = true;
        return true;
    }

    int UFRobotSystemHardware::_read_feedback(sensor_msgs::msg::JointState &sample)
    {
        const int code = xarm_driver_.update_joint_states(initialized_);
        if (code == 0) sample = *joint_state_msg_;
        return code;
    }

    int UFRobotSystemHardware::_send_joint_command(float *commands)
    {
        return velocity_control_ ? xarm_driver_.arm->vc_set_joint_velocity(commands, true, VELO_DURATION)
            : xarm_driver_.arm->set_servo_angle_j(commands, 0, 0, 0);
    }

    hardware_interface::return_type UFRobotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (fault_latched_) return hardware_interface::return_type::ERROR;
        sensor_msgs::msg::JointState sample;
        read_code_ = _read_feedback(sample);
        read_ready_ = _xarm_is_ready_read();
        if (read_code_ != 0 || !read_ready_) {
            return _fault("joint read", read_code_ != 0 ? read_code_ : UXBUS_STATE::STATE_NOT_READY);
        }
        if (!_accept_feedback(sample)) return _fault("invalid joint feedback", UXBUS_STATE::ERR_PARAM);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type UFRobotSystemHardware::write(const rclcpp::Time &time, const rclcpp::Duration &)
    {
        if (fault_latched_) return hardware_interface::return_type::ERROR;
        if (!have_feedback_ || !_xarm_is_ready_write()) {
            return _fault("joint write readiness", UXBUS_STATE::STATE_NOT_READY);
        }
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const double value = velocity_control_ ? velocity_cmds_[i] : position_cmds_[i];
            if (!std::isfinite(value)) return _fault("non-finite joint command", UXBUS_STATE::ERR_PARAM);
            cmds_float_[i] = static_cast<float>(value);
            if (!std::isfinite(cmds_float_[i])) return _fault("joint command overflow", UXBUS_STATE::ERR_PARAM);
        }
        initialized_ = true;
        curr_write_time_ = time;
        if (velocity_control_ || curr_write_time_.seconds() - prev_write_time_.seconds() > 1 ||
            _check_cmds_is_change(prev_cmds_float_, cmds_float_)) {
            write_code_ = _send_joint_command(cmds_float_);
            if (write_code_ != 0) return _fault("joint write", write_code_);
            prev_write_time_ = curr_write_time_;
            for (size_t i = 0; i < info_.joints.size(); ++i) prev_cmds_float_[i] = cmds_float_[i];
        }
        return hardware_interface::return_type::OK;
    }

    bool UFRobotSystemHardware::_check_cmds_is_change(float *prev, float *cur, double threshold)
	{
		for (int i = 0; i < 7; i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
	}

    bool UFRobotSystemHardware::_xarm_is_ready_read(void)
    {
        static int last_err = xarm_driver_.curr_err;
		int curr_err = xarm_driver_.curr_err;
        if (curr_err != 0) {
            if (last_err != curr_err) {
                RCLCPP_ERROR(LOGGER, "[%s] UFACTORY Error detected! Code C%d -> [ %s ] ", robot_ip_.c_str(), curr_err, xarm_driver_.controller_error_interpreter(curr_err).c_str());
            }
        }
        last_err = curr_err;
        return last_err == 0;
    }

    bool UFRobotSystemHardware::_xarm_is_ready_write(void)
    {
        static bool last_not_ready = false;
        static int last_state = xarm_driver_.curr_state;
        static int last_mode = xarm_driver_.curr_mode;
        int curr_mode = xarm_driver_.curr_mode;
		int curr_state = xarm_driver_.curr_state;

        if (!_xarm_is_ready_read()) {
            last_not_ready = true;
            return false;
        }

        if (curr_state > 2) {
            if (last_state != curr_state) {
                last_state = curr_state;
                RCLCPP_WARN(LOGGER, "[%s] Robot State detected! State: %d", robot_ip_.c_str(), curr_state);
            }
            last_not_ready = true;
            return false;
        }
        last_state = curr_state;

        if (!(velocity_control_ ? curr_mode == XARM_MODE::VELO_JOINT : curr_mode == XARM_MODE::SERVO)) {
            if (last_mode != curr_mode) {
                last_mode = curr_mode;
                RCLCPP_WARN(LOGGER, "[%s] Robot Mode detected! Mode: %d", robot_ip_.c_str(), curr_mode);
            }
            last_not_ready = true;
            return false;
        }
        last_mode = curr_mode;

        if (last_not_ready) {
            RCLCPP_INFO(LOGGER, "[%s] Robot is Ready", robot_ip_.c_str());
        }
        last_not_ready = false;
        return true;
    }

}
