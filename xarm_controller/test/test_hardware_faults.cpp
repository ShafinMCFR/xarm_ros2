// Copyright 2026 HAND contributors. SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <limits>
#include "xarm_controller/hardware/uf_robot_system_hardware.h"

using hardware_interface::return_type;
using uf_robot_hardware::CallbackReturn;

class OfflineHardware : public uf_robot_hardware::UFRobotSystemHardware {
public:
    int reads = 0, writes = 0, initializations = 0;
    int read_result = 0, write_result = 0, activation_result = 0, activations = 0;
    bool ready = true;
    sensor_msgs::msg::JointState next;
    std::vector<float> sent;

    hardware_interface::HardwareInfo description() {
        hardware_interface::HardwareInfo info;
        info.name = "offline_lite6";
        info.type = "system";
        for (int i = 1; i <= 6; ++i) {
            hardware_interface::ComponentInfo joint;
            joint.name = "joint" + std::to_string(i);
            hardware_interface::InterfaceInfo position, velocity;
            position.name = "position";
            velocity.name = "velocity";
            joint.command_interfaces = {position, velocity};
            joint.state_interfaces = {position, velocity};
            info.joints.push_back(joint);
        }
        return info;
    }
    void setup() {
        auto info = description();
        ASSERT_EQ(on_init(info), CallbackReturn::SUCCESS);
        for (auto &joint : info.joints) next.name.push_back(joint.name);
        next.position = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
        next.velocity.assign(6, 0.0);
    }
protected:
    void _init_ufactory_driver() override { ++initializations; }
    int _activate_driver() override { ++activations; return activation_result; }
    bool _xarm_is_ready_read() override { return ready; }
    bool _xarm_is_ready_write() override { return ready; }
    int _read_feedback(sensor_msgs::msg::JointState &sample) override {
        ++reads;
        sample = next;  // Includes the real SDK failure pattern: buffers may be zero.
        return read_result;
    }
    int _send_joint_command(float *commands) override {
        ++writes;
        sent.assign(commands, commands + 6);
        return write_result;
    }
};

static const rclcpp::Time now(10, 0);
static const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0/150);

TEST(HardwareFaults, SuccessfulReadSeedsHoldFromMeasuredPosition) {
    OfflineHardware hardware;
    hardware.setup();
    ASSERT_EQ(hardware.read(now, period), return_type::OK);
    ASSERT_EQ(hardware.write(now, period), return_type::OK);
    ASSERT_EQ(hardware.writes, 1);
    for (size_t i = 0; i < 6; ++i) EXPECT_FLOAT_EQ(hardware.sent[i], hardware.next.position[i]);
}

TEST(HardwareFaults, FailedReadPreservesStateAndPermanentlyBlocksWrites) {
    OfflineHardware hardware;
    hardware.setup();
    ASSERT_EQ(hardware.read(now, period), return_type::OK);
    const auto before = hardware.next.position;
    hardware.next.position.assign(6, 0.0);
    hardware.read_result = 3;
    EXPECT_EQ(hardware.read(now, period), return_type::ERROR);
    auto states = hardware.export_state_interfaces();
    for (size_t i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(states[2*i].get_value(), before[i]);
    const auto reads = hardware.reads;
    hardware.read_result = 0;
    EXPECT_EQ(hardware.read(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.reads, reads);  // No SDK retry/recovery loop.
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 0);
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

TEST(HardwareFaults, FailedWriteIsReturnedAndCannotBeRetried) {
    OfflineHardware hardware;
    hardware.setup();
    ASSERT_EQ(hardware.read(now, period), return_type::OK);
    hardware.write_result = 3;
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    hardware.write_result = 0;
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 1);
}

TEST(HardwareFaults, InvalidFeedbackIsRejectedBeforeAnyStateChanges) {
    OfflineHardware hardware;
    hardware.setup();
    ASSERT_EQ(hardware.read(now, period), return_type::OK);
    hardware.next.position[0] = 1.23;
    hardware.next.position[5] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(hardware.read(now, period), return_type::ERROR);
    EXPECT_DOUBLE_EQ(hardware.export_state_interfaces()[0].get_value(), 0.1);
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 0);
}

TEST(HardwareFaults, JointNameMismatchCannotSilentlyReorderCommands) {
    OfflineHardware hardware;
    hardware.setup();
    hardware.next.name[0] = "joint2";
    EXPECT_EQ(hardware.read(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 0);
}

TEST(HardwareFaults, InvalidCommandNeverReachesTransport) {
    OfflineHardware hardware;
    hardware.setup();
    ASSERT_EQ(hardware.read(now, period), return_type::OK);
    auto commands = hardware.export_command_interfaces();
    commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 0);
}

TEST(HardwareFaults, WriteBeforePhysicalFeedbackIsRejected) {
    OfflineHardware hardware;
    hardware.setup();
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 0);
}

TEST(HardwareFaults, ActualZeroAnglesAreValidData) {
    OfflineHardware hardware;
    hardware.setup();
    hardware.next.position.assign(6, 0.0);
    EXPECT_EQ(hardware.read(now, period), return_type::OK);
    EXPECT_EQ(hardware.write(now, period), return_type::OK);
}

TEST(HardwareFaults, InvalidDescriptionRejectedBeforeTransportInitialization) {
    OfflineHardware hardware;
    auto info = hardware.description();
    info.joints.clear();
    EXPECT_EQ(hardware.on_init(info), CallbackReturn::ERROR);
    EXPECT_EQ(hardware.initializations, 0);
}

TEST(HardwareFaults, ActivationCannotEnableMotorsWhenReadFails) {
    OfflineHardware hardware;
    hardware.setup();
    hardware.read_result = 3;
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
    EXPECT_EQ(hardware.activations, 0);
    EXPECT_EQ(hardware.writes, 0);
}

TEST(HardwareFaults, ActivationUsesActualPositionsAndZeroVelocity) {
    OfflineHardware hardware;
    hardware.setup();
    hardware.next.position = {3.56, -0.19, 0.38, 0.37, 0.58, -2.72};
    hardware.next.velocity.assign(6, 0.01);
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    EXPECT_EQ(hardware.activations, 1);
    auto commands = hardware.export_command_interfaces();
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(commands[2*i].get_value(), hardware.next.position[i]);
        EXPECT_DOUBLE_EQ(commands[2*i+1].get_value(), 0.0);
    }
}

TEST(HardwareFaults, ActivationFailureBlocksAllLaterCommands) {
    OfflineHardware hardware;
    hardware.setup();
    hardware.activation_result = 3;
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 0);
    EXPECT_EQ(hardware.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
    EXPECT_EQ(hardware.activations, 1);
}

TEST(HardwareFaults, FiniteDoubleOverflowCannotReachFloatTransport) {
    OfflineHardware hardware;
    hardware.setup();
    ASSERT_EQ(hardware.read(now, period), return_type::OK);
    auto commands = hardware.export_command_interfaces();
    commands[0].set_value(1e300);
    EXPECT_EQ(hardware.write(now, period), return_type::ERROR);
    EXPECT_EQ(hardware.writes, 0);
}
