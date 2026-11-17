// Copyright 2026 HAND contributors. SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <limits>
#include "xarm_api/joint_feedback.h"

class FeedbackCommit : public ::testing::Test {
protected:
    sensor_msgs::msg::JointState message;
    float position[7] = {0}, velocity[7] = {0}, effort[7] = {0};
    void SetUp() override {
        message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        message.position.assign(6, 0.25);
        message.velocity.assign(6, 0.5);
        message.effort.assign(6, 0.75);
        message.header.stamp.sec = 10;
    }
    int commit(int code = 0) {
        return xarm_api::commit_joint_feedback(message, code, position, velocity, effort,
                                              rclcpp::Time(20, 0));
    }
};
TEST_F(FeedbackCommit, TimeoutDoesNotRefreshTimestampOrOverwriteDataWithZeros) {
    EXPECT_EQ(commit(3), 3);
    EXPECT_EQ(message.header.stamp.sec, 10);
    EXPECT_DOUBLE_EQ(message.position[0], 0.25);
    EXPECT_DOUBLE_EQ(message.velocity[0], 0.5);
    EXPECT_DOUBLE_EQ(message.effort[0], 0.75);
}
TEST_F(FeedbackCommit, ValidZeroSampleIsCommitted) {
    EXPECT_EQ(commit(), 0);
    EXPECT_EQ(message.header.stamp.sec, 20);
    EXPECT_DOUBLE_EQ(message.position[0], 0.0);
}
TEST_F(FeedbackCommit, InvalidLastPositionRejectsWholeSample) {
    position[0] = 1.0;
    position[5] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(commit(), int(UXBUS_STATE::ERR_PARAM));
    EXPECT_DOUBLE_EQ(message.position[0], 0.25);
    EXPECT_EQ(message.header.stamp.sec, 10);
}
TEST_F(FeedbackCommit, InvalidVelocityIsNotPublishedAsValidFeedback) {
    velocity[5] = std::numeric_limits<float>::infinity();
    EXPECT_EQ(commit(), int(UXBUS_STATE::ERR_PARAM));
    EXPECT_EQ(message.header.stamp.sec, 10);
}
TEST_F(FeedbackCommit, InvalidEffortIsNotPublishedAsValidFeedback) {
    effort[5] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(commit(), int(UXBUS_STATE::ERR_PARAM));
    EXPECT_EQ(message.header.stamp.sec, 10);
}
TEST_F(FeedbackCommit, BadJointCountCannotOverrunSdkBuffers) {
    message.name.resize(8);
    EXPECT_EQ(commit(), int(UXBUS_STATE::ERR_PARAM));
    EXPECT_EQ(message.header.stamp.sec, 10);
}
