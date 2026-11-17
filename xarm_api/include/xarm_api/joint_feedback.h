// Copyright 2026 HAND contributors. SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/time.hpp>
#include "xarm/core/instruction/uxbus_cmd_config.h"

namespace xarm_api {
// Commit only a complete validated sample, never failed/partial SDK output.
inline int commit_joint_feedback(sensor_msgs::msg::JointState &message, int result,
    const float *positions, const float *velocities, const float *efforts,
    const rclcpp::Time &stamp)
{
    if (result != 0) return result;
    const auto count = message.name.size();
    if (count == 0 || count > 7) return UXBUS_STATE::ERR_PARAM;
    for (size_t i = 0; i < count; ++i) {
        if (!std::isfinite(positions[i]) || !std::isfinite(velocities[i]) ||
            !std::isfinite(efforts[i])) return UXBUS_STATE::ERR_PARAM;
    }
    auto next = message;
    next.position.assign(positions, positions + count);
    next.velocity.assign(velocities, velocities + count);
    next.effort.assign(efforts, efforts + count);
    next.header.stamp = stamp;
    message = std::move(next);
    return 0;
}
}  // namespace xarm_api
