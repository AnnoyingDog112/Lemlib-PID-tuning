#pragma once

#include "lemlib/api.hpp"
#include "pros/imu.hpp"

// Struct for passing args to PID logging task
struct AngularPidLogArgs {
    lemlib::Chassis* chassis;
    pros::Imu* imu;
    lemlib::ControllerSettings controller;
    float target;
    int timeout_ms;
    int loop_delay_ms = 20;
    std::atomic<bool> stop{false};
};

// Launches PID logging in a separate task
void start_angular_pid_logging_task(
    lemlib::Chassis* chassis,
    pros::Imu* imu,
    const lemlib::ControllerSettings& controller,
    float target,
    int timeout_ms,
    int loop_delay_ms = 20  // Default loop delay
);

struct LateralPidLogArgs {
    lemlib::Chassis* chassis;
    pros::Imu* imu;
    lemlib::ControllerSettings controller;
    float target;
    int timeout_ms;
    int loop_delay_ms = 20;
    std::atomic<bool> stop{false};
};

void start_lateral_pid_logging_task(
    lemlib::Chassis* chassis,
    pros::Imu* imu,
    const lemlib::ControllerSettings& controller,
    float target,
    int timeout_ms,
    int loop_delay_ms = 20  // Default loop delay
);
