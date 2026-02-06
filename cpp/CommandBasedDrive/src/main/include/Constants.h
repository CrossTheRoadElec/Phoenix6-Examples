// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/CANBus.hpp"

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

constexpr ctre::phoenix6::CANBus CANBUS = ctre::phoenix6::CANBus::RoboRIO();

/* Talon FX Device IDs */
constexpr int LEFT_LEADER_ID{1};
constexpr int LEFT_FOLLOWER_ID{2};
constexpr int RIGHT_LEADER_ID{3};
constexpr int RIGHT_FOLLOWER_ID{4};

/* Sensor IDs */
constexpr int PIGEON2_ID{1};
