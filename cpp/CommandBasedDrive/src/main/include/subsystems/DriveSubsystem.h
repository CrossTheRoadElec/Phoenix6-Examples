// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenixpro/TalonFX.hpp>
#include <ctre/phoenixpro/Pigeon2.hpp>

class DriveSubsystem : public frc2::SubsystemBase {
private:
    ctre::phoenixpro::hardware::TalonFX m_leftLeader{LEFT_LEADER_ID, CANBUS_NAME};
    ctre::phoenixpro::hardware::TalonFX m_leftFollower{LEFT_FOLLOWER_ID, CANBUS_NAME};
    ctre::phoenixpro::hardware::TalonFX m_rightLeader{RIGHT_LEADER_ID, CANBUS_NAME};
    ctre::phoenixpro::hardware::TalonFX m_rightFollower{RIGHT_FOLLOWER_ID, CANBUS_NAME};

    ctre::phoenixpro::controls::DutyCycleOut m_leftOut{0};  // Initialize with 0% output
    ctre::phoenixpro::controls::DutyCycleOut m_rightOut{0}; // Initialize with 0% output

    ctre::phoenixpro::hardware::Pigeon2 m_pigeon2{PIGEON2_ID, CANBUS_NAME};

public:
    DriveSubsystem();

    /**
     * Drive the robot using an arcade drive format.
     *
     * This must be called periodically or else the control frames will not get sent
     * out, resulting in the TalonFXs disabling
     * 
     * \param fwd Forward/Reverse output
     * \param rot Left/Right output
     */
    void ArcadeDrive(double fwd, double rot);

    auto &GetYaw() { return m_pigeon2.GetYaw(); }
    auto &GetLeftPos() { return m_leftLeader.GetPosition(); }
    auto &GetRightPos() { return m_rightLeader.GetPosition(); }

private:
    /**
     * Initialize TalonFX device from the configurator object
     * \param cfg Configurator of the TalonFX device
     */
    void InitializeTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator &cfg);
    /**
     * Initialize Pigeon2 device from the configurator object
     * \param cfg Configurator of the Pigeon2 device
     */
    void InitializePigeon2(ctre::phoenixpro::configs::Pigeon2Configurator &cfg);
};
