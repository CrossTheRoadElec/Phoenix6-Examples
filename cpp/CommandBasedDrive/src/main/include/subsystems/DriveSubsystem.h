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

    ctre::phoenixpro::controls::DutyCycleOut m_leftOut{0}; // Initialize with 0% output
    ctre::phoenixpro::controls::DutyCycleOut m_rightOut{0}; // Initialize with 0% output

    ctre::phoenixpro::hardware::Pigeon2 m_pigeon2{PIGEON2_ID, CANBUS_NAME};
 public:
    DriveSubsystem();
    
    void ArcadeDrive(double throttle, double wheel);
    auto& GetYaw() { return m_pigeon2.GetYaw(); }
    auto& GetLeftPos() { return m_leftLeader.GetPosition(); }
    auto& GetRightPos() { return m_rightLeader.GetPosition(); }

 private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    void InitializeTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator& cfg);
    void InitializePigeon2(ctre::phoenixpro::configs::Pigeon2Configurator& cfg);
};
