// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

using namespace ctre::phoenix;
using namespace ctre::phoenix6;

void Robot::RobotInit()
{
    configs::TalonFXConfiguration configs{};

    configs.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    configs.HardwareLimitSwitch.ForwardLimitSource = signals::ForwardLimitSourceValue::RemoteCANcoder;
    configs.HardwareLimitSwitch.ForwardLimitType = signals::ForwardLimitTypeValue::NormallyOpen;
    configs.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 1;
    configs.HardwareLimitSwitch.ForwardLimitEnable = true;
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
        status = m_fx.GetConfigurator().Apply(configs);
        if (status.IsOK()) break;
    }
    if (!status.IsOK()) {
        std::cout << "Could not apply configs, error code: " << status;
    }
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
    m_fx.SetControl(m_out.WithOutput(-m_joystick.GetLeftY())
                         .WithLimitForwardMotion(m_joystick.GetLeftBumper())
                         .WithLimitReverseMotion(m_joystick.GetRightBumper()));
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic()
{
    auto &fxSim = m_fx.GetSimState();
    fxSim.SetForwardLimit(m_joystick.GetAButton());
    fxSim.SetReverseLimit(m_joystick.GetBButton());

    auto &ccSim = m_cc.GetSimState();
    ccSim.SetMagnetHealth(m_joystick.GetYButton()
        ? signals::MagnetHealthValue::Magnet_Green
        : signals::MagnetHealthValue::Magnet_Red);
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
