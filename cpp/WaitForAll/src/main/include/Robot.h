// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
  
private:
  ctre::phoenix6::hardware::TalonFX m_motor1{0, "*"}; // Pick the first CANivore bus
  ctre::phoenix6::hardware::Pigeon2 m_pigdey{1, "*"}; // Pick the first CANivore bus also
  ctre::phoenix6::hardware::TalonFX m_transcientMotor{20, "*"}; // This motor may or may not be on the bus, 
                                                                  // selectively power it to completely test this example 
  ctre::phoenix6::hardware::TalonFX m_motor2{0, "rio"}; // Pick the RIO bus to force a failure we can detect
  
  ctre::phoenix6::StatusSignal<units::turn_t> m_canbus1signal1 = m_motor1.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_canbus1signal2 = m_motor1.GetVelocity();
  ctre::phoenix6::StatusSignal<ctre::phoenix6::signals::ControlModeValue> m_canbus1signal3 = m_motor1.GetControlMode();
  ctre::phoenix6::StatusSignal<units::degree_t> m_canbus1signal4 = m_pigdey.GetYaw();
  ctre::phoenix6::StatusSignal<units::degree_t> m_canbus1signal5 = m_pigdey.GetRoll();
  
  ctre::phoenix6::StatusSignal<units::turn_t> m_canbus2signal1 = m_motor2.GetPosition();
  
  ctre::phoenix6::StatusSignal<units::turn_t> m_canbus1transcient1 = m_transcientMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_canbus1transcient2 = m_transcientMotor.GetVelocity();

  std::vector<ctre::phoenix6::BaseStatusSignal *> m_signalsAcrossCANbuses = {
    &m_canbus1signal1,
    &m_canbus2signal1
  };
  std::vector<ctre::phoenix6::BaseStatusSignal *> m_lotsOfSignals = {
    &m_canbus1signal1,
    &m_canbus1signal2,
    &m_canbus1signal3,
    &m_canbus1signal5
  };
  std::vector<ctre::phoenix6::BaseStatusSignal *> m_noSignals = {};
  std::vector<ctre::phoenix6::BaseStatusSignal *> m_tanscientSignals = {
    &m_canbus1signal1,
    &m_canbus1signal2,
    &m_canbus1transcient1,
    &m_canbus1transcient2
  };

  frc::XboxController m_joystick{0}; // Allow us to see the different errors
  
  units::second_t m_waitForAllTimeout{0.1_s};
};
