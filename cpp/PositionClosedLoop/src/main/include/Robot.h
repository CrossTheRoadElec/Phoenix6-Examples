// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

class Robot : public frc::TimedRobot {
  ctre::phoenix6::hardware::TalonFX m_fx{1, ctre::phoenix6::CANBus{"canivore"}};

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, use slot 0 */
  ctre::phoenix6::controls::PositionVoltage m_positionVoltage =
      ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
  /* Start at position 0, use slot 1 */
  ctre::phoenix6::controls::PositionTorqueCurrentFOC m_positionTorque =
      ctre::phoenix6::controls::PositionTorqueCurrentFOC{0_tr}.WithSlot(1);
  /* Keep a brake request so we can disable the motor */
  ctre::phoenix6::controls::StaticBrake m_brake{};

  frc::XboxController m_joystick{0};

 public:
  Robot();
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
};
