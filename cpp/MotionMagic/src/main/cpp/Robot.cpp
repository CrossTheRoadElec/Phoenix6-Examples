// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "sim/PhysicsSim.h"
#include "sim/TalonFXSimProfile.h"
#include <iostream>

using namespace ctre::phoenix6;

void Robot::SimulationInit() {
  PhysicsSim::GetInstance().AddTalonFX(m_motor, 0.001_kg_sq_m);
}
void Robot::SimulationPeriodic() {
  PhysicsSim::GetInstance().Run();
}

void Robot::RobotInit() {
  configs::TalonFXConfiguration cfg{};

  configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio = 12.8;

  /* Configure Motion Magic */
  configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
  mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.1 seconds to reach max accel 
  mm.MotionMagicJerk = 100;

  configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
  slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
  slot0.kI = 0; // No output for integrated error
  slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i) {
    status = m_motor.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
}
void Robot::RobotPeriodic() {
  if (m_printCount++ > 10) {
    m_printCount = 0;
    std::cout << "Pos: " << m_motor.GetPosition() << std::endl;
    std::cout << "Vel: " << m_motor.GetVelocity() << std::endl;
    std::cout << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  /* Deadband the joystick */
  double leftY = m_joystick.GetLeftY();
  if (fabs(leftY) < 0.1) leftY = 0;

  m_motor.SetControl(m_mmReq.WithPosition(leftY * 10_tr).WithSlot(0));
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
