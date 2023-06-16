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

  /* Configure current limits */
  configs::MotionMagicConfigs mm{};
  mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
  mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
  // Take approximately 0.2 seconds to reach max accel 
  mm.MotionMagicJerk = 50;
  cfg.MotionMagic = mm;

  configs::Slot0Configs slot0{};
  slot0.kP = 60;
  slot0.kI = 0;
  slot0.kD = 0.1;
  slot0.kV = 0.12;
  slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving
  cfg.Slot0 = slot0;

  configs::FeedbackConfigs fdb{};
  fdb.SensorToMechanismRatio = 12.8;
  cfg.Feedback = fdb;

  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
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
  if(leftY > -0.1 && leftY < 0.1) leftY = 0;

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
