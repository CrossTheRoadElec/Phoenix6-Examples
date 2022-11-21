// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "Robot.h"

static constexpr units::time::second_t print_period = 500_ms;

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  /* Every print_period get the CANcoder position/velocity and report it */
  if (frc::Timer::GetFPGATimestamp() - currentTime > print_period)
  {
    currentTime += print_period;
    /**
     * GetPosition automatically calls Refresh(), no need to manually refresh
     * StatusSignalValues also have the "ostream <<" operator implemented, to provide
     * a useful print of the signal
     */
    auto pos = cc.GetPosition();
    std::cout << "Position is " << pos << " with " << pos.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl; 
    /**
     * Get the velocity StatusSignalValue to check other components of the signal
     */
    auto vel = cc.GetVelocity();
    /* This time wait for the signal to reduce latency */
    vel.WaitForUpdate(print_period); // Wait up to our period
    std::cout << "Velocity is " << vel << " with " << vel.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl; 
    std::cout << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
