// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "Robot.h"

using namespace ctre::phoenixpro;

constexpr units::time::second_t print_period{500_ms};

void Robot::RobotInit() {
  /* Configure CANcoder */
  configs::CANcoderConfiguration toApply{};

  /* User can change the configs if they want, or leave it empty for factory-default */

  cancoder.GetConfigurator().Apply(toApply);

  /* Speed up signals to an appropriate rate */
  cancoder.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder.GetVelocity().SetUpdateFrequency(100_Hz);
}
void Robot::RobotPeriodic() {
  /* Every print_period get the CANcoder position/velocity and report it */
  if (frc::Timer::GetFPGATimestamp() - currentTime > print_period) {
    currentTime += print_period;
    /**
     * GetPosition automatically calls Refresh(), no need to manually refresh.
     * 
     * StatusSignalValues also have the "ostream <<" operator implemented, to provide
     * a useful print of the signal.
     */
    auto &pos = cancoder.GetPosition();
    std::cout << "Position is " << pos << " with " << pos.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl;

    /**
     * Get the velocity StatusSignalValue
     */
    auto &vel = cancoder.GetVelocity();
    /* This time wait for the signal to reduce latency */
    vel.WaitForUpdate(print_period); // Wait up to our period
    /**
     * This uses the explicit GetValue and GetUnits functions to print, even though it's not
     * necessary for the ostream print
     */
    std::cout << "Velocity is " <<
                  vel.GetValue().value() << " " <<
                  vel.GetUnits() << " with " <<
                  vel.GetTimestamp().GetLatency().value() << " seconds of latency" <<
                  std::endl;
    /**
     * Notice when running this example that the second print's latency is always shorter than the first print's latency.
     * This is because we explicitly wait for the signal using the WaitForUpdate() method instead of using the Refresh()
     * method, which only gets the last cached value (similar to how Phoenix v5 works).
     * This can be used to make sure we synchronously update our control loop from the CAN bus, reducing any latency or jitter in
     * CAN bus measurements.
     * When the device is on a CANivore, the reported latency is very close to the true latency of the sensor, as the CANivore
     * timestamps when it receives the frame. This can be further used for latency compensation.
     */
    std::cout << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  /**
   * When we teleop init, set the position of the Pigeon2 and wait for the setter to take affect.
   */
  cancoder.SetPosition(0.4_tr, 100_ms); // Set our position to .4 rotations and wait up to 100 ms for the setter to take affect
  cancoder.GetPosition().WaitForUpdate(100_ms); // And wait up to 100 ms for the position to take affect
  std::cout << "Set the position to 0.4 rotations, we are currently at " << cancoder.GetPosition() << std::endl;
}
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
