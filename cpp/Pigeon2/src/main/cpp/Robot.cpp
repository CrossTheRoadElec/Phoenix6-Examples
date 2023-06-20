// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "Robot.h"

using namespace ctre::phoenix6;

constexpr units::time::second_t print_period{500_ms};

void Robot::RobotInit() {
  /* Configure Pigeon2 */
  configs::Pigeon2Configuration toApply{};

  /* User can change the configs if they want, or leave it empty for factory-default */

  pidgey.GetConfigurator().Apply(toApply);

  /* Speed up signals to an appropriate rate */
  pidgey.GetYaw().SetUpdateFrequency(100_Hz);
  pidgey.GetGravityVectorZ().SetUpdateFrequency(100_Hz);
}
void Robot::RobotPeriodic() {
  /* Every print_period get the CANcoder position/velocity and report it */
  if (frc::Timer::GetFPGATimestamp() - currentTime > print_period) {
    currentTime += print_period;
    /**
     * GetYaw automatically calls Refresh(), no need to manually refresh.
     *
     * StatusSignalValues also have the "ostream <<" operator implemented, to provide
     * a useful print of the signal
     */
    auto &yaw = pidgey.GetYaw();
    std::cout << "Yaw is " << yaw << " with " << yaw.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl;

    /**
     * Get the Gravity Vector Z component StatusSignalValue
     */
    auto &gravityZ = pidgey.GetGravityVectorZ();
    /* This time wait for the signal to reduce latency */
    gravityZ.WaitForUpdate(print_period); // Wait up to our period
    /**
     * This uses the explicit GetValue and GetUnits methods to print, even though it's not
     * necessary for the ostream print
     */
    std::cout << "Gravity Vector in the Z direction is " <<
                  gravityZ.GetValue().value() << " " <<
                  gravityZ.GetUnits() << " with " <<
                  gravityZ.GetTimestamp().GetLatency().value() << " seconds of latency" <<
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
   * When we teleop init, set the yaw of the Pigeon2 and wait for the setter to take affect.
   */
  pidgey.SetYaw(144_deg, 100_ms); // Set our yaw to 144 degrees and wait up to 100 milliseconds for the setter to take affect
  pidgey.GetYaw().WaitForUpdate(100_ms); // And wait up to 100 milliseconds for the yaw to take affect
  std::cout << "Set the yaw to 144 degrees, we are currently at " << pidgey.GetYaw() << std::endl;
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
