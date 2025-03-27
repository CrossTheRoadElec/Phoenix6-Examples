// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <iostream>

using namespace ctre::phoenix6;

constexpr units::time::second_t print_period{500_ms};

Robot::Robot() {
  /* Configure CANrange */
  configs::CANrangeConfiguration config{};

  config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000, it is a valid measurement.
  config.ProximityParams.ProximityThreshold = 0.1_m; // If CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal.

  config.ToFParams.UpdateMode = signals::UpdateModeValue::ShortRange100Hz; // Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.

  canRange.GetConfigurator().Apply(config);
}
void Robot::RobotPeriodic() {
  /* Every print_period get the CANcoder position/velocity and report it */
  if (frc::Timer::GetFPGATimestamp() - currentTime >= print_period) {
    currentTime += print_period;

    /**
     * getDistance() and getSignalStrength() automatically call refresh(), no need to manually refresh.
     * 
     * StatusSignalValues also have the toString method implemented, to provide
     * a useful print of the signal.
     */
    auto& distance = canRange.GetDistance();
    auto& signalStrength = canRange.GetSignalStrength();
    std::cout << "Distance is " << distance << " with a signal strength of " << signalStrength << " and " << distance.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl;

    /**
     * Get the isDetected StatusSignalValue without refreshing
     */
    auto& isDetected = canRange.GetIsDetected(false);
    /* This time wait for the signal to reduce latency */
    isDetected.WaitForUpdate(print_period); // Wait up to our period
    /**
     * This uses the explicit getValue and getUnits functions to print, even though it's not
     * necessary for the ostream print
     */
    std::cout <<
      "Is Detected is " <<
      isDetected.GetValue() << " " <<
      isDetected.GetUnits() << " with " <<
      isDetected.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl;
    /**
     * Notice when running this example that the second print's latency is always shorter than the first print's latency.
     * This is because we explicitly wait for the signal using the waitForUpdate() method instead of using the refresh()
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
