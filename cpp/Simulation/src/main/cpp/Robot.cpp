// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <iostream>

using namespace ctre::phoenix6;

void Robot::RobotInit()
{
  ctre::phoenix::StatusCode returnCode;

  configs::TalonFXConfiguration fxCfg{};
  fxCfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  int retryCount = 5;
  for (int i = 0; i < retryCount; ++i) {
    returnCode = leftFX.GetConfigurator().Apply(fxCfg);
    if (returnCode.IsOK()) break;
  }

  fxCfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  retryCount = 5;
  for (int i = 0; i < retryCount; ++i) {
    returnCode = rightFX.GetConfigurator().Apply(fxCfg);
    if (returnCode.IsOK()) break;
  }

  configs::CANcoderConfiguration ccCfg{};
  ccCfg.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  leftSensor.GetConfigurator().Apply(ccCfg);
  ccCfg.MagnetSensor.SensorDirection = signals::SensorDirectionValue::Clockwise_Positive;
  rightSensor.GetConfigurator().Apply(ccCfg);

  configs::Pigeon2Configuration imuCfg{};
  imu.GetConfigurator().Apply(imuCfg);

  /* Make sure all critical signals are synchronized */
  /*
   * Setting all these signals to 100hz means they get sent at the same time if
   * they're all on a CANivore
   */
  BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz,
      leftFX.GetPosition(),
      rightFX.GetPosition(),
      imu.GetYaw());

  /* Publish field pose data to read back from */
  frc::SmartDashboard::PutData("Field", &m_field);
}

void Robot::RobotPeriodic()
{
  /*
   * This will get the simulated sensor readings that we set
   * in the previous article while in simulation, but will use
   * real values on the robot itself.
   */
  m_odometry.Update(imu.GetRotation2d(),
                    rotationsToMeters(leftSensor.GetPosition().GetValue()),
                    rotationsToMeters(rightSensor.GetPosition().GetValue()));
  m_field.SetRobotPose(m_odometry.GetPose());

  if (++printCount >= 50) {
    printCount = 0;
    std::cout << "Left FX: " << leftFX.GetPosition() << std::endl;
    std::cout << "Right FX: " << rightFX.GetPosition() << std::endl;
    std::cout << "Left CANcoder: " << leftSensor.GetPosition() << std::endl;
    std::cout << "Right CANcoder: " << rightSensor.GetPosition() << std::endl;
    std::cout << "Left Forward limit: " << leftFX.GetForwardLimit() << std::endl;
    std::cout << "Left Reverse limit: " << leftFX.GetReverseLimit() << std::endl;
    std::cout << "Right Forward limit: " << rightFX.GetForwardLimit() << std::endl;
    std::cout << "Right Reverse limit: " << rightFX.GetReverseLimit() << std::endl;
    std::cout << "Pigeon2: " << imu.GetYaw() << std::endl;
    std::cout << "" << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
  drivetrain.CurvatureDrive(-joystick.GetLeftY(), -joystick.GetRightX(), joystick.GetRightStickButton());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit()
{
  /*
   * Set the orientation of the simulated devices relative to the robot chassis.
   * WPILib expects +V to be forward. Specify orientations to match that behavior.
   */
  /* left devices are CCW+ */
  leftSim.Orientation = sim::ChassisReference::CounterClockwise_Positive;
  leftSensSim.Orientation = sim::ChassisReference::CounterClockwise_Positive;
  /* right devices are CW+ */
  rightSim.Orientation = sim::ChassisReference::Clockwise_Positive;
  rightSensSim.Orientation = sim::ChassisReference::Clockwise_Positive;
}
void Robot::SimulationPeriodic()
{
  /* Pass the robot battery voltage to the simulated devices */
  leftSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  leftSensSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightSensSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  imuSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  /*
   * CTRE simulation is low-level, so SimState inputs
   * and outputs are not affected by user-level inversion.
   * However, inputs and outputs *are* affected by the mechanical
   * orientation of the device relative to the robot chassis,
   * as specified by the `orientation` field.
   *
   * WPILib expects +V to be forward. We have already configured
   * our orientations to match this behavior.
   */
  m_driveSim.SetInputs(leftSim.GetMotorVoltage(),
                       rightSim.GetMotorVoltage());

  /*
   * Advance the model by 20 ms. Note that if you are running this
   * subsystem in a separate thread or have changed the nominal
   * timestep of TimedRobot, this value needs to match it.
   */
  m_driveSim.Update(20_ms);

  /* Update all of our sensors. */
  auto const leftPos = metersToRotations(m_driveSim.GetLeftPosition());
  auto const leftVel = metersToRotationsVel(m_driveSim.GetLeftVelocity());
  auto const rightPos = metersToRotations(m_driveSim.GetRightPosition());
  auto const rightVel = metersToRotationsVel(m_driveSim.GetRightVelocity());
  leftSensSim.SetRawPosition(leftPos);
  leftSensSim.SetVelocity(leftVel);
  rightSensSim.SetRawPosition(rightPos);
  rightSensSim.SetVelocity(rightVel);
  leftSim.SetRawRotorPosition(leftPos * kGearRatio);
  leftSim.SetRotorVelocity(leftVel * kGearRatio);
  rightSim.SetRawRotorPosition(rightPos * kGearRatio);
  rightSim.SetRotorVelocity(rightVel * kGearRatio);
  imuSim.SetRawYaw(m_driveSim.GetHeading().Degrees());

  /*
   * If a bumper is pressed, trigger the forward limit switch to test it,
   * if a trigger is pressed, trigger the reverse limit switch
   */
  leftSim.SetForwardLimit(joystick.GetLeftBumperButton());
  leftSim.SetReverseLimit(joystick.GetLeftTriggerAxis() > 0.5);
  rightSim.SetForwardLimit(joystick.GetRightBumperButton());
  rightSim.SetReverseLimit(joystick.GetRightTriggerAxis() > 0.5);
}

units::meter_t Robot::rotationsToMeters(units::turn_t rotations)
{
    /* Every radian of rotation, the wheel travels this many inches */
    constexpr auto wheelDistancePerRad = kWheelRadiusInches / 1_rad;
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / kGearRatio;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * wheelDistancePerRad;
}

units::turn_t Robot::metersToRotations(units::meter_t meters)
{
    /* Every radian of rotation, the wheel travels this many inches */
    constexpr auto wheelDistancePerRad = kWheelRadiusInches / 1_rad;
    /* Now get wheel rotations from input meters */
    auto wheelRadians = meters / wheelDistancePerRad;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRadians * kGearRatio;
}

units::meters_per_second_t Robot::rotationsToMetersVel(units::turns_per_second_t rotations)
{
    /* Every radian of rotation, the wheel travels this many inches */
    constexpr auto wheelDistancePerRad = kWheelRadiusInches / 1_rad;
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / kGearRatio;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * wheelDistancePerRad;
}

units::turns_per_second_t Robot::metersToRotationsVel(units::meters_per_second_t meters)
{
    /* Every radian of rotation, the wheel travels this many inches */
    constexpr auto wheelDistancePerRad = kWheelRadiusInches / 1_rad;
    /* Now get wheel rotations from input meters */
    auto wheelRadians = meters / wheelDistancePerRad;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRadians * kGearRatio;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
