// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <iostream>

using namespace ctre::phoenixpro;

void Robot::RobotInit()
{
  configs::TalonFXConfiguration fxCfg{};
  fxCfg.MotorOutput.Inverted = spns::InvertedValue::Clockwise_Positive;
  ctre::phoenix::StatusCode returnCode;
  do
  {
      returnCode = leftFx.GetConfigurator().Apply(fxCfg);
  } while(!returnCode.IsOK());

  fxCfg.MotorOutput.Inverted = spns::InvertedValue::CounterClockwise_Positive;
  do
  {
      returnCode = rightFx.GetConfigurator().Apply(fxCfg);
  } while(!returnCode.IsOK());

  configs::CANcoderConfiguration ccCfg{};
  ccCfg.MagnetSensor.SensorDirection = spns::CANcoder_SensorDirectionValue::Clockwise_Positive;
  leftSensor.GetConfigurator().Apply(ccCfg);
  ccCfg.MagnetSensor.SensorDirection = spns::CANcoder_SensorDirectionValue::CounterClockwise_Positive;
  rightSensor.GetConfigurator().Apply(ccCfg);

  configs::Pigeon2Configuration imuCfg{};
  imu.GetConfigurator().Apply(imuCfg);

  /* Make sure all critical signals are synchronized */
  /*
   * Setting all these signals to 100hz means they get sent at the same time if
   * they're all on a CANivore
   */
  imu.GetYaw().SetUpdateFrequency(100_Hz);
  leftFx.GetPosition().SetUpdateFrequency(100_Hz);
  rightFx.GetPosition().SetUpdateFrequency(100_Hz);

  /* Publish field pose data to read back from */
  frc::SmartDashboard::PutData("Field", &m_field);
}
int printCount = 0;
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

  if (printCount++ > 50)
  {
    printCount = 0;
    std::cout << "Left FX: " << leftFx.GetPosition() << std::endl;
    std::cout << "Right FX: " << rightFx.GetPosition() << std::endl;
    std::cout << "Left CANcoder: " << leftSensor.GetPosition() << std::endl;
    std::cout << "Right CANcoder: " << rightSensor.GetPosition() << std::endl;
    std::cout << "Left Forward limit: " << leftFx.GetForwardLimit() << std::endl;
    std::cout << "Left Reverse limit: " << leftFx.GetReverseLimit() << std::endl;
    std::cout << "Right Forward limit: " << rightFx.GetForwardLimit() << std::endl;
    std::cout << "Right Reverse limit: " << rightFx.GetReverseLimit() << std::endl;
    std::cout << "Pigeon2: " << imu.GetYaw() << std::endl;
    std::cout << "" << std::endl;
  }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic()
{
  drivetrain.CurvatureDrive(-joystick.GetLeftY(), joystick.GetRightX(), joystick.GetRightStickButton());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic()
{
  /* Pass the robot battery voltage to the simulated Talon SRXs */
  leftSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  leftSensSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  rightSensSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  imuSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  /*
   * CTRE simulation is low-level, so SimCollection inputs
   * and outputs are not affected by SetInverted(). Only
   * the regular user-level API calls are affected.
   *
   * WPILib expects +V to be forward.
   * Positive motor output lead voltage is ccw. We observe
   * on our physical robot that this is reverse for the
   * right motor, so negate it.
   *
   * We are hard-coding the negation of the values instead of
   * using getInverted() so we can catch a possible bug in the
   * robot code where the wrong value is passed to setInverted().
   */
  m_driveSim.SetInputs(leftSim.GetMotorVoltage(),
                       -rightSim.GetMotorVoltage());

  /*
   * Advance the model by 20 ms. Note that if you are running this
   * subsystem in a separate thread or have changed the nominal
   * timestep of TimedRobot, this value needs to match it.
   */
  m_driveSim.Update(20_ms);

  /*
   * Update all of our sensors.
   *
   * Since WPILib's simulation class is assuming +V is forward,
   * but -V is forward for the right motor, we need to negate the
   * position reported by the simulation class. Basically, we
   * negated the input, so we need to negate the output.
   *
   * We also observe on our physical robot that a positive voltage
   * across the output leads results in a positive sensor velocity
   * for both the left and right motors, so we do not need to negate
   * the output any further.
   * If we had observed that a positive voltage results in a negative
   * sensor velocity, we would need to negate the output once more.
   */
  auto leftPos = metersToRotations(m_driveSim.GetLeftPosition());
  auto leftVel = metersToRotationsVel(m_driveSim.GetLeftVelocity());
  auto rightPos = metersToRotations(-m_driveSim.GetRightPosition());
  auto rightVel = metersToRotationsVel(-m_driveSim.GetRightVelocity());
  leftSensSim.SetRawPosition(leftPos);
  leftSensSim.SetVelocity(leftVel);
  rightSensSim.SetRawPosition(rightPos);
  rightSensSim.SetVelocity(rightVel);
  leftSim.SetRawRotorPosition(leftPos * kGearRatio);
  leftSim.SetRotorVelocity(leftVel * kGearRatio);
  rightSim.SetRawRotorPosition(rightPos * kGearRatio);
  rightSim.SetRotorVelocity(rightVel * kGearRatio);
  imuSim.SetRawYaw(m_driveSim.GetHeading().Degrees());

  /**
   *  If a bumper is pressed, trigger the forward limit switch to test it,
   * if a trigger is pressed, trigger the reverse limit switch
   */
  leftSim.SetForwardLimit(joystick.GetLeftBumper());
  leftSim.SetReverseLimit(joystick.GetLeftTriggerAxis() > 0.5);
  rightSim.SetForwardLimit(joystick.GetRightBumper());
  rightSim.SetReverseLimit(joystick.GetRightTriggerAxis() > 0.5);
}

units::meter_t Robot::rotationsToMeters(units::turn_t rotations)
{
  /* Get circumference of wheel */
  auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
  /* Every rotation of the wheel travels this many inches */
  /* Now apply gear ratio to input rotations */
  return rotations * circumference;
}

units::turn_t Robot::metersToRotations(units::meter_t meters)
{
  /* Get circumference of wheel */
  auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
  /* Every rotation of the wheel travels this many inches */
  /* Now apply wheel rotations to inptu meters */
  auto wheelRotations = meters / circumference;
  /* And multiply by gear ratio to get rotor rotations */
  return wheelRotations;
}

units::meters_per_second_t Robot::rotationsToMetersVel(units::turns_per_second_t rotations)
{
  /* Get circumference of wheel */
  auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
  /* Every rotation of the wheel travels this many inches */
  /* And multiply geared rotations by meters per rotation */
  return rotations * circumference;
}

units::turns_per_second_t Robot::metersToRotationsVel(units::meters_per_second_t meters)
{
  /* Get circumference of wheel */
  auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
  /* Every rotation of the wheel travels this many inches */
  /* Now apply wheel rotations to inptu meters */
  auto wheelRotations = meters / circumference;
  /* And multiply by gear ratio to get rotor rotations */
  return wheelRotations;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
