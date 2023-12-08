// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <units/velocity.h>

class Robot : public frc::TimedRobot {
  ctre::phoenix6::hardware::TalonFX leftFX{0};
  ctre::phoenix6::hardware::TalonFX rightFX{1};
  ctre::phoenix6::hardware::CANcoder leftSensor{0};
  ctre::phoenix6::hardware::CANcoder rightSensor{1};
  ctre::phoenix6::hardware::Pigeon2 imu{0};

  ctre::phoenix6::sim::TalonFXSimState &leftSim = leftFX.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState &rightSim = rightFX.GetSimState();
  ctre::phoenix6::sim::CANcoderSimState &leftSensSim = leftSensor.GetSimState();
  ctre::phoenix6::sim::CANcoderSimState &rightSensSim = rightSensor.GetSimState();
  ctre::phoenix6::sim::Pigeon2SimState &imuSim = imu.GetSimState();

  frc::DifferentialDrive drivetrain{leftFX, rightFX};

  frc::XboxController joystick{0};

  /*
    * These numbers are an example AndyMark Drivetrain with some additional weight.
    * This is a fairly light robot.
    * Note you can utilize results from robot characterization instead of
    * theoretical numbers.
    * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-
    * characterization/introduction.html#introduction-to-robot-characterization
    */
  static constexpr units::dimensionless::scalar_t kGearRatio = 10.71; // Standard AndyMark Gearing reduction.
  static constexpr units::inch_t kWheelRadiusInches = 3_in;

  frc::sim::DifferentialDrivetrainSim m_driveSim{
    frc::DCMotor::Falcon500FOC(2),
    kGearRatio,
    2.1_kg_sq_m, // MOI of 2.1 kg m^2 (from CAD model)
    26.5_kg,     // Mass of robot is 26.5 kg
    kWheelRadiusInches,
    0.546_m,     // Distance between wheels is _ meters.
  };

  frc::Field2d m_field{};

  frc::DifferentialDriveOdometry m_odometry{
    imu.GetRotation2d(),
    0_m, 0_m
  };
  
  int printCount{0};

  units::meter_t rotationsToMeters(units::turn_t rotations);
  units::meters_per_second_t rotationsToMetersVel(units::turns_per_second_t rotations);
  units::turn_t metersToRotations(units::meter_t meters);
  units::turns_per_second_t metersToRotationsVel(units::meters_per_second_t meters);

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
};
