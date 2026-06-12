// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "wpi/drive/DifferentialDrive.hpp"
#include "wpi/driverstation/NiDsXboxController.hpp"
#include "wpi/framework/TimedRobot.hpp"
#include "wpi/math/kinematics/DifferentialDriveOdometry.hpp"
#include "wpi/simulation/DifferentialDrivetrainSim.hpp"
#include "wpi/smartdashboard/Field2d.hpp"
#include "wpi/units/velocity.hpp"

class Robot : public wpi::TimedRobot {
  static constexpr ctre::phoenix6::CANBus CANBUS{ctre::phoenix6::CANBus::Systemcore(1)};

  ctre::phoenix6::hardware::TalonFX leftFX{0, CANBUS};
  ctre::phoenix6::hardware::TalonFX rightFX{1, CANBUS};
  ctre::phoenix6::hardware::CANcoder leftSensor{0, CANBUS};
  ctre::phoenix6::hardware::CANcoder rightSensor{1, CANBUS};
  ctre::phoenix6::hardware::Pigeon2 imu{0, CANBUS};

  ctre::phoenix6::sim::TalonFXSimState &leftSim = leftFX.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState &rightSim = rightFX.GetSimState();
  ctre::phoenix6::sim::CANcoderSimState &leftSensSim = leftSensor.GetSimState();
  ctre::phoenix6::sim::CANcoderSimState &rightSensSim = rightSensor.GetSimState();
  ctre::phoenix6::sim::Pigeon2SimState &imuSim = imu.GetSimState();

  wpi::DifferentialDrive drivetrain{
    [this](double output) { leftFX.SetThrottle(output); },
    [this](double output) { rightFX.SetThrottle(output); }
  };

  wpi::NiDsXboxController joystick{0};

  /*
    * These numbers are an example AndyMark Drivetrain with some additional weight.
    * This is a fairly light robot.
    * Note you can utilize results from robot characterization instead of
    * theoretical numbers.
    * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-
    * characterization/introduction.html#introduction-to-robot-characterization
    */
  static constexpr wpi::units::dimensionless::scalar_t kGearRatio = 10.71; // Standard AndyMark Gearing reduction.
  static constexpr wpi::units::inch_t kWheelRadiusInches = 3_in;

  wpi::sim::DifferentialDrivetrainSim m_driveSim{
    wpi::math::DCMotor::KrakenX60FOC(2),
    kGearRatio,
    2.1_kg_sq_m, // MOI of 2.1 kg m^2 (from CAD model)
    26.5_kg,     // Mass of robot is 26.5 kg
    kWheelRadiusInches,
    0.546_m,     // Distance between wheels is _ meters.
  };

  wpi::Field2d m_field{};

  wpi::math::DifferentialDriveOdometry m_odometry{
    imu.GetRotation2d(),
    0_m, 0_m
  };
  
  int printCount{0};

  wpi::units::meter_t rotationsToMeters(wpi::units::turn_t rotations);
  wpi::units::meters_per_second_t rotationsToMetersVel(wpi::units::turns_per_second_t rotations);
  wpi::units::turn_t metersToRotations(wpi::units::meter_t meters);
  wpi::units::turns_per_second_t metersToRotationsVel(wpi::units::meters_per_second_t meters);

 public:
  Robot();
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void UtilityInit() override;
  void UtilityPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};
