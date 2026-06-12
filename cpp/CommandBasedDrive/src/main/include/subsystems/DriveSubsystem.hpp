// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/Pigeon2.hpp"
#include "wpi/commands2/SubsystemBase.hpp"
#include "wpi/math/kinematics/DifferentialDriveOdometry.hpp"
#include "wpi/simulation/DifferentialDrivetrainSim.hpp"
#include "wpi/smartdashboard/Field2d.hpp"

#include "Constants.hpp"

class DriveSubsystem : public wpi::cmd::SubsystemBase {
private:
    ctre::phoenix6::hardware::TalonFX m_leftLeader{LEFT_LEADER_ID, CANBUS};
    ctre::phoenix6::hardware::TalonFX m_leftFollower{LEFT_FOLLOWER_ID, CANBUS};
    ctre::phoenix6::hardware::TalonFX m_rightLeader{RIGHT_LEADER_ID, CANBUS};
    ctre::phoenix6::hardware::TalonFX m_rightFollower{RIGHT_FOLLOWER_ID, CANBUS};

    ctre::phoenix6::controls::DutyCycleOut m_leftOut{0};  // Initialize with 0% output
    ctre::phoenix6::controls::DutyCycleOut m_rightOut{0}; // Initialize with 0% output

    ctre::phoenix6::hardware::Pigeon2 m_pigeon2{PIGEON2_ID, CANBUS};

    ctre::phoenix6::sim::TalonFXSimState &m_leftSimState = m_leftLeader.GetSimState();
    ctre::phoenix6::sim::TalonFXSimState &m_rightSimState = m_rightLeader.GetSimState();
    ctre::phoenix6::sim::TalonFXSimState &m_leftFollowerSimState = m_leftFollower.GetSimState();
    ctre::phoenix6::sim::TalonFXSimState &m_rightFollowerSimState = m_rightFollower.GetSimState();
    ctre::phoenix6::sim::Pigeon2SimState &m_pigeon2SimState = m_pigeon2.GetSimState();

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
        m_pigeon2.GetRotation2d(),
        0_m, 0_m
    };

    wpi::units::meter_t rotationsToMeters(wpi::units::turn_t rotations);
    wpi::units::meters_per_second_t rotationsToMetersVel(wpi::units::turns_per_second_t rotations);
    wpi::units::turn_t metersToRotations(wpi::units::meter_t meters);
    wpi::units::turns_per_second_t metersToRotationsVel(wpi::units::meters_per_second_t meters);

public:
    DriveSubsystem();

    /**
     * Drive the robot using an arcade drive format.
     *
     * This must be called periodically or else the control frames will not get sent
     * out, resulting in the TalonFXs disabling
     *
     * \param fwd Forward/Reverse output
     * \param rot Left/Right output
     */
    void ArcadeDrive(double fwd, double rot);

    auto &GetYaw() { return m_pigeon2.GetYaw(); }
    auto &GetLeftPos() { return m_leftLeader.GetPosition(); }
    auto &GetRightPos() { return m_rightLeader.GetPosition(); }

    void Periodic() override;
    void SimulationPeriodic() override;

private:
    /**
     * Initialize a left drive TalonFX device from the configurator object
     * \param cfg Configurator of the TalonFX device
     */
    void InitializeLeftDriveTalonFX(ctre::phoenix6::configs::TalonFXConfigurator &cfg);
    /**
     * Initialize a right drive TalonFX device from the configurator object
     * \param cfg Configurator of the TalonFX device
     */
    void InitializeRightDriveTalonFX(ctre::phoenix6::configs::TalonFXConfigurator &cfg);
    /**
     * Initialize Pigeon2 device from the configurator object
     * \param cfg Configurator of the Pigeon2 device
     */
    void InitializePigeon2(ctre::phoenix6::configs::Pigeon2Configurator &cfg);
};
