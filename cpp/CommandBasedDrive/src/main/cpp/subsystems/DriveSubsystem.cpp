// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ctre/phoenixpro/Utils.hpp"
#include "subsystems/DriveSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

using namespace ctre::phoenixpro;

DriveSubsystem::DriveSubsystem()
{
    /* Initialize all the devices */
    InitializeLeftDriveTalonFX(m_leftLeader.GetConfigurator());
    InitializeLeftDriveTalonFX(m_leftFollower.GetConfigurator());
    InitializeRightDriveTalonFX(m_rightLeader.GetConfigurator());
    InitializeRightDriveTalonFX(m_rightFollower.GetConfigurator());
    InitializePigeon2(m_pigeon2.GetConfigurator());

    /* Set followers to follow leader */
    m_leftFollower.SetControl(controls::Follower{m_leftLeader.GetDeviceID(), false});
    m_rightFollower.SetControl(controls::Follower{m_rightLeader.GetDeviceID(), false});

    /* Make sure all critical signals are synchronized */
    /* Setting all these signals to 100hz means they get sent at the same time if they're all on a CANivore */
    m_pigeon2.GetYaw().SetUpdateFrequency(100_Hz);
    m_leftLeader.GetPosition().SetUpdateFrequency(100_Hz);
    m_rightLeader.GetPosition().SetUpdateFrequency(100_Hz);

    /* Set the update frequency of the main requests to 0 so updates are sent immediately in the arcadeDrive method */
    m_leftOut.UpdateFreqHz = 0_Hz;
    m_rightOut.UpdateFreqHz = 0_Hz;
    
    /* Currently in simulation, we do not support FOC, so disable it while simulating */
    if(ctre::phoenixpro::IsSimulation())
    {
        m_leftOut.EnableFOC = false;
        m_rightOut.EnableFOC = false;
    }

    /* Publish field pose data to read back from */
    frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot)
{
    m_leftOut.Output = fwd + rot;
    m_rightOut.Output = fwd - rot;
    m_leftLeader.SetControl(m_leftOut);
    m_rightLeader.SetControl(m_rightOut);
}

void DriveSubsystem::InitializeLeftDriveTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator &cfg)
{
    configs::TalonFXConfiguration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */
    toApply.MotorOutput.Inverted = false;

    cfg.Apply(toApply);

    /* And initialize position to 0 */
    cfg.SetRotorPosition(0_tr);
}
void DriveSubsystem::InitializeRightDriveTalonFX(ctre::phoenixpro::configs::TalonFXConfigurator &cfg)
{
    configs::TalonFXConfiguration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */
    toApply.MotorOutput.Inverted = true;

    cfg.Apply(toApply);

    /* And initialize position to 0 */
    cfg.SetRotorPosition(0_tr);
}
void DriveSubsystem::InitializePigeon2(ctre::phoenixpro::configs::Pigeon2Configurator &cfg)
{
    configs::Pigeon2Configuration toApply{};

    /* User can change configs if they want, or leave this blank for factory-default */

    cfg.Apply(toApply);

    /* And initialize yaw to 0 */
    cfg.SetYaw(0_deg);
}

void DriveSubsystem::Periodic()
{
    /*
     * This will get the simulated sensor readings that we set
     * in the previous article while in simulation, but will use
     * real values on the robot itself.
     */
    m_odometry.Update(m_pigeon2.GetRotation2d(),
            rotationsToMeters(m_leftLeader.GetPosition().GetValue()),
            rotationsToMeters(m_rightLeader.GetPosition().GetValue()));
    m_field.SetRobotPose(m_odometry.GetPose());
}

void DriveSubsystem::SimulationPeriodic()
{
    /* Pass the robot battery voltage to the simulated Talon SRXs */
    m_leftSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_rightSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_leftFollowerSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_rightFollowerSimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    m_pigeon2SimState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

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
    m_driveSim.SetInputs(m_leftSimState.GetMotorVoltage(),
                         -m_rightSimState.GetMotorVoltage());

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
    m_leftSimState.SetRawRotorPosition(
        metersToRotations(
            m_driveSim.GetLeftPosition()));
    m_leftSimState.SetRotorVelocity(
        metersToRotationsVel(
            m_driveSim.GetLeftVelocity()));
    m_rightSimState.SetRawRotorPosition(
        metersToRotations(
            -m_driveSim.GetRightPosition()));
    m_rightSimState.SetRotorVelocity(
        metersToRotationsVel(
            -m_driveSim.GetRightVelocity()));
    m_leftFollowerSimState.SetRawRotorPosition(
        metersToRotations(
            m_driveSim.GetLeftPosition()));
    m_leftFollowerSimState.SetRotorVelocity(
        metersToRotationsVel(
            m_driveSim.GetLeftVelocity()));
    m_rightFollowerSimState.SetRawRotorPosition(
        metersToRotations(
            -m_driveSim.GetRightPosition()));
    m_rightFollowerSimState.SetRotorVelocity(
        metersToRotationsVel(
            -m_driveSim.GetRightVelocity()));
    m_pigeon2SimState.SetRawYaw(m_driveSim.GetHeading().Degrees());
}

units::meter_t DriveSubsystem::rotationsToMeters(units::turn_t rotations)
{
    /* Get circumference of wheel */
    auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / kGearRatio;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * circumference;
}

units::turn_t DriveSubsystem::metersToRotations(units::meter_t meters)
{
    /* Get circumference of wheel */
    auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply wheel rotations to inptu meters */
    auto wheelRotations = meters / circumference;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRotations * kGearRatio;
}

units::meters_per_second_t DriveSubsystem::rotationsToMetersVel(units::turns_per_second_t rotations)
{
    /* Get circumference of wheel */
    auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / kGearRatio;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * circumference;
}

units::turns_per_second_t DriveSubsystem::metersToRotationsVel(units::meters_per_second_t meters)
{
    /* Get circumference of wheel */
    auto circumference = kWheelRadiusInches * 2 * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply wheel rotations to inptu meters */
    auto wheelRotations = meters / circumference;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRotations * kGearRatio;
}
