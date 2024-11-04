#pragma once

#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"
#include "ctre/phoenix6/SignalLogger.hpp"

#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

using namespace ctre::phoenix6;

namespace subsystems {

/**
 * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
class CommandSwerveDrivetrain : public frc2::SubsystemBase, public swerve::SwerveDrivetrain {
    static constexpr units::second_t kSimLoopPeriod = 5_ms;
    std::unique_ptr<frc::Notifier> m_simNotifier;
    units::second_t m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    static constexpr frc::Rotation2d kBlueAlliancePerspectiveRotation{0_deg};
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    static constexpr frc::Rotation2d kRedAlliancePerspectiveRotation{180_deg};
    /* Keep track if we've ever applied the operator perspective before or not */
    bool m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    swerve::requests::ApplyRobotSpeeds m_pathApplyRobotSpeeds;

    /* Swerve requests to apply during SysId characterization */
    swerve::requests::SysIdSwerveTranslation m_translationCharacterization;
    swerve::requests::SysIdSwerveSteerGains m_steerCharacterization;
    swerve::requests::SysIdSwerveRotation m_rotationCharacterization;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    frc2::sysid::SysIdRoutine m_sysIdRoutineTranslation{
        frc2::sysid::Config{
            std::nullopt, // Use default ramp rate (1 V/s)
            4_V,          // Reduce dynamic step voltage to 4 V to prevent brownout
            std::nullopt, // Use default timeout (10 s)
            // Log state with SignalLogger class
            [](frc::sysid::State state)
            {
                SignalLogger::WriteString("SysIdTranslation_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t output) { SetControl(m_translationCharacterization.WithVolts(output)); },
            {},
            this
        }
    };

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    frc2::sysid::SysIdRoutine m_sysIdRoutineSteer{
        frc2::sysid::Config{
            std::nullopt, // Use default ramp rate (1 V/s)
            7_V,          // Use dynamic voltage of 7 V
            std::nullopt, // Use default timeout (10 s)
            // Log state with SignalLogger class
            [](frc::sysid::State state)
            {
                SignalLogger::WriteString("SysIdSteer_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t output) { SetControl(m_steerCharacterization.WithVolts(output)); },
            {},
            this
        }
    };

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of swerve::requests::SysIdSwerveRotation for info on importing the log to SysId.
     */
    frc2::sysid::SysIdRoutine m_sysIdRoutineRotation{
        frc2::sysid::Config{
            /* This is in radians per second², but SysId only supports "volts per second" */
            units::constants::detail::PI_VAL / 6 * (1_V / 1_s),
            /* This is in radians per second, but SysId only supports "volts" */
            units::constants::detail::PI_VAL * 1_V,
            std::nullopt, // Use default timeout (10 s)
            // Log state with SignalLogger class
            [](frc::sysid::State state)
            {
                SignalLogger::WriteString("SysIdRotation_State", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
            }
        },
        frc2::sysid::Mechanism{
            [this](units::volt_t output)
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                SetControl(m_rotationCharacterization.WithRotationalRate(output * (1_rad_per_s / 1_V)));
                /* also log the requested output for SysId */
                SignalLogger::WriteValue("Rotational_Rate", output * (1_rad_per_s / 1_V));
            },
            {},
            this
        }
    };

    /* The SysId routine to test */
    frc2::sysid::SysIdRoutine *m_sysIdRoutineToApply = &m_sysIdRoutineTranslation;

public:
    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * \param modules             Constants for each specific module
     */
    template <std::same_as<swerve::SwerveModuleConstants>... ModuleConstants>
    CommandSwerveDrivetrain(swerve::SwerveDrivetrainConstants const &driveTrainConstants, ModuleConstants const &... modules) :
        SwerveDrivetrain{driveTrainConstants, modules...}
    {
        if (utils::IsSimulation()) {
            StartSimThread();
        }
        ConfigureAutoBuilder();
    }

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param driveTrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<swerve::SwerveModuleConstants>... ModuleConstants>
    CommandSwerveDrivetrain(swerve::SwerveDrivetrainConstants const &driveTrainConstants, units::hertz_t odometryUpdateFrequency,
                    ModuleConstants const &... modules) :
        SwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency, modules...}
    {
        if (utils::IsSimulation()) {
            StartSimThread();
        }
        ConfigureAutoBuilder();
    }

    /**
     * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through getters in the classes.
     *
     * \param driveTrainConstants        Drivetrain-wide constants for the swerve drive
     * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * \param odometryStandardDeviation  The standard deviation for odometry calculation
     * \param visionStandardDeviation    The standard deviation for vision calculation
     * \param modules                    Constants for each specific module
     */
    template <std::same_as<swerve::SwerveModuleConstants>... ModuleConstants>
    CommandSwerveDrivetrain(swerve::SwerveDrivetrainConstants const &driveTrainConstants, units::hertz_t odometryUpdateFrequency,
                    std::array<double, 3> const &odometryStandardDeviation, std::array<double, 3> const &visionStandardDeviation,
                    ModuleConstants const &... modules) :
        SwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency,
            odometryStandardDeviation, visionStandardDeviation, modules...}
    {
        if (utils::IsSimulation()) {
            StartSimThread();
        }
        ConfigureAutoBuilder();
    }

    /**
     * \brief Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * \param request Function returning the request to apply
     * \returns Command to run
     */
    template <typename RequestSupplier>
        requires std::derived_from<
            std::remove_reference_t<std::invoke_result_t<RequestSupplier>>,
            swerve::requests::SwerveRequest>
    frc2::CommandPtr ApplyRequest(RequestSupplier request)
    {
        return Run([this, request=std::move(request)] {
            return SetControl(request());
        });
    }

    void Periodic() override;

    /**
     * \brief Runs the SysId Quasistatic test in the given direction for the routine
     * specified by m_sysIdRoutineToApply.
     *
     * \param direction Direction of the SysId Quasistatic test
     * \returns Command to run
     */
    frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction)
    {
        return m_sysIdRoutineToApply->Quasistatic(direction);
    }

    /**
     * \brief Runs the SysId Dynamic test in the given direction for the routine
     * specified by m_sysIdRoutineToApply.
     *
     * \param direction Direction of the SysId Dynamic test
     * \returns Command to run
     */
    frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction)
    {
        return m_sysIdRoutineToApply->Dynamic(direction);
    }

private:
    void ConfigureAutoBuilder();
    void StartSimThread();
};

}
