#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "subsystems/CommandSwerveDrivetrain.h"

class Telemetry {
private:
    units::meters_per_second_t MaxSpeed;

    /* What to publish over networktables for telemetry */
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    /* Robot swerve drive state */
    std::shared_ptr<nt::NetworkTable> driveStateTable = inst.GetTable("DriveState");
    nt::StructPublisher<frc::Pose2d> drivePose = driveStateTable->GetStructTopic<frc::Pose2d>("Pose").Publish();
    nt::StructPublisher<frc::ChassisSpeeds> driveSpeeds = driveStateTable->GetStructTopic<frc::ChassisSpeeds>("Speeds").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> driveModuleStates = driveStateTable->GetStructArrayTopic<frc::SwerveModuleState>("ModuleStates").Publish();
    nt::StructArrayPublisher<frc::SwerveModuleState> driveModuleTargets = driveStateTable->GetStructArrayTopic<frc::SwerveModuleState>("ModuleTargets").Publish();
    nt::StructArrayPublisher<frc::SwerveModulePosition> driveModulePositions = driveStateTable->GetStructArrayTopic<frc::SwerveModulePosition>("ModulePositions").Publish();
    nt::DoublePublisher driveTimestamp = driveStateTable->GetDoubleTopic("Timestamp").Publish();
    nt::DoublePublisher driveOdometryFrequency = driveStateTable->GetDoubleTopic("OdometryFrequency").Publish();

    /* Robot pose for field positioning */
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("Pose");
    nt::DoubleArrayPublisher fieldPub = table->GetDoubleArrayTopic("robotPose").Publish();
    nt::StringPublisher fieldTypePub = table->GetStringTopic(".type").Publish();

    /* Mechanisms to represent the swerve module states */
    std::array<frc::Mechanism2d, 4> m_moduleMechanisms{
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
        frc::Mechanism2d{1, 1},
    };
    /* A direction and length changing ligament for speed representation */
    std::array<frc::MechanismLigament2d *, 4> m_moduleSpeeds{
        m_moduleMechanisms[0].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[1].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[2].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
        m_moduleMechanisms[3].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg),
    };
    /* A direction changing and length constant ligament for module direction */
    std::array<frc::MechanismLigament2d *, 4> m_moduleDirections{
        m_moduleMechanisms[0].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[1].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[2].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
        m_moduleMechanisms[3].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite}),
    };

public:
    /**
     * Construct a telemetry object with the specified max speed of the robot.
     *
     * \param maxSpeed Maximum speed
     */
    Telemetry(units::meters_per_second_t maxSpeed) : MaxSpeed{maxSpeed}
    {
        ctre::phoenix6::SignalLogger::Start();
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    void Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state);
};
