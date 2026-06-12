#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"
#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/nt/DoubleArrayTopic.hpp"
#include "wpi/nt/DoubleTopic.hpp"
#include "wpi/nt/StringTopic.hpp"
#include "wpi/nt/StructArrayTopic.hpp"
#include "wpi/nt/StructTopic.hpp"
#include "wpi/smartdashboard/Mechanism2d.hpp"
#include "wpi/smartdashboard/MechanismLigament2d.hpp"

#include "subsystems/CommandSwerveDrivetrain.hpp"

class Telemetry {
private:
    wpi::units::meters_per_second_t MaxSpeed;

    /* What to publish over networktables for telemetry */
    wpi::nt::NetworkTableInstance inst = wpi::nt::NetworkTableInstance::GetDefault();

    /* Robot swerve drive state */
    std::shared_ptr<wpi::nt::NetworkTable> driveStateTable = inst.GetTable("DriveState");
    wpi::nt::StructPublisher<wpi::math::Pose2d> drivePose = driveStateTable->GetStructTopic<wpi::math::Pose2d>("Pose").Publish();
    wpi::nt::StructPublisher<wpi::math::ChassisVelocities> driveVelocity = driveStateTable->GetStructTopic<wpi::math::ChassisVelocities>("Velocity").Publish();
    wpi::nt::StructArrayPublisher<wpi::math::SwerveModulePosition> driveModulePositions = driveStateTable->GetStructArrayTopic<wpi::math::SwerveModulePosition>("ModulePositions").Publish();
    wpi::nt::StructArrayPublisher<wpi::math::SwerveModuleVelocity> driveModuleVelocities = driveStateTable->GetStructArrayTopic<wpi::math::SwerveModuleVelocity>("ModuleVelocities").Publish();
    wpi::nt::StructArrayPublisher<wpi::math::SwerveModuleVelocity> driveModuleTargets = driveStateTable->GetStructArrayTopic<wpi::math::SwerveModuleVelocity>("ModuleTargets").Publish();
    wpi::nt::DoublePublisher driveTimestamp = driveStateTable->GetDoubleTopic("Timestamp").Publish();
    wpi::nt::DoublePublisher driveOdometryFrequency = driveStateTable->GetDoubleTopic("OdometryFrequency").Publish();

    /* Robot pose for field positioning */
    std::shared_ptr<wpi::nt::NetworkTable> table = inst.GetTable("Pose");
    wpi::nt::DoubleArrayPublisher fieldPub = table->GetDoubleArrayTopic("Robot").Publish();
    wpi::nt::StringPublisher fieldTypePub = table->GetStringTopic(".type").Publish();

    /* Mechanisms to represent the swerve module states */
    std::array<wpi::Mechanism2d, 4> moduleMechanisms{
        wpi::Mechanism2d{1, 1},
        wpi::Mechanism2d{1, 1},
        wpi::Mechanism2d{1, 1},
        wpi::Mechanism2d{1, 1},
    };
    /* A direction and length changing ligament for speed representation */
    std::array<wpi::MechanismLigament2d *, 4> moduleSpeeds{
        moduleMechanisms[0].GetRoot("RootSpeed", 0.5, 0.5)->Append<wpi::MechanismLigament2d>("Speed", 0.5, 0_deg),
        moduleMechanisms[1].GetRoot("RootSpeed", 0.5, 0.5)->Append<wpi::MechanismLigament2d>("Speed", 0.5, 0_deg),
        moduleMechanisms[2].GetRoot("RootSpeed", 0.5, 0.5)->Append<wpi::MechanismLigament2d>("Speed", 0.5, 0_deg),
        moduleMechanisms[3].GetRoot("RootSpeed", 0.5, 0.5)->Append<wpi::MechanismLigament2d>("Speed", 0.5, 0_deg),
    };
    /* A direction changing and length constant ligament for module direction */
    std::array<wpi::MechanismLigament2d *, 4> moduleDirections{
        moduleMechanisms[0].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<wpi::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, wpi::util::Color8Bit{wpi::util::Color::WHITE}),
        moduleMechanisms[1].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<wpi::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, wpi::util::Color8Bit{wpi::util::Color::WHITE}),
        moduleMechanisms[2].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<wpi::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, wpi::util::Color8Bit{wpi::util::Color::WHITE}),
        moduleMechanisms[3].GetRoot("RootDirection", 0.5, 0.5)
            ->Append<wpi::MechanismLigament2d>("Direction", 0.1, 0_deg, 0, wpi::util::Color8Bit{wpi::util::Color::WHITE}),
    };

public:
    /**
     * Construct a telemetry object with the specified max speed of the robot.
     *
     * \param maxSpeed Maximum speed
     */
    Telemetry(wpi::units::meters_per_second_t maxSpeed);

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    void Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state);
};
