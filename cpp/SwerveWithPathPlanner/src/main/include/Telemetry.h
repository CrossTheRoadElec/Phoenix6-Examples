#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

#include "subsystems/CommandSwerveDrivetrain.h"

class Telemetry {
private:
    units::meters_per_second_t MaxSpeed;

    /* What to publish over networktables for telemetry */
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    /* Robot pose for field positioning */
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("Pose");
    nt::DoubleArrayPublisher fieldPub = table->GetDoubleArrayTopic("robotPose").Publish();
    nt::StringPublisher fieldTypePub = table->GetStringTopic(".type").Publish();

    /* Robot speeds for general checking */
    std::shared_ptr<nt::NetworkTable> driveStats = inst.GetTable("Drive");
    nt::DoublePublisher velocityX = driveStats->GetDoubleTopic("Velocity X").Publish();
    nt::DoublePublisher velocityY = driveStats->GetDoubleTopic("Velocity Y").Publish();
    nt::DoublePublisher speed = driveStats->GetDoubleTopic("Speed").Publish();
    nt::DoublePublisher odomFreq = driveStats->GetDoubleTopic("Odometry Frequency").Publish();

    /* Keep a reference of the last pose to calculate the speeds */
    frc::Pose2d m_lastPose{};
    units::second_t lastTime = ctre::phoenix6::utils::GetCurrentTime();

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
