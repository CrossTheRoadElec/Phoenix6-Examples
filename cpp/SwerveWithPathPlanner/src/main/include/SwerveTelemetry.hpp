#pragma once

#include "wpi/smartdashboard/Field2d.hpp"
#include "wpi/smartdashboard/Mechanism2d.hpp"
#include "wpi/smartdashboard/MechanismLigament2d.hpp"
#include "wpi/telemetry/Telemetry.hpp"

#include "subsystems/CommandSwerveDrivetrain.hpp"

class SwerveTelemetry {
private:
    using SwerveDriveState = subsystems::CommandSwerveDrivetrain::SwerveDriveState;

    wpi::units::meters_per_second_t MAX_SPEED;

    /* Table for swerve telemetry publishing */
    wpi::telemetry::TelemetryTable &swerveTelem = wpi::telemetry::GetTable("Swerve");

    /* Robot pose on a field */
    wpi::Field2d field{};

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
     * Constructs a swerve telemetry object.
     *
     * \param maxSpeed Maximum speed of the robot
     */
    SwerveTelemetry(wpi::units::meters_per_second_t maxSpeed);

    /** Accepts the swerve drive state and telemeterize it to SignalLogger and Telemetry. */
    void Telemeterize(SwerveDriveState const &state);
};
