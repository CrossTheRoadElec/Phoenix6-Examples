#include "Telemetry.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

Telemetry::Telemetry(units::meters_per_second_t maxSpeed) : MaxSpeed{maxSpeed}
{
    SignalLogger::Start();

    /* Set up the module state Mechanism2d telemetry */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i) {
        frc::SmartDashboard::PutData("Module " + std::to_string(i), &m_moduleMechanisms[i]);
    }
}

void Telemetry::Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state)
{
    /* Telemeterize the swerve drive state */
    drivePose.Set(state.Pose);
    driveSpeeds.Set(state.Speeds);
    driveModuleStates.Set(state.ModuleStates);
    driveModuleTargets.Set(state.ModuleTargets);
    driveModulePositions.Set(state.ModulePositions);
    driveTimestamp.Set(state.Timestamp.value());
    driveOdometryFrequency.Set(1.0 / state.OdometryPeriod.value());

    /* Also write to log file */
    SignalLogger::WriteStruct("DriveState/Pose", state.Pose);
    SignalLogger::WriteStruct("DriveState/Speeds", state.Speeds);
    SignalLogger::WriteStructArray<frc::SwerveModuleState>("DriveState/ModuleStates", state.ModuleStates);
    SignalLogger::WriteStructArray<frc::SwerveModuleState>("DriveState/ModuleTargets", state.ModuleTargets);
    SignalLogger::WriteStructArray<frc::SwerveModulePosition>("DriveState/ModulePositions", state.ModulePositions);
    SignalLogger::WriteValue("DriveState/OdometryPeriod", state.OdometryPeriod);

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.Set("Field2d");
    fieldPub.Set(std::array{
        state.Pose.X().value(),
        state.Pose.Y().value(),
        state.Pose.Rotation().Degrees().value()
    });

    /* Telemeterize each module state to a Mechanism2d */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i) {
        m_moduleDirections[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetLength(state.ModuleStates[i].speed / (2 * MaxSpeed));
    }
}
