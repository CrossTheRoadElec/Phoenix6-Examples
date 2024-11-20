#include "Telemetry.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

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
    std::array<double, 8> moduleStatesArray{};
    std::array<double, 8> moduleTargetsArray{};
    for (int i = 0; i < 4; ++i) {
        moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.Radians().value();
        moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speed.value();
        moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.Radians().value();
        moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speed.value();
    }
    SignalLogger::WriteDoubleArray("DriveState/Pose", {state.Pose.X().value(), state.Pose.Y().value(), state.Pose.Rotation().Degrees().value()});
    SignalLogger::WriteDoubleArray("DriveState/ModuleStates", moduleStatesArray);
    SignalLogger::WriteDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
    SignalLogger::WriteValue("DriveState/OdometryPeriod", state.OdometryPeriod);

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.Set("Field2d");
    fieldPub.Set(std::array{
        state.Pose.X().value(),
        state.Pose.Y().value(),
        state.Pose.Rotation().Degrees().value()
    });

    /* Telemeterize the module states to a Mechanism2d */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i) {
        m_moduleDirections[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetLength(state.ModuleStates[i].speed / (2 * MaxSpeed));

        frc::SmartDashboard::PutData("Module " + std::to_string(i), &m_moduleMechanisms[i]);
    }
}
