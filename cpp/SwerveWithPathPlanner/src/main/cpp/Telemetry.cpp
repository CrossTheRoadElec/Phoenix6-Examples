#include "Telemetry.hpp"
#include "wpi/smartdashboard/SmartDashboard.hpp"

using namespace ctre::phoenix6;

Telemetry::Telemetry(wpi::units::meters_per_second_t maxSpeed) : MaxSpeed{maxSpeed}
{
    SignalLogger::Start();

    /* Set up the module state Mechanism2d telemetry */
    for (size_t i = 0; i < moduleSpeeds.size(); ++i) {
        wpi::SmartDashboard::PutData("Module " + std::to_string(i), &moduleMechanisms[i]);
    }
}

void Telemetry::Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state)
{
    /* Telemeterize the swerve drive state */
    drivePose.Set(state.Pose);
    driveVelocity.Set(state.Velocity);
    driveModulePositions.Set(state.ModulePositions);
    driveModuleVelocities.Set(state.ModuleVelocities);
    driveModuleTargets.Set(state.ModuleTargets);
    driveTimestamp.Set(state.Timestamp.value());
    driveOdometryFrequency.Set(1.0 / state.OdometryPeriod.value());

    /* Also write to log file */
    SignalLogger::WriteStruct("DriveState/Pose", state.Pose);
    SignalLogger::WriteStruct("DriveState/Velocity", state.Velocity);
    SignalLogger::WriteStructArray<wpi::math::SwerveModulePosition>("DriveState/ModulePositions", state.ModulePositions);
    SignalLogger::WriteStructArray<wpi::math::SwerveModuleVelocity>("DriveState/ModuleVelocities", state.ModuleVelocities);
    SignalLogger::WriteStructArray<wpi::math::SwerveModuleVelocity>("DriveState/ModuleTargets", state.ModuleTargets);
    SignalLogger::WriteValue("DriveState/OdometryPeriod", state.OdometryPeriod);
    SignalLogger::WriteInteger("DriveState/FailedDaqs", state.FailedDaqs);

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.Set("Field2d");
    fieldPub.Set(std::array{
        state.Pose.X().value(),
        state.Pose.Y().value(),
        state.Pose.Rotation().Degrees().value()
    });

    /* Telemeterize each module state to a Mechanism2d */
    for (size_t i = 0; i < moduleSpeeds.size(); ++i) {
        moduleDirections[i]->SetAngle(state.ModuleVelocities[i].angle.Degrees());
        moduleSpeeds[i]->SetAngle(state.ModuleVelocities[i].angle.Degrees());
        moduleSpeeds[i]->SetLength(state.ModuleVelocities[i].velocity / (2 * MaxSpeed));
    }
}
