#include "SwerveTelemetry.hpp"
#include "ctre/phoenix6/SignalLogger.hpp"

using namespace ctre::phoenix6;

SwerveTelemetry::SwerveTelemetry(wpi::units::meters_per_second_t maxSpeed) :
    MAX_SPEED{maxSpeed}
{
    /* optionally start the SignalLogger immediately */
    // SignalLogger::Start();
}

void SwerveTelemetry::Telemeterize(SwerveDriveState const &state)
{
    /* Write the swerve drive state to the log file */
    SignalLogger::WriteStruct<SwerveDriveState, size_t>("DriveState", state, state.ModulePositions.size(), state.Timestamp);

    /* Also telemeterize the swerve drive state */
    swerveTelem.Log("DriveState", state, state.ModulePositions.size());

    /* Telemeterize the pose to a Field2d */
    field.SetRobotPose(state.Pose);
    swerveTelem.Log("Pose", field);

    /* Telemeterize each module state to a Mechanism2d */
    for (size_t i = 0; i < moduleMechanisms.size(); ++i) {
        moduleDirections[i]->SetAngle(state.ModuleVelocities[i].angle.Degrees());
        moduleSpeeds[i]->SetAngle(state.ModuleVelocities[i].angle.Degrees());
        moduleSpeeds[i]->SetLength(state.ModuleVelocities[i].velocity / (2 * MAX_SPEED));

        swerveTelem.Log(std::format("Module {}", i), moduleMechanisms[i]);
    }
}
