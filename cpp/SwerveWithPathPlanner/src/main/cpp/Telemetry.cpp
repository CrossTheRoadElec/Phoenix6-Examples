#include "Telemetry.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6;

void Telemetry::Telemeterize(subsystems::CommandSwerveDrivetrain::SwerveDriveState const &state)
{
    /* Telemeterize the pose */
    frc::Pose2d const pose = state.Pose;
    fieldTypePub.Set("Field2d");
    fieldPub.Set(std::array{
        pose.X().value(),
        pose.Y().value(),
        pose.Rotation().Degrees().value()
    });

    /* Telemeterize the robot's general speeds */
    units::second_t const currentTime = utils::GetCurrentTime();
    units::second_t const diffTime = currentTime - lastTime;
    lastTime = currentTime;

    frc::Translation2d const distanceDiff = (pose - m_lastPose).Translation();
    m_lastPose = pose;

    frc::Translation2d const velocities = distanceDiff / diffTime.value();

    speed.Set(velocities.Norm().value());
    velocityX.Set(velocities.X().value());
    velocityY.Set(velocities.Y().value());
    odomFreq.Set((1.0 / state.OdometryPeriod).value());

    /* Telemeterize the module's states */
    for (size_t i = 0; i < m_moduleSpeeds.size(); ++i) {
        m_moduleSpeeds[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleDirections[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
        m_moduleSpeeds[i]->SetLength(state.ModuleStates[i].speed / (2 * MaxSpeed));

        frc::SmartDashboard::PutData("Module " + std::to_string(i), &m_moduleMechanisms[i]);
    }

    SignalLogger::WriteDoubleArray("odometry", {pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()});
    SignalLogger::WriteDouble("odom period", state.OdometryPeriod.value(), "seconds");
}
