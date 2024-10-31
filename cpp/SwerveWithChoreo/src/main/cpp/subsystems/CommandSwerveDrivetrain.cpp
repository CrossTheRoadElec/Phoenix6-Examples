#include "subsystems/CommandSwerveDrivetrain.h"

using namespace subsystems;

void CommandSwerveDrivetrain::FollowPath(frc::Pose2d const &pose, choreo::SwerveSample const &sample)
{
    m_pathThetaController.EnableContinuousInput(
        units::radian_t{-0.5_tr}.value(),
        units::radian_t{0.5_tr}.value()
    );

    auto targetSpeeds = sample.GetChassisSpeeds();
    targetSpeeds.vx += m_pathXController.Calculate(
        pose.X().value(), sample.x.value()
    ) * 1_mps;
    targetSpeeds.vy += m_pathYController.Calculate(
        pose.Y().value(), sample.y.value()
    ) * 1_mps;
    targetSpeeds.omega += m_pathThetaController.Calculate(
        pose.Rotation().Radians().value(), sample.heading.value()
    ) * 1_rad_per_s;

    std::vector moduleForcesX(sample.moduleForcesX.begin(), sample.moduleForcesX.end());
    std::vector moduleForcesY(sample.moduleForcesY.begin(), sample.moduleForcesY.end());

    SetControl(
        m_applyFieldSpeeds.WithSpeeds(targetSpeeds)
            .WithWheelForceFeedforwardsX(std::move(moduleForcesX))
            .WithWheelForceFeedforwardsY(std::move(moduleForcesY))
    );
}

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}
