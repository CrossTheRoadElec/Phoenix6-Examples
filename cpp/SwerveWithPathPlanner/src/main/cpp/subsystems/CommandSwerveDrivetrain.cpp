#include "subsystems/CommandSwerveDrivetrain.hpp"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/controllers/PPHolonomicDriveController.h"
#include "wpi/driverstation/MatchState.hpp"
#include "wpi/driverstation/RobotState.hpp"
#include "wpi/system/RobotController.hpp"

using namespace subsystems;

void CommandSwerveDrivetrain::ConfigureAutoBuilder()
{
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        // Supplier of current robot pose
        [this] { return GetState().Pose; },
        // Consumer for seeding pose against auto
        [this](wpi::math::Pose2d const &pose) { return ResetPose(pose); },
        // Supplier of current robot velocity
        [this] { return GetState().Velocity; },
        // Consumer of ChassisVelocities and feedforwards to drive the robot
        [this](wpi::math::ChassisVelocities const &velocity, pathplanner::DriveFeedforwards const &feedforwards) {
            return SetControl(
                pathApplyRobotVelocity.WithVelocity(velocity.Discretize(20_ms))
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
            );
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // PID constants for translation
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            // PID constants for rotation
            pathplanner::PIDConstants{7.0, 0.0, 0.0}
        ),
        std::move(config),
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        [] {
            auto const alliance = wpi::MatchState::GetAlliance().value_or(wpi::Alliance::BLUE);
            return alliance == wpi::Alliance::RED;
        },
        this // Subsystem for requirements
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
    if (!hasAppliedOperatorPerspective || wpi::RobotState::IsDisabled()) {
        auto const allianceColor = wpi::MatchState::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == wpi::Alliance::RED
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    lastSimTime = utils::GetCurrentTime();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier = std::make_unique<wpi::Notifier>([this] {
        wpi::units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - lastSimTime;
        lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, wpi::RobotController::GetBatteryVoltage());
    });
    simNotifier->StartPeriodic(SIM_LOOP_PERIOD);
}
