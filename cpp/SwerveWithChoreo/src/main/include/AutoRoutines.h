#pragma once

#include <choreo/auto/AutoFactory.h>

#include "subsystems/CommandSwerveDrivetrain.h"

class AutoRoutines {
private:
    subsystems::CommandSwerveDrivetrain &m_drivetrain;

    /* cache AutoTrajectory and AutoRoutine instances due to lifetime issues */
    std::optional<choreo::AutoLoop<choreo::SwerveSample>> simplePathRoutine;
    choreo::AutoTrajectory<choreo::SwerveSample> simplePathTraj;

public:
    AutoRoutines(subsystems::CommandSwerveDrivetrain &drivetrain) :
        m_drivetrain{drivetrain}
    {}

    frc2::CommandPtr SimplePathAuto(choreo::AutoFactory<choreo::SwerveSample> &factory);
};
