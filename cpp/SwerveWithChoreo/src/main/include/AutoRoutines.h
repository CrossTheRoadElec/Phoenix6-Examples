#pragma once

#include <choreo/auto/AutoFactory.h>

#include "subsystems/CommandSwerveDrivetrain.h"

class AutoRoutines {
private:
    subsystems::CommandSwerveDrivetrain &m_drivetrain;

public:
    AutoRoutines(subsystems::CommandSwerveDrivetrain &drivetrain) :
        m_drivetrain{drivetrain}
    {}

    frc2::CommandPtr SimplePathAuto(choreo::AutoFactory<choreo::SwerveSample> &factory);
};
