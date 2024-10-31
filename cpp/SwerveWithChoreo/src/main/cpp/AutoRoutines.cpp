#include "AutoRoutines.h"

frc2::CommandPtr AutoRoutines::SimplePathAuto(choreo::AutoFactory<choreo::SwerveSample> &factory)
{
    simplePathRoutine = factory.NewLoop("SimplePath Auto");
    simplePathTraj = factory.Trajectory("SimplePath", *simplePathRoutine);

    simplePathRoutine->Enabled().OnTrue(
        m_drivetrain.RunOnce([this] {
            auto const pose = simplePathTraj.GetInitialPose();
            if (pose) {
                m_drivetrain.ResetPose(*pose);
            } else {
                simplePathRoutine->Kill();
            }
        })
        .AndThen(simplePathTraj.Cmd())
    );
    return simplePathRoutine->Cmd();
}
