#include "AutoRoutines.h"

frc2::CommandPtr AutoRoutines::SimplePathAuto(choreo::AutoFactory<choreo::SwerveSample> &factory)
{
    auto routine = std::make_shared<choreo::AutoLoop<choreo::SwerveSample>>(
        factory.NewLoop("SimplePath Auto")
    );
    auto simplePath = std::make_shared<choreo::AutoTrajectory<choreo::SwerveSample>>(
        factory.Trajectory("SimplePath", *routine)
    );

    routine->Enabled().OnTrue(
        m_drivetrain.RunOnce([=] {
            auto const pose = simplePath->GetInitialPose();
            if (pose) {
                m_drivetrain.ResetPose(*pose);
            } else {
                routine->Kill();
            }
        })
        .AndThen(simplePath->Cmd())
    );
    return routine->Cmd();
}
