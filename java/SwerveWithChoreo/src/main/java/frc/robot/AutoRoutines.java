package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoRoutines {
    private final CommandSwerveDrivetrain m_drivetrain;

    public AutoRoutines(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    public Command simplePathAuto(AutoFactory factory) {
        final AutoLoop routine = factory.newLoop("SimplePath Auto");
        final AutoTrajectory simplePath = factory.trajectory("SimplePath", routine);

        routine.enabled().onTrue(
            m_drivetrain.runOnce(() ->
                m_drivetrain.resetPose(
                    simplePath.getInitialPose().orElseGet(() -> {
                        routine.kill();
                        return new Pose2d();
                    })
                )
            ).andThen(simplePath.cmd())
        );
        return routine.cmd();
    }
}
