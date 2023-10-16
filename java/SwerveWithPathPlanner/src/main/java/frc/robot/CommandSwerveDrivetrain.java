package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    public CommandBase applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return new RunCommand(()->{this.setControl(requestSupplier.get());}, this);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume  */
        updateSimState(0.02, 12);
    }
    /**
     * Takes the specified location and makes it the current pose for
     * field-relative maneuvers
     *
     * @param location Pose to make the current pose at.
     */
    @Override
    public void seedFieldRelative(Pose2d location) {
        try {
            m_stateLock.writeLock().lock();

            m_odometry.resetPosition(Rotation2d.fromDegrees(m_pigeon2.getYaw().getValue()), m_modulePositions, location);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }
}
