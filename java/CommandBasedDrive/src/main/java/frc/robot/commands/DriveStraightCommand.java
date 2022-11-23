package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenixpro.StatusSignalValue;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraightCommand extends CommandBase {
    DoubleSupplier m_throttle;
    DriveStraightThread m_driveStraightThread;
    DriveSubsystem m_drivebase;

    public DriveStraightCommand(DriveSubsystem drivebase, DoubleSupplier throttle) {
        m_throttle = throttle;
        m_driveStraightThread = new DriveStraightThread(drivebase.getYaw(), (val) -> correctiveDrive(val));
        m_drivebase = drivebase;
        addRequirements(drivebase);
        m_driveStraightThread.start();
    }

    private void correctiveDrive(double correction)
    {
        m_drivebase.arcadeDrive(m_throttle.getAsDouble(), correction);
    }

    /**
     * We do the calculation and updates in a separate thread to make use
     * of the WaitForUpdate from StatusSignalValue. This allows us to
     * react immediately to new data, instead of reacting to potentially
     * old data
     */
    private class DriveStraightThread extends Thread {
        final double MAX_UPDATE_PERIOD = 0.05; // Wait up to 50ms
        boolean m_calculateCorrective = false;
        StatusSignalValue<Double> yawGetter;
        Consumer<Double> correctiveOut;

        public DriveStraightThread(StatusSignalValue<Double> yawGetter, Consumer<Double> correctiveOut) {
            this.yawGetter = yawGetter;
            this.correctiveOut = correctiveOut;
        }

        public void beginCorrective() {
            m_calculateCorrective = true;
        }

        public void endCorrective() {
            m_calculateCorrective = false;
        }

        @Override
        public void run() {
            double holdYaw = 0;
            while (true) {
                holdYaw = yawGetter.waitForUpdate(MAX_UPDATE_PERIOD).getValue();
                while (m_calculateCorrective) {
                    double err = holdYaw - yawGetter.waitForUpdate(MAX_UPDATE_PERIOD).getValue();
                    /* Simple P-loop, where 100degrees off corresponds to 100% output */
                    correctiveOut.accept(err * 0.01);
                }
            }
        }
    }

    @Override
    public void initialize()
    {
        /* On initialize, begin the corrective calculation */
        m_driveStraightThread.beginCorrective();
    }

    /* No need for an execute, as our thread will execute automatically */

    @Override
    public void end(boolean isInterrupted)
    {
        m_driveStraightThread.endCorrective();
    }

}
