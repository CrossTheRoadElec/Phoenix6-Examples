package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class FlywheelMechanism extends SubsystemBase {
    private final TalonFX m_motorToTest = new TalonFX(Constants.kTalonFxId, Constants.kCANbus);
    private final DutyCycleOut m_joystickControl = new DutyCycleOut(0);
    private final VoltageOut m_sysIdControl = new VoltageOut(0);

    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage volts)-> m_motorToTest.setControl(m_sysIdControl.withOutput(volts.in(Volts))),
                null,
                this));

    public FlywheelMechanism() {
        setName("Flywheel");

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        // Set any necessary configs in the Feedback group here
        m_motorToTest.getConfigurator().apply(cfg);

        /* Speed up signals for better characterization data */
        BaseStatusSignal.setUpdateFrequencyForAll(250,
            m_motorToTest.getPosition(),
            m_motorToTest.getVelocity(),
            m_motorToTest.getMotorVoltage());

        /* Optimize out the other signals, since they're not useful for SysId */
        m_motorToTest.optimizeBusUtilization();

        /* Start the signal logger */
        SignalLogger.start();
    }

    public Command joystickDriveCommand(DoubleSupplier output) {
        return run(()->m_motorToTest.setControl(m_joystickControl.withOutput(output.getAsDouble())));
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
