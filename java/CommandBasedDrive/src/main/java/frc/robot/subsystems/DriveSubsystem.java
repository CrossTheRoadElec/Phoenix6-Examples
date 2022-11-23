package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.Pigeon2Configurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
    private final TalonFX m_leftLeader = new TalonFX(LEFT_LEADER_ID, CANBUS_NAME);
    private final TalonFX m_leftFollower = new TalonFX(LEFT_FOLLOWER_ID, CANBUS_NAME);
    private final TalonFX m_rightLeader = new TalonFX(RIGHT_LEADER_ID, CANBUS_NAME);
    private final TalonFX m_rightFollower = new TalonFX(RIGHT_FOLLOWER_ID, CANBUS_NAME);

    private final DutyCycleOut m_leftOut = new DutyCycleOut(0); // Initialize with 0% output
    private final DutyCycleOut m_rightOut = new DutyCycleOut(0); // Initialize with 0% output

    private final Pigeon2 m_pigeon2 = new Pigeon2(PIGEON2_ID, CANBUS_NAME);

    public DriveSubsystem() {
        /* Initialize all the devices */
        initializeTalonFX(m_leftLeader.getConfigurator());
        initializeTalonFX(m_leftFollower.getConfigurator());
        initializeTalonFX(m_rightLeader.getConfigurator());
        initializeTalonFX(m_rightFollower.getConfigurator());
        initializePigeon2(m_pigeon2.getConfigurator());

        /* Set followers to follow leader */
        m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
        m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

        /* Make sure all critical signals are synchronized */
        /* Setting all these signals to 100hz means they get sent at the same time if they're all on a CANivore */
        m_pigeon2.getYaw().setUpdateFrequency(100);
        m_leftLeader.getPosition().setUpdateFrequency(100);
        m_rightLeader.getPosition().setUpdateFrequency(100);

        /* Set the update frequency of the main requests to 0 so updates are sent immediately in the arcadeDrive method */
        m_leftOut.UpdateFreqHz = 0;
        m_rightOut.UpdateFreqHz = 0;
    }

    /**
     * Drive the robot using an arcade drive format.
     * <p>
     * <b>This must be called periodically</b> or else the control frames will not get sent
     * out, resulting in the TalonFXs disabling
     * 
     * @param fwd Forward/Reverse output
     * @param rot Left/Right output
     */
    public void arcadeDrive(double fwd, double rot) {
        m_leftOut.output = fwd + rot;
        m_rightOut.output = fwd - rot;

        m_leftLeader.setControl(m_leftOut);
        m_rightLeader.setControl(m_rightOut);
    }

    public StatusSignalValue<Double> getYaw() {
        return m_pigeon2.getYaw();
    }
    public StatusSignalValue<Double> getLeftPos() {
        return m_leftLeader.getPosition();
    }
    public StatusSignalValue<Double> getRightPos() {
        return m_rightLeader.getPosition();
    }

    /**
     * Initialize TalonFX device from the configurator object
     * @param cfg Configurator of the TalonFX device
     */
    private void initializeTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        /* User can change configs if they want, or leave this blank for factory-default */

        cfg.apply(toApply);

        /* And initialize position to 0 */
        cfg.setRotorPosition(0);
    }

    /**
     * Initialize Pigeon2 device from the configurator object
     * @param cfg Configurator of the Pigeon2 device
     */
    private void initializePigeon2(Pigeon2Configurator cfg) {
        var toApply = new Pigeon2Configuration();

        /* User can change configs if they want, or leave this blank for factory-default */

        cfg.apply(toApply);

        /* And initialize yaw to 0 */
        cfg.setYaw(0);
    }
}
