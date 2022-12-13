package frc.robot.subsystems;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.Utils;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.Pigeon2Configurator;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.sim.Pigeon2SimState;
import com.ctre.phoenixpro.sim.TalonFXSimState;
import com.ctre.phoenixpro.spns.InvertedValue;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
    private final TalonFX m_leftLeader = new TalonFX(LEFT_LEADER_ID, CANBUS_NAME);
    private final TalonFX m_leftFollower = new TalonFX(LEFT_FOLLOWER_ID, CANBUS_NAME);
    private final TalonFX m_rightLeader = new TalonFX(RIGHT_LEADER_ID, CANBUS_NAME);
    private final TalonFX m_rightFollower = new TalonFX(RIGHT_FOLLOWER_ID, CANBUS_NAME);

    private final TalonFXSimState m_leftSimState = m_leftLeader.getSimState();
    private final TalonFXSimState m_rightSimState = m_rightLeader.getSimState();

    private final DutyCycleOut m_leftOut = new DutyCycleOut(0); // Initialize with 0% output
    private final DutyCycleOut m_rightOut = new DutyCycleOut(0); // Initialize with 0% output

    private final Pigeon2 m_pigeon2 = new Pigeon2(PIGEON2_ID, CANBUS_NAME);

    private final Pigeon2SimState m_pigeon2SimState = m_pigeon2.getSimState();

    /*
     * These numbers are an example AndyMark Drivetrain with some additional weight.
     * This is a fairly light robot.
     * Note you can utilize results from robot characterization instead of
     * theoretical numbers.
     * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-
     * characterization/introduction.html#introduction-to-robot-characterization
     */
    final double kGearRatio = 10.71;
    final double kWheelRadiusInches = 3;

    /* Simulation model of the drivetrain */
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2), // 2 CIMS on each side of the drivetrain.
            kGearRatio, // Standard AndyMark Gearing reduction.
            2.1, // MOI of 2.1 kg m^2 (from CAD model).
            26.5, // Mass of the robot is 26.5 kg.
            Units.inchesToMeters(kWheelRadiusInches), // Robot uses 3" radius (6" diameter) wheels.
            0.546, // Distance between wheels is _ meters.

            /*
             * The standard deviations for measurement noise:
             * x and y: 0.001 m
             * heading: 0.001 rad
             * l and r velocity: 0.1 m/s
             * l and r position: 0.005 m
             */
            /* Uncomment the following line to add measurement noise. */
            null // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    Field2d m_field = new Field2d();
    /*
     * Creating my odometry object. Here,
     * our starting pose is 5 meters along the long end of the field and in the
     * center of the field along the short end, facing forward.
     */
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_pigeon2.getRotation2d(),
            0, 0);

    public DriveSubsystem() {
        /* Initialize all the devices */
        initializeLeftDriveTalonFX(m_leftLeader.getConfigurator());
        initializeLeftDriveTalonFX(m_leftFollower.getConfigurator());
        initializeRightDriveTalonFX(m_rightLeader.getConfigurator());
        initializeRightDriveTalonFX(m_rightFollower.getConfigurator());
        initializePigeon2(m_pigeon2.getConfigurator());

        /* Set followers to follow leader */
        m_leftFollower.setControl(new Follower(m_leftLeader.getDeviceID(), false));
        m_rightFollower.setControl(new Follower(m_rightLeader.getDeviceID(), false));

        /* Make sure all critical signals are synchronized */
        /*
         * Setting all these signals to 100hz means they get sent at the same time if
         * they're all on a CANivore
         */
        m_pigeon2.getYaw().setUpdateFrequency(100);
        m_leftLeader.getPosition().setUpdateFrequency(100);
        m_rightLeader.getPosition().setUpdateFrequency(100);

        /*
         * Set the update frequency of the main requests to 0 so updates are sent
         * immediately in the arcadeDrive method
         */
        m_leftOut.UpdateFreqHz = 0;
        m_rightOut.UpdateFreqHz = 0;

        /* Currently in simulation, we do not support FOC, so disable it while simulating */
        if(Utils.isSimulation()) {
            m_leftOut.EnableFOC = false;
            m_rightOut.EnableFOC = false;
        }

        /* Publish field pose data to read back from */
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Drive the robot using an arcade drive format.
     * <p>
     * <b>This must be called periodically</b> or else the control frames will not
     * get sent
     * out, resulting in the TalonFXs disabling
     * 
     * @param fwd Forward/Reverse output
     * @param rot Left/Right output
     */
    public void arcadeDrive(double fwd, double rot) {
        m_leftOut.Output = fwd + rot;
        m_rightOut.Output = fwd - rot;

        m_leftLeader.setControl(m_leftOut);
        m_rightLeader.setControl(m_rightOut);
    }

    @Override
    public void periodic() {
        /*
         * This will get the simulated sensor readings that we set
         * in the previous article while in simulation, but will use
         * real values on the robot itself.
         */
        m_odometry.update(m_pigeon2.getRotation2d(),
                rotationsToMeters(m_leftLeader.getPosition().getValue()),
                rotationsToMeters(m_rightLeader.getPosition().getValue()));
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated Talon SRXs */
        m_leftSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        m_pigeon2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        /*
         * CTRE simulation is low-level, so SimCollection inputs
         * and outputs are not affected by SetInverted(). Only
         * the regular user-level API calls are affected.
         *
         * WPILib expects +V to be forward.
         * Positive motor output lead voltage is ccw. We observe
         * on our physical robot that this is reverse for the
         * right motor, so negate it.
         *
         * We are hard-coding the negation of the values instead of
         * using getInverted() so we can catch a possible bug in the
         * robot code where the wrong value is passed to setInverted().
         */
        m_driveSim.setInputs(m_leftSimState.getMotorVoltage(),
                -m_rightSimState.getMotorVoltage());

        /*
         * Advance the model by 20 ms. Note that if you are running this
         * subsystem in a separate thread or have changed the nominal
         * timestep of TimedRobot, this value needs to match it.
         */
        m_driveSim.update(0.02);

        /*
         * Update all of our sensors.
         *
         * Since WPILib's simulation class is assuming +V is forward,
         * but -V is forward for the right motor, we need to negate the
         * position reported by the simulation class. Basically, we
         * negated the input, so we need to negate the output.
         *
         * We also observe on our physical robot that a positive voltage
         * across the output leads results in a positive sensor velocity
         * for both the left and right motors, so we do not need to negate
         * the output any further.
         * If we had observed that a positive voltage results in a negative
         * sensor velocity, we would need to negate the output once more.
         */
        m_leftSimState.setRawRotorPosition(
                metersToRotations(
                        m_driveSim.getLeftPositionMeters()));
        m_leftSimState.setRotorVelocity(
                metersToRotations( // This is OK, since the time base is the same
                        m_driveSim.getLeftVelocityMetersPerSecond()));
        m_rightSimState.setRawRotorPosition(
                metersToRotations(
                        -m_driveSim.getRightPositionMeters()));
        m_rightSimState.setRotorVelocity(
                metersToRotations( // This is OK, since the time base is the same
                        -m_driveSim.getRightVelocityMetersPerSecond()));
        m_pigeon2SimState.setRawYaw(m_driveSim.getHeading().getDegrees());
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
     * Initialize a left drive TalonFX device from the configurator object
     * 
     * @param cfg Configurator of the TalonFX device
     */
    private void initializeLeftDriveTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        /*
         * User can change configs if they want, or leave this blank for factory-default
         */
        toApply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        cfg.apply(toApply);

        /* And initialize position to 0 */
        cfg.setRotorPosition(0);
    }

    /**
     * Initialize a right drive TalonFX device from the configurator object
     * 
     * @param cfg Configurator of the TalonFX device
     */
    private void initializeRightDriveTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        /*
         * User can change configs if they want, or leave this blank for factory-default
         */
        toApply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.apply(toApply);

        /* And initialize position to 0 */
        cfg.setRotorPosition(0);
    }

    /**
     * Initialize Pigeon2 device from the configurator object
     * 
     * @param cfg Configurator of the Pigeon2 device
     */
    private void initializePigeon2(Pigeon2Configurator cfg) {
        var toApply = new Pigeon2Configuration();

        /*
         * User can change configs if they want, or leave this blank for factory-default
         */

        cfg.apply(toApply);

        /* And initialize yaw to 0 */
        cfg.setYaw(48);
    }

    private double rotationsToMeters(double rotations) {
        /* Get circumference of wheel */
        double circumference = this.kWheelRadiusInches * 2 * Math.PI;
        /* Every rotation of the wheel travels this many inches */
        /* So now get the meters traveled per rotation */
        double metersPerWheelRotation = Units.inchesToMeters(circumference);
        /* Now apply gear ratio to input rotations */
        double gearedRotations = rotations / this.kGearRatio;
        /* And multiply geared rotations by meters per rotation */
        return gearedRotations * metersPerWheelRotation;
    }

    private double metersToRotations(double meters) {
        /* Get circumference of wheel */
        double circumference = this.kWheelRadiusInches * 2 * Math.PI;
        /* Every rotation of the wheel travels this many inches */
        /* So now get the meters traveled per rotation */
        double wheelRotationsPerMeter = 1.0 / Units.inchesToMeters(circumference);
        /* Now apply wheel rotations to inptu meters */
        double wheelRotations = wheelRotationsPerMeter * meters;
        /* And multiply by gear ratio to get rotor rotations */
        return wheelRotations * this.kGearRatio;
    }
}
