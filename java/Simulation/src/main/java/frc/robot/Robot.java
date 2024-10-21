// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final XboxController joystick = new XboxController(0);

    private final TalonFX leftFX = new TalonFX(1);
    private final TalonFX rightFX = new TalonFX(2);
    private final CANcoder leftSensor = new CANcoder(1);
    private final CANcoder rightSensor = new CANcoder(2);
    private final Pigeon2 imu = new Pigeon2(0);

    private final TalonFXSimState leftSim = leftFX.getSimState();
    private final TalonFXSimState rightSim = rightFX.getSimState();
    private final CANcoderSimState leftSensSim = leftSensor.getSimState();
    private final CANcoderSimState rightSensSim = rightSensor.getSimState();
    private final Pigeon2SimState imuSim = imu.getSimState();

    private final DifferentialDrive drivetrain = new DifferentialDrive(leftFX, rightFX);

    /*
     * These numbers are an example AndyMark Drivetrain with some additional weight.
     * This is a fairly light robot.
     * Note you can utilize results from robot characterization instead of
     * theoretical numbers.
     * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-
     * characterization/introduction.html#introduction-to-robot-characterization
     */
    private final double kGearRatio = 10.71;
    private final Distance kWheelRadius = Inches.of(3);

    /* Simulation model of the drivetrain */
    private final DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500Foc(2), // 2 CIMS on each side of the drivetrain.
        kGearRatio, // Standard AndyMark Gearing reduction.
        2.1, // MOI of 2.1 kg m^2 (from CAD model).
        26.5, // Mass of the robot is 26.5 kg.
        kWheelRadius.in(Inches), // Robot uses 3" radius (6" diameter) wheels.
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

    private final Field2d m_field = new Field2d();
    /*
     * Creating my odometry object.
     */
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
        imu.getRotation2d(),
        0, 0
    );

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        StatusCode returnCode;

        TalonFXConfiguration fxCfg = new TalonFXConfiguration();
        fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        for (int i = 0; i < 5; ++i) {
            returnCode = leftFX.getConfigurator().apply(fxCfg);
            if (returnCode.isOK()) break;
        }

        fxCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        for (int i = 0; i < 5; ++i) {
            returnCode = rightFX.getConfigurator().apply(fxCfg);
            if (returnCode.isOK()) break;
        };

        CANcoderConfiguration ccCfg = new CANcoderConfiguration();
        ccCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        leftSensor.getConfigurator().apply(ccCfg);
        ccCfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        rightSensor.getConfigurator().apply(ccCfg);

        Pigeon2Configuration imuCfg = new Pigeon2Configuration();
        imu.getConfigurator().apply(imuCfg);

        /* Make sure all critical signals are synchronized */
        /*
         * Setting all these signals to 100hz means they get sent at the same time if
         * they're all on a CANivore
         */
        BaseStatusSignal.setUpdateFrequencyForAll(100,
            leftFX.getPosition(),
            rightFX.getPosition(),
            imu.getYaw());

        /* Publish field pose data to read back from */
        SmartDashboard.putData("Field", m_field);
    }

    private int printCount = 0;

    @Override
    public void robotPeriodic() {
        /*
         * This will get the simulated sensor readings that we set
         * in the previous article while in simulation, but will use
         * real values on the robot itself.
         */
        m_odometry.update(imu.getRotation2d(),
                rotationsToMeters(leftSensor.getPosition().getValue()).in(Meters),
                rotationsToMeters(rightSensor.getPosition().getValue()).in(Meters));
        m_field.setRobotPose(m_odometry.getPoseMeters());

        if (++printCount >= 50) {
            printCount = 0;
            System.out.println("Left FX: " + leftFX.getPosition());
            System.out.println("Right FX: " + rightFX.getPosition());
            System.out.println("Left CANcoder: " + leftSensor.getPosition());
            System.out.println("Right CANcoder: " + rightSensor.getPosition());
            System.out.println("Left Forward limit: " + leftFX.getForwardLimit());
            System.out.println("Left Reverse limit: " + leftFX.getReverseLimit());
            System.out.println("Right Forward limit: " + rightFX.getForwardLimit());
            System.out.println("Right Reverse limit: " + rightFX.getReverseLimit());
            System.out.println("Pigeon2: " + imu.getYaw());
            System.out.println();
        }
    }

    @Override
    public void simulationInit() {
        /*
         * Set the orientation of the simulated devices relative to the robot chassis.
         * WPILib expects +V to be forward. Specify orientations to match that behavior.
         */
        /* left devices are CCW+ */
        leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
        leftSensSim.Orientation = ChassisReference.CounterClockwise_Positive;
        /* right devices are CW+ */
        rightSim.Orientation = ChassisReference.Clockwise_Positive;
        rightSensSim.Orientation = ChassisReference.Clockwise_Positive;
    }

    @Override
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated devices */
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        leftSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        /*
         * CTRE simulation is low-level, so SimState inputs
         * and outputs are not affected by user-level inversion.
         * However, inputs and outputs *are* affected by the mechanical
         * orientation of the device relative to the robot chassis,
         * as specified by the `orientation` field.
         *
         * WPILib expects +V to be forward. We have already configured
         * our orientations to match this behavior.
         */
        m_driveSim.setInputs(leftSim.getMotorVoltage(),
                rightSim.getMotorVoltage());

        /*
         * Advance the model by 20 ms. Note that if you are running this
         * subsystem in a separate thread or have changed the nominal
         * timestep of TimedRobot, this value needs to match it.
         */
        m_driveSim.update(0.02);

        /* Update all of our sensors. */
        final var leftPos = metersToRotations(
                Meters.of(m_driveSim.getLeftPositionMeters()));
        // This is OK, since the time base is the same
        final var leftVel = metersToRotationsVel(
                MetersPerSecond.of(m_driveSim.getLeftVelocityMetersPerSecond()));
        final var rightPos = metersToRotations(
                Meters.of(m_driveSim.getRightPositionMeters()));
        // This is OK, since the time base is the same
        final var rightVel = metersToRotationsVel(
                MetersPerSecond.of(m_driveSim.getRightVelocityMetersPerSecond()));
        leftSensSim.setRawPosition(leftPos);
        leftSensSim.setVelocity(leftVel);
        rightSensSim.setRawPosition(rightPos);
        rightSensSim.setVelocity(rightVel);
        leftSim.setRawRotorPosition(leftPos.times(this.kGearRatio));
        leftSim.setRotorVelocity(leftVel.times(this.kGearRatio));
        rightSim.setRawRotorPosition(rightPos.times(this.kGearRatio));
        rightSim.setRotorVelocity(rightVel.times(this.kGearRatio));
        imuSim.setRawYaw(m_driveSim.getHeading().getDegrees());


        /*
         * If a bumper is pressed, trigger the forward limit switch to test it, 
         * if a trigger is pressed, trigger the reverse limit switch 
         */
        leftSim.setForwardLimit(joystick.getLeftBumperButton());
        leftSim.setReverseLimit(joystick.getLeftTriggerAxis() > 0.5);
        rightSim.setForwardLimit(joystick.getRightBumperButton());
        rightSim.setReverseLimit(joystick.getRightTriggerAxis() > 0.5);
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        drivetrain.curvatureDrive(-joystick.getLeftY(), -joystick.getRightX(), joystick.getRightStickButton());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    private Distance rotationsToMeters(Angle rotations) {
        /* Apply gear ratio to input rotations */
        var gearedRadians = rotations.in(Radians) / this.kGearRatio;
        /* Then multiply the wheel radius by radians of rotation to get distance */
        return this.kWheelRadius.times(gearedRadians);
    }

    private Angle metersToRotations(Distance meters) {
        /* Divide the distance by the wheel radius to get radians */
        var wheelRadians = meters.in(Meters) / this.kWheelRadius.in(Meters);
        /* Then multiply by gear ratio to get rotor rotations */
        return Radians.of(wheelRadians * this.kGearRatio);
    }

    private LinearVelocity rotationsToMetersVel(AngularVelocity rotations) {
        /* Apply gear ratio to input rotations */
        var gearedRotations = rotations.divide(this.kGearRatio);
        /* Then multiply the wheel radius by radians of rotation to get distance */
        return this.kWheelRadius.per(Second).times(gearedRotations.in(RadiansPerSecond));
    }

    private AngularVelocity metersToRotationsVel(LinearVelocity meters) {
        /* Divide the distance by the wheel radius to get radians */
        var wheelRadians = meters.in(MetersPerSecond) / this.kWheelRadius.in(Meters);
        /* Then multiply by gear ratio to get rotor rotations */
        return RadiansPerSecond.of(wheelRadians * this.kGearRatio);
    }
}
