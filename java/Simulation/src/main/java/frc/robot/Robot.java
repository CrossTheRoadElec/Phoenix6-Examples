// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.ctre.phoenixpro.sim.Pigeon2SimState;
import com.ctre.phoenixpro.sim.TalonFXSimState;
import com.ctre.phoenixpro.spns.CANcoder_SensorDirectionValue;
import com.ctre.phoenixpro.spns.InvertedValue;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
    XboxController joystick = new XboxController(0);

    TalonFX leftFx = new TalonFX(1);
    TalonFXSimState leftSim = leftFx.getSimState();
    TalonFX rightFx = new TalonFX(2);
    TalonFXSimState rightSim = rightFx.getSimState();
    CANcoder leftSensor = new CANcoder(1);
    CANcoderSimState leftSensSim = leftSensor.getSimState();
    CANcoder rightSensor = new CANcoder(2);
    CANcoderSimState rightSensSim = rightSensor.getSimState();
    Pigeon2 imu = new Pigeon2(0);
    Pigeon2SimState imuSim = imu.getSimState();

    DifferentialDrive drivetrain = new DifferentialDrive(leftFx, rightFx);

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
     * Creating my odometry object.
     */
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(imu.getRotation2d(),
            rotationsToMeters(leftSensor.getPosition().getValue()), rotationsToMeters(rightSensor.getPosition().getValue()));

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        TalonFXConfiguration fxCfg = new TalonFXConfiguration();
        fxCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        StatusCode returnCode;
        do
        {
            returnCode = leftFx.getConfigurator().apply(fxCfg);
        } while(!returnCode.isOK());

        fxCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        do
        {
            returnCode = rightFx.getConfigurator().apply(fxCfg);
        } while(!returnCode.isOK());

        CANcoderConfiguration ccCfg = new CANcoderConfiguration();
        ccCfg.MagnetSensor.SensorDirection = CANcoder_SensorDirectionValue.Clockwise_Positive;
        leftSensor.getConfigurator().apply(ccCfg);
        ccCfg.MagnetSensor.SensorDirection = CANcoder_SensorDirectionValue.CounterClockwise_Positive;
        rightSensor.getConfigurator().apply(ccCfg);

        Pigeon2Configuration imuCfg = new Pigeon2Configuration();
        imu.getConfigurator().apply(imuCfg);

        /* Make sure all critical signals are synchronized */
        /*
         * Setting all these signals to 100hz means they get sent at the same time if
         * they're all on a CANivore
         */
        imu.getYaw().setUpdateFrequency(100);
        leftFx.getPosition().setUpdateFrequency(100);
        rightFx.getPosition().setUpdateFrequency(100);

        /* Publish field pose data to read back from */
        SmartDashboard.putData("Field", m_field);
    }

    int printCount = 0;
    @Override
    public void robotPeriodic() {
        /*
         * This will get the simulated sensor readings that we set
         * in the previous article while in simulation, but will use
         * real values on the robot itself.
         */
        m_odometry.update(imu.getRotation2d(),
                rotationsToMeters(leftSensor.getPosition().getValue()),
                rotationsToMeters(rightSensor.getPosition().getValue()));
        m_field.setRobotPose(m_odometry.getPoseMeters());

        if(printCount++ > 50)
        {
            printCount = 0;
            System.out.println("Left FX: " + leftFx.getPosition());
            System.out.println("Right FX: " + rightFx.getPosition());
            System.out.println("Left CANcoder: " + leftSensor.getPosition());
            System.out.println("Right CANcoder: " + rightSensor.getPosition());
            System.out.println("Left Forward limit: " + leftFx.getForwardLimit());
            System.out.println("Left Reverse limit: " + leftFx.getReverseLimit());
            System.out.println("Right Forward limit: " + rightFx.getForwardLimit());
            System.out.println("Right Reverse limit: " + rightFx.getReverseLimit());
            System.out.println("Pigeon2: " + imu.getYaw());
            System.out.println("");
        }
    }

    @Override
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated Talon SRXs */
        leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        leftSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rightSensSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        imuSim.setSupplyVoltage(RobotController.getBatteryVoltage());

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
        m_driveSim.setInputs(leftSim.getMotorVoltage(),
                -rightSim.getMotorVoltage());

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
        double leftPos = metersToRotations(
                m_driveSim.getLeftPositionMeters());
        double leftVel = metersToRotations( // This is OK, since the time base is the same
                m_driveSim.getLeftVelocityMetersPerSecond());
        double rightPos = metersToRotations(
                -m_driveSim.getRightPositionMeters());
        double rightVel = metersToRotations( // This is OK, since the time base is the same
                -m_driveSim.getRightVelocityMetersPerSecond());
        leftSensSim.setRawPosition(leftPos);
        leftSensSim.setVelocity(leftVel);
        rightSensSim.setRawPosition(rightPos);
        rightSensSim.setVelocity(rightVel);
        leftSim.setRawRotorPosition(leftPos * this.kGearRatio);
        leftSim.setRotorVelocity(leftVel * this.kGearRatio);
        rightSim.setRawRotorPosition(rightPos * this.kGearRatio);
        rightSim.setRotorVelocity(rightVel * this.kGearRatio);
        imuSim.setRawYaw(m_driveSim.getHeading().getDegrees());


        /**
         *  If a bumper is pressed, trigger the forward limit switch to test it, 
         * if a trigger is pressed, trigger the reverse limit switch 
         */
        leftSim.setForwardLimit(joystick.getLeftBumper());
        leftSim.setReverseLimit(joystick.getLeftTriggerAxis() > 0.5);
        rightSim.setForwardLimit(joystick.getRightBumper());
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
        drivetrain.curvatureDrive(-joystick.getLeftY(), joystick.getRightX(), joystick.getRightStickButton());
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

    @Override
    public void simulationInit() {
    }

    private double rotationsToMeters(double rotations) {
        /* Get circumference of wheel */
        double circumference = this.kWheelRadiusInches * 2 * Math.PI;
        /* Every rotation of the wheel travels this many inches */
        /* So now get the meters traveled per rotation */
        double metersPerWheelRotation = Units.inchesToMeters(circumference);
        /* And multiply geared rotations by meters per rotation */
        return rotations * metersPerWheelRotation;
    }

    private double metersToRotations(double meters) {
        /* Get circumference of wheel */
        double circumference = this.kWheelRadiusInches * 2 * Math.PI;
        /* Every rotation of the wheel travels this many inches */
        /* So now get the meters traveled per rotation */
        double wheelRotationsPerMeter = 1.0 / Units.inchesToMeters(circumference);
        /* Now apply wheel rotations to inptu meters */
        return wheelRotationsPerMeter * meters;
    }
}
