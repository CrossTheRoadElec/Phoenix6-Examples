#!/usr/bin/env python3
"""
    This is a demo program for arcade drive in Python with Phoenix 6
"""
import math
import wpilib
from wpilib.simulation import DifferentialDrivetrainSim
from wpimath import DCMotor, DifferentialDriveKinematics, DifferentialDriveOdometry, units

from phoenix6 import CANBus, configs, controls, hardware, signals, sim


class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows to do simple arcade drive in robotpy
    with Phoenix 6
    """

    def __init__(self):
        """Robot initialization function"""
        super().__init__()

        # Keep a reference to all the motor controllers used
        self.canivore = CANBus("canivore")
        self.front_left_motor = hardware.TalonFX(0, self.canivore)
        self.rear_left_motor = hardware.TalonFX(1, self.canivore)
        self.front_right_motor = hardware.TalonFX(2, self.canivore)
        self.rear_right_motor = hardware.TalonFX(3, self.canivore)
        self.pigeon = hardware.Pigeon2(0, self.canivore)

        cfg = configs.TalonFXConfiguration()
        cfg.motor_output.inverted = configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.front_left_motor.configurator.apply(cfg)

        cfg.motor_output.inverted = configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE
        self.front_right_motor.configurator.apply(cfg)

        # Configure the rear motors to follow the front motors
        follow_left_request = controls.Follower(0, signals.MotorAlignmentValue.ALIGNED)
        self.rear_left_motor.set_control(follow_left_request)

        follow_right_request = controls.Follower(2, signals.MotorAlignmentValue.ALIGNED)
        self.rear_right_motor.set_control(follow_right_request)

        # Keep a reference to the DutyCycleOut control request to update periodically
        self.left_out = controls.DutyCycleOut(0)
        self.right_out = controls.DutyCycleOut(0)

        # Keep a reference to an Xbox Controller for teleop control
        self.joy = wpilib.NiDsXboxController(0)

        # Simulation
        self.wheel_radius = units.inchesToMeters(3)
        self.gear_ratio = 10.71
        track_width = 0.546

        self.drivetrain = DifferentialDrivetrainSim(
            DCMotor.krakenX60FOC(2),            # 2 Kraken X60 on each side of the drivetrain
            self.gear_ratio,                    # drivetrain gear ratio
            2.1,                                # MOI of 2.1 kg m^2 (from CAD model)
            26.5,                               # Mass of the robot is 26.5 kg
            self.wheel_radius,                  # Robot uses 3" radius (6" diameter) wheels
            track_width,                        # Distance between wheels is _ meters.
        )

        self.kinematics = DifferentialDriveKinematics(track_width)
        self.odometry = DifferentialDriveOdometry(self.pigeon.getRotation2d(), 0, 0)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

    def robotPeriodic(self):
        self.odometry.update(
            self.pigeon.getRotation2d(),
            self.rotations_to_meters(self.front_left_motor.get_position().value),
            self.rotations_to_meters(self.front_right_motor.get_position().value),
        )
        self.field.setRobotPose(self.odometry.getPose())

    def teleopPeriodic(self):
        """Runs the motors with arcade drive"""
        # Get throttle and wheel values for arcade drive
        throttle = self.joy.getLeftY() * -1
        wheel = self.joy.getRightX() * 1

        # And set the DutyCycleOut to the motor controllers
        self.front_left_motor.set_control(self.left_out.with_output(throttle + wheel))
        self.front_right_motor.set_control(self.right_out.with_output(throttle - wheel))

    def simulationInit(self):
        # Set the orientation of the simulated devices relative to the robot chassis.
        # WPILib expects +V to be forward. Specify orientations to match that behavior.

        # left devices are CCW+
        self.front_left_motor.sim_state.orientation = sim.ChassisReference.COUNTER_CLOCKWISE_POSITIVE
        # right devices are CW+
        self.front_right_motor.sim_state.orientation = sim.ChassisReference.CLOCKWISE_POSITIVE

    def simulationPeriodic(self):
        left_talon_sim = self.front_left_motor.sim_state
        right_talon_sim = self.front_right_motor.sim_state
        pigeon_sim = self.pigeon.sim_state

        battery_v = wpilib.RobotController.getBatteryVoltage()
        left_talon_sim.set_supply_voltage(battery_v)
        right_talon_sim.set_supply_voltage(battery_v)

        # CTRE simulation is low-level, so SimState inputs
        # and outputs are not affected by user-level inversion.
        # However, inputs and outputs *are* affected by the mechanical
        # orientation of the device relative to the robot chassis,
        # as specified by the `orientation` field.
        #
        # WPILib expects +V to be forward. We have already configured
        # our orientations to match this behavior.
        self.drivetrain.setInputs(left_talon_sim.motor_voltage, right_talon_sim.motor_voltage)

        # Advance the model by 0.020 seconds
        self.drivetrain.update(0.020)

        left_talon_sim.set_raw_rotor_position(self.meters_to_rotations(self.drivetrain.getLeftPosition()))
        left_talon_sim.set_rotor_velocity(self.meters_to_rotations(self.drivetrain.getLeftVelocity()))
        right_talon_sim.set_raw_rotor_position(self.meters_to_rotations(self.drivetrain.getRightPosition()))
        right_talon_sim.set_rotor_velocity(self.meters_to_rotations(self.drivetrain.getRightVelocity()))
        pigeon_sim.set_raw_yaw(self.drivetrain.getHeading().degrees())

    def meters_to_rotations(self, dist: float) -> float:
        circumference = self.wheel_radius * 2.0 * math.tau
        rotations_per_meter = self.gear_ratio / circumference
        return dist * rotations_per_meter

    def rotations_to_meters(self, rot: float) -> float:
        circumference = self.wheel_radius * 2.0 * math.tau
        meters_per_rotation = circumference / self.gear_ratio
        return rot * meters_per_rotation
