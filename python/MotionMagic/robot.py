#!/usr/bin/env python3
"""
    This is a demo program for TalonFX Motion Magic usage in Phoenix 6
"""
import wpilib
from wpilib import XboxController, RobotController, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import radians_to_rotations
from phoenix6 import CANBus, StatusCode, configs, controls, hardware

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def __init__(self):
        """Robot initialization function"""
        super().__init__()

        # Keep a reference to all the motor controllers used
        self.talonfx = hardware.TalonFX(1, CANBus("canivore"))
        self.motion_magic = controls.MotionMagicVoltage(0)

        self.joystick = XboxController(0)

        cfg = configs.TalonFXConfiguration()

        # Configure gear ratio
        fdb = cfg.feedback
        fdb.sensor_to_mechanism_ratio = 12.8 # 12.8 rotor rotations per mechanism rotation

        # Configure Motion Magic
        mm = cfg.motion_magic
        mm.motion_magic_cruise_velocity = 5 # 5 (mechanism) rotations per second cruise
        mm.motion_magic_acceleration = 10 # Take approximately 0.5 seconds to reach max vel
        # Take apprximately 0.1 seconds to reach max accel
        mm.motion_magic_jerk = 100

        slot0 = cfg.slot0
        slot0.k_s = 0.25 # Add 0.25 V output to overcome static friction
        slot0.k_v = 0.12 # A velocity target of 1 rps results in 0.12 V output
        slot0.k_a = 0.01 # An acceleration of 1 rps/s requires 0.01 V output
        slot0.k_p = 60 # A position error of 0.2 rotations results in 12 V output
        slot0.k_i = 0 # No output for integrated error
        slot0.k_d = 0.5 # A velocity error of 1 rps results in 0.5 V output

        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 2):
            status = self.talonfx.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        # Create a DCMotorSim for physics sim
        gearbox = DCMotor.kraken_x60_foc(1)
        self.motor_sim = sim.DCMotorSim(Models.single_jointed_arm_from_physical_constants(gearbox, 0.01, 1.0), gearbox)

    def teleop_init(self):
        pass

    def teleop_periodic(self):
        left_y = self.joystick.get_left_y()
        self.talonfx.set_control(self.motion_magic.with_position(left_y * 10).with_slot(0))

        if (self.joystick.get_b_button()):
            self.talonfx.set_position(1)

    def simulation_periodic(self):
        talon_sim = self.talonfx.sim_state

        talon_sim.set_supply_voltage(RobotController.get_battery_voltage())
        self.motor_sim.set_input_voltage(talon_sim.motor_voltage)
        self.motor_sim.update(0.020)
        talon_sim.set_raw_rotor_position(radians_to_rotations(self.motor_sim.get_angular_position()))
        talon_sim.set_rotor_velocity(radians_to_rotations(self.motor_sim.get_angular_velocity()))
