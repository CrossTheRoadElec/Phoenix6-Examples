#!/usr/bin/env python3
"""
    This is a demo program for StatusSignal usage in Phoenix 6
"""
import wpilib
from wpilib import XboxController, RobotController, Timer, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import radians_to_rotations
from phoenix6 import BaseStatusSignal, CANBus, SignalLogger, controls, hardware


class MyRobot(wpilib.TimedRobot):
    """
    Example program that provides basic usage on StatusSignals
    in Phoenix 6 python
    """

    def __init__(self):
        """Robot initialization function"""
        super().__init__()

        # Keep a reference to all the motor controllers used
        self.motor = hardware.TalonFX(1, CANBus("canivore"))
        self.request = controls.DutyCycleOut(0)

        self.pos = self.motor.get_position()
        self.vel = self.motor.get_velocity()

        self.timer = Timer()
        self.timer.start()

        self.joystick = XboxController(0)

        self.motor.set_position(6)

        # Create a DCMotorSim for physics sim
        gearbox = DCMotor.kraken_x60_foc(1)
        self.motor_sim = sim.DCMotorSim(Models.single_jointed_arm_from_physical_constants(gearbox, 0.01, 1.0), gearbox)

    def robot_periodic(self) -> None:
        # Drive the motor so we have a changing position/velocity
        self.motor.set_control(self.request.with_output(self.joystick.get_left_y()))

    def teleop_init(self) -> None:
        """Start signal logger for logging purposes"""
        SignalLogger.start()

    def teleop_periodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.has_elapsed(0.1):
            self.timer.reset()
            BaseStatusSignal.refresh_all(self.pos, self.vel)

            pos_timestamp = self.pos.all_timestamps.get_device_timestamp().time
            print(f"Position is {self.pos} and velocity is {self.vel} at timestamp {pos_timestamp}")

            latency_compensated_pos = BaseStatusSignal.get_latency_compensated_value(
                self.pos, self.vel
            )
            print(f"Latency compensated position is {latency_compensated_pos}")

    def simulation_periodic(self):
        talon_sim = self.motor.sim_state

        talon_sim.set_supply_voltage(RobotController.get_battery_voltage())
        self.motor_sim.set_input_voltage(talon_sim.motor_voltage)
        self.motor_sim.update(0.020)
        talon_sim.set_raw_rotor_position(radians_to_rotations(self.motor_sim.get_angular_position()))
        talon_sim.set_rotor_velocity(radians_to_rotations(self.motor_sim.get_angular_velocity()))
