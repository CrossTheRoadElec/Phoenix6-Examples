#!/usr/bin/env python3
"""
    This is a demo program for CANcoder usage in Phoenix 6
"""
import wpilib
from wpilib import RobotController, Timer, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import radians_to_rotations
from phoenix6 import CANBus, hardware

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use CANcoder
    in Phoenix 6 python
    """

    def __init__(self):
        """Robot initialization function"""
        super().__init__()

        # Keep a reference to all the motor controllers used
        self.cancoder = hardware.CANcoder(1, CANBus("canivore"))

        self.timer = Timer()
        self.timer.start()
        self.controller = wpilib.XboxController(0)

        gearbox = DCMotor.kraken_x60_foc(1)
        self.motor_sim = sim.DCMotorSim(Models.single_jointed_arm_from_physical_constants(gearbox, 0.01, 1.0), gearbox)

    def teleop_periodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.has_elapsed(0.1):
            self.timer.reset()
            # get_position automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            pos = self.cancoder.get_position()
            print(f"Positions is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

            # Get the velocity StatusSignal without refreshing
            vel = self.cancoder.get_velocity(False)
            # This time wait for the signal to reduce latency
            vel.wait_for_update(0.1)
            print(f"Velocity is {vel} with {vel.timestamp.get_latency()} seconds of latency")

            print("")

    def simulation_periodic(self):
        cancoder_sim = self.cancoder.sim_state

        cancoder_sim.set_supply_voltage(RobotController.get_battery_voltage())
        self.motor_sim.set_input_voltage(self.controller.get_left_y() * 12)
        self.motor_sim.update(0.020)
        cancoder_sim.set_raw_position(radians_to_rotations(self.motor_sim.get_angular_position()))
        cancoder_sim.set_velocity(radians_to_rotations(self.motor_sim.get_angular_velocity()))
