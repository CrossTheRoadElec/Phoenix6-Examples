#!/usr/bin/env python3
"""
    This is a demo program for CANrange usage in Phoenix 6
"""
import wpilib
from wpilib import RobotController, Timer, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import inches_to_meters
from phoenix6 import CANBus, configs, hardware, signals

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use CANrange
    in Phoenix 6 python
    """

    def __init__(self):
        """Robot initialization function"""
        super().__init__()

        # Keep a reference to all the devices used
        self.canrange = hardware.CANrange(1, CANBus("canivore"))

        # Configure CANrange
        cfg = configs.CANrangeConfiguration()

        # If CANrange has a signal strength of at least 2000, it is a valid measurement.
        cfg.proximity_params.min_signal_strength_for_valid_measurement = 2000
        # If CANrange detects an object within 0.1 meters, it will trigger the "is_detected" signal.
        cfg.proximity_params.proximity_threshold = 0.1
        # Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.
        cfg.to_f_params.update_mode = signals.UpdateModeValue.SHORT_RANGE100_HZ

        self.canrange.configurator.apply(cfg)

        self.timer = Timer()
        self.timer.start()
        self.controller = wpilib.XboxController(0)

        # Create a DCMotorSim for physics sim
        gearbox = DCMotor.kraken_x60_foc(1)
        self.motor_sim = sim.DCMotorSim(Models.single_jointed_arm_from_physical_constants(gearbox, 0.01, 1.0), gearbox)

    def teleop_periodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.has_elapsed(0.1):
            self.timer.reset()
            # get_distance automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            distance = self.canrange.get_distance()
            signal_strength = self.canrange.get_signal_strength()
            print(f"Distance is {str(distance)} with a signal strength of {str(signal_strength)} and {distance.timestamp.get_latency()} seconds of latency")

            # Get the is_detected StatusSignal without refreshing
            is_detected = self.canrange.get_is_detected(False)
            # This time wait for the signal to reduce latency
            is_detected.wait_for_update(0.1)
            print(f"Is Detected is {is_detected} with {is_detected.timestamp.get_latency()} seconds of latency")

            print("")

    def simulation_periodic(self):
        canrange_sim = self.canrange.sim_state

        canrange_sim.set_supply_voltage(RobotController.get_battery_voltage())
        self.motor_sim.set_input_voltage(self.controller.get_left_y() * 12)
        self.motor_sim.update(0.020)
        canrange_sim.set_distance(self.motor_sim.get_angular_position() * inches_to_meters(3))
