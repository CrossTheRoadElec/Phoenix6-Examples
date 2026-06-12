#!/usr/bin/env python3
"""
    This is a demo program for CANrange usage in Phoenix 6
"""
import wpilib
from wpilib import RobotController, Timer, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import inchesToMeters
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
        # If CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal.
        cfg.proximity_params.proximity_threshold = 0.1
        # Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.
        cfg.to_f_params.update_mode = signals.UpdateModeValue.SHORT_RANGE100_HZ

        self.canrange.configurator.apply(cfg)

        self.timer = Timer()
        self.timer.start()
        self.controller = wpilib.NiDsXboxController(0)

        # Create a DCMotorSim for physics sim
        gearbox = DCMotor.krakenX60FOC(1)
        self.motor_sim = sim.DCMotorSim(Models.singleJointedArmFromPhysicalConstants(gearbox, 0.01, 1.0), gearbox)

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        if self.timer.hasElapsed(0.1):
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

    def simulationPeriodic(self):
        canrange_sim = self.canrange.sim_state

        canrange_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.motor_sim.setInputVoltage(self.controller.getLeftY() * 12)
        self.motor_sim.update(0.020)
        canrange_sim.set_distance(self.motor_sim.getAngularPosition() * inchesToMeters(3))
