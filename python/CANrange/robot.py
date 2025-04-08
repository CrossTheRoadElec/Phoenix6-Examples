#!/usr/bin/env python3
"""
    This is a demo program for CANrange usage in Phoenix 6
"""
import wpilib
from wpilib import Timer
from phoenix6 import configs, hardware, signals

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use CANrange
    in Phoenix 6 python
    """

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the devices used
        self.canrange = hardware.CANrange(1, "canivore")

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
        self.controller = wpilib.XboxController(0)

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

if __name__ == "__main__":
    wpilib.run(MyRobot)
