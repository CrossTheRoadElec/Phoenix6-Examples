#!/usr/bin/env python3
"""
    This is a demo program for CANdi usage in Phoenix 6
"""
import wpilib
from wpilib import RobotController, Timer, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import radians_to_rotations
from phoenix6 import CANBus, configs, hardware, signals

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use CANdi
    in Phoenix 6 python
    """

    def __init__(self):
        """Robot initialization function"""
        super().__init__()

        # Keep a reference to all the devices used
        self.candi = hardware.CANdi(1, CANBus("canivore"))

        # Configure CANdi
        cfg = configs.CANdiConfiguration()

        # Pulse-width sensor will drive low. Default of FloatDetect will typically work on most sensors.
        cfg.digital_inputs.s1_float_state = signals.S1FloatStateValue.PULL_HIGH
        # This example specifically assumes a hardware limit switch will close S2 to Ground. Default of CloseWhenNotFloating will also work.
        cfg.digital_inputs.s2_close_state = signals.S2CloseStateValue.CLOSE_WHEN_LOW

        # Invert the PWM1 position.
        cfg.pwm1.sensor_direction = True
        # If the PWM 1 position on boot is above 0.75 rotations, treat it as x - 1 rotations.
        # As an example, if the position is 0.87, it will boot to 0.87 - 1 = -0.13 rotations.
        cfg.pwm1.absolute_sensor_discontinuity_point = 0.75

        self.candi.configurator.apply(cfg)

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
            # get_pwm1_position automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            pos = self.candi.get_pwm1_position()
            print(f"Position is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

            # Get the S2 State StatusSignal without refreshing
            s2_state = self.candi.get_s2_state(False)
            # This time wait for the signal to reduce latency
            s2_state.wait_for_update(0.1)
            print(f"S2 State is {s2_state} with {s2_state.timestamp.get_latency()} seconds of latency")

            print("")

    def simulation_periodic(self):
        candi_sim = self.candi.sim_state

        candi_sim.set_supply_voltage(RobotController.get_battery_voltage())
        self.motor_sim.set_input_voltage(self.controller.get_left_y() * 12)
        self.motor_sim.update(0.020)
        candi_sim.set_pwm1_position(radians_to_rotations(self.motor_sim.get_angular_position()))
        candi_sim.set_pwm1_velocity(radians_to_rotations(self.motor_sim.get_angular_velocity()))
