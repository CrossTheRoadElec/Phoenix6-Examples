#!/usr/bin/env python3
"""
    This is a demo program for TalonFX usage in Phoenix 6
"""
import wpilib
from wpilib import NiDsXboxController, RobotController, Timer, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import radiansToRotations
from phoenix6 import CANBus, controls, hardware

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
        self.control = controls.DutyCycleOut(0)

        self.timer = Timer()
        self.timer.start()

        self.joystick = NiDsXboxController(0)

        # Create a DCMotorSim for physics sim
        gearbox = DCMotor.krakenX60FOC(1)
        self.motor_sim = sim.DCMotorSim(Models.singleJointedArmFromPhysicalConstants(gearbox, 0.01, 1.0), gearbox)

    def teleopPeriodic(self):
        """Every 100ms, print the status of the StatusSignal"""

        self.talonfx.set_control(self.control.with_output(self.joystick.getLeftY()))

        if self.timer.hasElapsed(0.1):
            self.timer.reset()
            # get_position automatically calls refresh(), no need to manually refresh.
            #
            # StatusSignals also implement the str dunder to provide a useful print of the signal
            pos = self.talonfx.get_position()
            print(f"Positions is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

            # Get the velocity StatusSignal without refreshing
            vel = self.talonfx.get_velocity(False)
            # This time wait for the signal to reduce latency
            vel.wait_for_update(0.1)
            print(f"Velocity is {vel} with {vel.timestamp.get_latency()} seconds of latency")

            print("")

    def simulationPeriodic(self):
        talon_sim = self.talonfx.sim_state

        talon_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.motor_sim.setInputVoltage(talon_sim.motor_voltage)
        self.motor_sim.update(0.020)
        talon_sim.set_raw_rotor_position(radiansToRotations(self.motor_sim.getAngularPosition()))
        talon_sim.set_rotor_velocity(radiansToRotations(self.motor_sim.getAngularVelocity()))
