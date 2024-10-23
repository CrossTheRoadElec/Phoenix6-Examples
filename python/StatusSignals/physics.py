import wpilib.simulation as sim
from wpilib import RobotController, DriverStation

from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations

from pyfrc.physics.core import PhysicsInterface
from phoenix6 import unmanaged

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Create a DCMotorSim for physics sim
        gearbox = DCMotor.krakenX60FOC(1)
        self.motor_sim = sim.DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, 0.01, 1.0), gearbox)
        # Keep a reference to the motor sim state so we can update it
        self.talon_sim = robot.motor.sim_state

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # If the driver station is enabled, then feed enable for phoenix devices
        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)

        self.talon_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.motor_sim.setInputVoltage(self.talon_sim.motor_voltage)
        self.motor_sim.update(tm_diff)
        self.talon_sim.set_raw_rotor_position(radiansToRotations(self.motor_sim.getAngularPosition()))
        self.talon_sim.set_rotor_velocity(radiansToRotations(self.motor_sim.getAngularVelocity()))
