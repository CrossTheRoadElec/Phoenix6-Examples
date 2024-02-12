import wpilib.simulation as sim
from wpilib import RobotController, DriverStation

from wpimath.system.plant import DCMotor
from wpimath.units import radiansToRotations

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

from phoenix6 import unmanaged

import math

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        # Keep a reference to the motor sim state so we can update it
        self.left_talon_sim = robot.front_left_motor.sim_state
        self.right_talon_sim = robot.front_right_motor.sim_state

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        self.wheel_radius = 3
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_FALCON_500,    # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            self.wheel_radius * 2 * units.inch,  # wheel diameter
        )

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

        battery_v = RobotController.getBatteryVoltage()
        self.left_talon_sim.set_supply_voltage(battery_v)
        self.right_talon_sim.set_supply_voltage(battery_v)

        transform = self.drivetrain.calculate(self.left_talon_sim.motor_voltage / battery_v, self.right_talon_sim.motor_voltage / battery_v, tm_diff)
        self.physics_controller.move_robot(transform)

        self.left_talon_sim.set_raw_rotor_position(self.feet_to_rotations(self.drivetrain.l_position))
        self.left_talon_sim.set_rotor_velocity(self.feet_to_rotations(self.drivetrain.l_velocity))
        self.right_talon_sim.set_raw_rotor_position(self.feet_to_rotations(self.drivetrain.r_position))
        self.right_talon_sim.set_rotor_velocity(self.feet_to_rotations(self.drivetrain.r_velocity))

    def feet_to_rotations(self, dist: float) -> float:
        circumference = self.wheel_radius * 2.0 * math.pi
        rotations_per_foot = 1.0 / (circumference / 12.0)
        return rotations_per_foot * dist

