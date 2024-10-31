import wpilib.simulation as sim
from wpilib import RobotController, DriverStation

from wpilib.simulation import DifferentialDrivetrainSim
from wpimath.kinematics import DifferentialDriveKinematics, DifferentialDriveWheelSpeeds
from wpimath.system.plant import DCMotor
from wpimath.units import inchesToMeters

from pyfrc.physics.core import PhysicsInterface

from phoenix6 import unmanaged, sim

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

        self.wheel_radius = inchesToMeters(3)
        self.gear_ratio = 10.71
        track_width = 0.546

        self.drivetrain = DifferentialDrivetrainSim(
            DCMotor.krakenX60FOC(2),            # 2 Kraken X60 on each side of the drivetrain
            self.gear_ratio,                    # drivetrain gear ratio
            2.1,                                # MOI of 2.1 kg m^2 (from CAD model)
            26.5,                               # Mass of the robot is 26.5 kg
            self.wheel_radius,                  # Robot uses 3" radius (6" diameter) wheels
            track_width,                        # Distance between wheels is _ meters.
        )

        self.kinematics = DifferentialDriveKinematics(track_width)

        # Set the orientation of the simulated devices relative to the robot chassis.
        # WPILib expects +V to be forward. Specify orientations to match that behavior.

        # left devices are CCW+
        self.left_talon_sim.orientation = sim.ChassisReference.CounterClockwise_Positive
        # right devices are CW+
        self.right_talon_sim.orientation = sim.ChassisReference.Clockwise_Positive

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        battery_v = RobotController.getBatteryVoltage()
        self.left_talon_sim.set_supply_voltage(battery_v)
        self.right_talon_sim.set_supply_voltage(battery_v)

        # CTRE simulation is low-level, so SimState inputs
        # and outputs are not affected by user-level inversion.
        # However, inputs and outputs *are* affected by the mechanical
        # orientation of the device relative to the robot chassis,
        # as specified by the `orientation` field.
        #
        # WPILib expects +V to be forward. We have already configured
        # our orientations to match this behavior.
        self.drivetrain.setInputs(self.left_talon_sim.motor_voltage, self.right_talon_sim.motor_voltage)

        # Advance the model by tm_diff
        self.drivetrain.update(tm_diff)

        self.left_talon_sim.set_raw_rotor_position(self.meters_to_rotations(self.drivetrain.getLeftPosition()))
        self.left_talon_sim.set_rotor_velocity(self.meters_to_rotations(self.drivetrain.getLeftVelocity()))
        self.right_talon_sim.set_raw_rotor_position(self.meters_to_rotations(self.drivetrain.getRightPosition()))
        self.right_talon_sim.set_rotor_velocity(self.meters_to_rotations(self.drivetrain.getRightVelocity()))

        self.physics_controller.drive(
            self.kinematics.toChassisSpeeds(
                DifferentialDriveWheelSpeeds(
                    self.drivetrain.getLeftVelocity(),
                    self.drivetrain.getRightVelocity()
                )
            ),
            tm_diff,
        )

    def meters_to_rotations(self, dist: float) -> float:
        circumference = self.wheel_radius * 2.0 * math.pi
        rotations_per_meter = self.gear_ratio / circumference
        return dist * rotations_per_meter

