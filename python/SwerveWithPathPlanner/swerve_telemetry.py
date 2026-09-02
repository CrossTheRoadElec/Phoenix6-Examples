import telemetry
from phoenix6 import SignalLogger, swerve, units
from telemetry import TelemetryTable
from wpilib import Field2d, Mechanism2d, MechanismLigament2d
from wpiutil import Color, Color8Bit


class SwerveTelemetry:
    def __init__(self, max_speed: units.meters_per_second):
        """
        Constructs a swerve telemetry object.

        :param max_speed: Maximum speed of the robot
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed
        # optionally start the SignalLogger immediately
        # SignalLogger.start()

        # Table for swerve telemetry publishing
        self._swerve_telem: TelemetryTable = telemetry.get_table("Swerve")

        # Robot pose on a field
        self._field = Field2d()

        # Mechanisms to represent the swerve module states
        self._module_mechanisms: list[Mechanism2d] = [
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
        ]
        # A direction and length changing ligament for speed representation
        self._module_speeds: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .get_root("RootSpeed", 0.5, 0.5)
            .append_ligament("Speed", 0.5, 0),
            self._module_mechanisms[1]
            .get_root("RootSpeed", 0.5, 0.5)
            .append_ligament("Speed", 0.5, 0),
            self._module_mechanisms[2]
            .get_root("RootSpeed", 0.5, 0.5)
            .append_ligament("Speed", 0.5, 0),
            self._module_mechanisms[3]
            .get_root("RootSpeed", 0.5, 0.5)
            .append_ligament("Speed", 0.5, 0),
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions: list[MechanismLigament2d] = [
            self._module_mechanisms[0]
            .get_root("RootDirection", 0.5, 0.5)
            .append_ligament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
            self._module_mechanisms[1]
            .get_root("RootDirection", 0.5, 0.5)
            .append_ligament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
            self._module_mechanisms[2]
            .get_root("RootDirection", 0.5, 0.5)
            .append_ligament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
            self._module_mechanisms[3]
            .get_root("RootDirection", 0.5, 0.5)
            .append_ligament("Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)),
        ]

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accepts the swerve drive state and telemeterizes it to SignalLogger and Telemetry.
        """
        # Write the swerve drive state to the log file
        SignalLogger.write_struct("DriveState", state.struct, state, state.timestamp)

        # Also telemeterize the swerve drive state
        self._swerve_telem.log("DriveState", state)

        # Telemeterize the pose to a Field2d
        self._field.set_robot_pose(state.pose)
        self._swerve_telem.log("Pose", self._field)

        # Telemeterize each module state to a Mechanism2d
        for i, module_state in enumerate(state.module_velocities):
            self._module_directions[i].set_angle(module_state.angle.degrees())
            self._module_speeds[i].set_angle(module_state.angle.degrees())
            self._module_speeds[i].set_length(module_state.velocity / (2 * self._max_speed))

            self._swerve_telem.log("Module " + str(i), self._module_mechanisms[i])
