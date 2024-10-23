from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units, utils
from wpilib import Color, Color8Bit, Mechanism2d, MechanismLigament2d, SmartDashboard
from wpimath.geometry import Pose2d

class Telemetry:
    def __init__(self, max_speed: units.meters_per_second):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed
        SignalLogger.start()

        # What to publish over networktables for telemetry
        self._inst = NetworkTableInstance.getDefault()

        # Robot pose for field positioning
        self._table = self._inst.getTable("Pose")
        self._field_pub = self._table.getDoubleArrayTopic("robotPose").publish()
        self._field_type_pub = self._table.getStringTopic(".type").publish()

        # Robot speeds for general checking
        self._drive_stats = self._inst.getTable("Drive")
        self._velocity_x = self._drive_stats.getDoubleTopic("Velocity X").publish()
        self._velocity_y = self._drive_stats.getDoubleTopic("Velocity Y").publish()
        self._speed = self._drive_stats.getDoubleTopic("Speed").publish()
        self._odom_freq = self._drive_stats.getDoubleTopic("Odometry Frequency").publish()

        # Keep a reference of the last pose to calculate the speeds
        self._last_pose = Pose2d()
        self._last_time = utils.get_current_time_seconds()

        # Mechanisms to represent the swerve module states
        self._module_mechanisms: list[Mechanism2d] = [
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
            Mechanism2d(1, 1),
        ]
        # A direction and length changing ligament for speed representation
        self._module_speeds: list[MechanismLigament2d] = [
            self._module_mechanisms[0].getRoot("RootSpeed", 0.5, 0.5).appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[1].getRoot("RootSpeed", 0.5, 0.5).appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[2].getRoot("RootSpeed", 0.5, 0.5).appendLigament("Speed", 0.5, 0),
            self._module_mechanisms[3].getRoot("RootSpeed", 0.5, 0.5).appendLigament("Speed", 0.5, 0),
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions: list[MechanismLigament2d] = [
            self._module_mechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
            self._module_mechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                .appendLigament("Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)),
        ]

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
        """
        # Telemeterize the pose
        pose_array = [state.pose.x, state.pose.y, state.pose.rotation().degrees()]

        self._field_type_pub.set("Field2d")
        self._field_pub.set(pose_array)

        # Telemeterize the robot's general speeds
        current_time = utils.get_current_time_seconds()
        diff_time = current_time - self._last_time
        self._last_time = current_time

        distance_diff = (state.pose - self._last_pose).translation()
        self._last_pose = state.pose

        velocities = distance_diff / diff_time

        self._speed.set(velocities.norm())
        self._velocity_x.set(velocities.x)
        self._velocity_y.set(velocities.y)
        self._odom_freq.set(1.0 / state.odometry_period)

        # Telemeterize the module's states
        for i, module_state in enumerate(state.module_states):
            self._module_speeds[i].setAngle(module_state.angle.degrees())
            self._module_directions[i].setAngle(module_state.angle.degrees())
            self._module_speeds[i].setLength(module_state.speed / (2 * self._max_speed))

            SmartDashboard.putData(f"Module {i}", self._module_mechanisms[i])

        SignalLogger.write_double_array("odometry", pose_array)
        SignalLogger.write_double("odom period", state.odometry_period, "seconds")
