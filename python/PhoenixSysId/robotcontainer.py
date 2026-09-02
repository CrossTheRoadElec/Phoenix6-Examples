from commands2 import Command, cmd
from commands2.button import CommandXboxController
from commands2.sysid import SysIdRoutine

from phoenix6 import SignalLogger

from subsystems.flywheel import FlywheelMechanism

class RobotContainer:
    def __init__(self) -> None:
        self.joystick = CommandXboxController(0)
        self.mechanism = FlywheelMechanism()

        self.configure_bindings()

    def configure_bindings(self) -> None:
        """Use this method to define bindings between conditions and commands. These are useful for
        automating robot behaviors based on button and sensor input.

        Should be called during :meth:`.Robot.__init__`.

        Event binding methods are available on the :class:`.Trigger` class.
        """

        # Default command is duty cycle control with the left up/down stick
        self.mechanism.set_default_command(self.mechanism.joystick_drive_command(self.joystick.get_left_y))

        # Manually start logging with left bumper before running any tests,
        # and stop logging with right bumper after we're done with ALL tests.
        # This isn't necessary but is convenient to reduce the size of the hoot file.
        self.joystick.left_bumper().on_true(cmd.run_once(SignalLogger.start))
        self.joystick.right_bumper().on_true(cmd.run_once(SignalLogger.stop))
        
        # Joystick Y = quasistatic forward
        # Joystick A = quasistatic reverse
        # Joystick B = dynamic forward
        # Joystick X = dynamic reverse
        self.joystick.y().while_true(self.mechanism.sys_id_quasistatic(SysIdRoutine.Direction.FORWARD))
        self.joystick.a().while_true(self.mechanism.sys_id_quasistatic(SysIdRoutine.Direction.REVERSE))
        self.joystick.b().while_true(self.mechanism.sys_id_dynamic(SysIdRoutine.Direction.FORWARD))
        self.joystick.x().while_true(self.mechanism.sys_id_dynamic(SysIdRoutine.Direction.REVERSE))

    def get_autonomous_command(self) -> Command:
        """Use this to define the command that runs during autonomous.

        Scheduled during :meth:`Robot.autonomous_init`.
        """

        return cmd.print_("No autonomous command configured")
