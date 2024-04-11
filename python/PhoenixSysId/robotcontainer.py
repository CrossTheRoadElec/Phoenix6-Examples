from commands2 import Command, cmd
from commands2.button import CommandXboxController
from commands2.sysid import SysIdRoutine

from phoenix6 import SignalLogger

from subsystems.flywheel import FlywheelMechanism

class RobotContainer:
    def __init__(self) -> None:
        self.joystick = CommandXboxController(0)
        self.mechanism = FlywheelMechanism()

        self.configureBindings()

    def configureBindings(self) -> None:
        """Use this method to define bindings between conditions and commands. These are useful for
        automating robot behaviors based on button and sensor input.

        Should be called during :meth:`.Robot.robotInit`.

        Event binding methods are available on the :class:`.Trigger` class.
        """

        # Default command is duty cycle control with the left up/down stick
        self.mechanism.setDefaultCommand(self.mechanism.joystick_drive_command(self.joystick.getLeftY))

        # Manually start logging with left bumper before running any tests,
        # and stop logging with right bumper after we're done with ALL tests.
        # This isn't necessary but is convenient to reduce the size of the hoot file.
        self.joystick.leftBumper().onTrue(cmd.runOnce(SignalLogger.start))
        self.joystick.rightBumper().onTrue(cmd.runOnce(SignalLogger.stop))
        
        # Joystick Y = quasistatic forward
        # Joystick A = quasistatic reverse
        # Joystick B = dynamic forward
        # Joystick X = dynamic reverse
        self.joystick.y().whileTrue(self.mechanism.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        self.joystick.a().whileTrue(self.mechanism.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
        self.joystick.b().whileTrue(self.mechanism.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.joystick.x().whileTrue(self.mechanism.sys_id_dynamic(SysIdRoutine.Direction.kReverse))

    def getAutonomousCommand(self) -> Command:
        """Use this to define the command that runs during autonomous.

        Scheduled during :meth:`.Robot.autonomousInit`.
        """

        return cmd.print_("No autonomous command configured")
