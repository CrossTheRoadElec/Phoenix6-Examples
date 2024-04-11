from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from phoenix6 import configs, controls, hardware, BaseStatusSignal, SignalLogger

from constants import Constants

from typing import Callable

class FlywheelMechanism(Subsystem):
    def __init__(self) -> None:
        self.motor_to_test = hardware.TalonFX(Constants.kTalonFxId, Constants.kCANbus)
        self.joystick_control = controls.DutyCycleOut(0)
        self.sys_id_control = controls.VoltageOut(0)

        self.sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 to prevent brownout
                stepVoltage = 4.0,
                # Log state with Phoenix SignalLogger class
                recordState = lambda state: SignalLogger.write_string("state", SysIdRoutineLog.stateEnumToString(state))
            ),
            SysIdRoutine.Mechanism(
                lambda volts: self.motor_to_test.set_control(self.sys_id_control.with_output(volts)),
                lambda log: None,
                self
            )
        )

        self.setName("Flywheel")

        cfg = configs.TalonFXConfiguration()
        # Set any necessary configs in the Feedback group here
        self.motor_to_test.configurator.apply(cfg)

        # Speed up signals for better characterization data
        BaseStatusSignal.set_update_frequency_for_all(250,
            self.motor_to_test.get_position(),
            self.motor_to_test.get_velocity(),
            self.motor_to_test.get_motor_voltage())
        
        # Optimize out the other signals, since they're not useful for SysId
        self.motor_to_test.optimize_bus_utilization()

        # Start the signal logger
        SignalLogger.start()

    def joystick_drive_command(self, output: Callable[[], float]) -> Command:
        return self.run(lambda: self.motor_to_test.set_control(self.joystick_control.with_output(output())))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sys_id_routine.dynamic(direction)
