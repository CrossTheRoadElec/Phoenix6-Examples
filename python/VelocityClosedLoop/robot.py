#!/usr/bin/env python3
"""
    This is a demo program for TalonFX Velocity PID usage in Phoenix 6
"""
import wpilib
from wpilib import NiDsXboxController, RobotController, simulation as sim
from wpimath import DCMotor, Models
from wpimath.units import radiansToRotations
from phoenix6 import CANBus, StatusCode, configs, controls, hardware, signals

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use TalonFX
    in Phoenix 6 python
    """

    def __init__(self):
        """Robot initialization function"""
        super().__init__()

        self.canbus = CANBus("canivore")

        # Keep a reference to all the motor controllers used
        self.talonfx = hardware.TalonFX(0, self.canbus)
        self.talonfx_follower = hardware.TalonFX(1, self.canbus)

        # Be able to switch which control request to use based on a button press
        # Start at velocity 0, use slot 0
        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)
        # Start at velocity 0, use slot 1
        self.velocity_torque = controls.VelocityTorqueCurrentFOC(0).with_slot(1)
        # Keep a neutral out so we can disable the motor
        self.brake = controls.NeutralOut()

        self.joystick = NiDsXboxController(0)

        cfg = configs.TalonFXConfiguration()
        
        # Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
        cfg.slot0.k_s = 0.1 # To account for friction, add 0.1 V of static feedforward
        cfg.slot0.k_v = 0.12 # Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        cfg.slot0.k_p = 0.11 # An error of 1 rotation per second results in 2V output
        cfg.slot0.k_i = 0 # No output for integrated error
        cfg.slot0.k_d = 0 # No output for error derivative
        # Peak output of 8 volts
        cfg.voltage.peak_forward_voltage = 8
        cfg.voltage.peak_reverse_voltage = -8

        # Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself
        cfg.slot1.k_s = 2.5 # To account for friction, add 2.5 A of static feedforward
        cfg.slot1.k_p = 5 # An error of 1 rotation per second results in 5 A output
        cfg.slot1.k_i = 0 # No output for integrated error
        cfg.slot1.k_d = 0 # No output for error derivative
        # Peak output of 40 A
        cfg.torque_current.peak_forward_torque_current = 40
        cfg.torque_current.peak_reverse_torque_current = -40

        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.talonfx.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.talonfx_follower.set_control(controls.Follower(self.talonfx.device_id, signals.MotorAlignmentValue.ALIGNED))

        # Create a DCMotorSim for physics sim
        gearbox = DCMotor.krakenX60FOC(1)
        self.motor_sim = sim.DCMotorSim(Models.singleJointedArmFromPhysicalConstants(gearbox, 0.01, 1.0), gearbox)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        joy_value = self.joystick.getLeftY()
        if abs(joy_value) < 0.1:
            joy_value = 0

        # Go for plus/minus 50 rotations per second
        desired_rotations_per_second = joy_value * 50

        if self.joystick.getLeftBumperButton():
            # Use velocity voltage
            self.talonfx.set_control(self.velocity_voltage.with_velocity(desired_rotations_per_second))
        elif self.joystick.getRightBumperButton():
            # Use velocity torque
            self.talonfx.set_control(self.velocity_torque.with_velocity(desired_rotations_per_second))
        else:
            # Disable the motor instead
            self.talonfx.set_control(self.brake)

    def simulationPeriodic(self):
        talon_sim = self.talonfx.sim_state

        talon_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.motor_sim.setInputVoltage(talon_sim.motor_voltage)
        self.motor_sim.update(0.020)
        talon_sim.set_raw_rotor_position(radiansToRotations(self.motor_sim.getAngularPosition()))
        talon_sim.set_rotor_velocity(radiansToRotations(self.motor_sim.getAngularVelocity()))
