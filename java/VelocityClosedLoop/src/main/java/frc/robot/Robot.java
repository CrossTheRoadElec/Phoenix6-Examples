// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String canbusName = "canivore";
  private final TalonFX m_fx = new TalonFX(0, canbusName);
  private final TalonFX m_fllr = new TalonFX(1, canbusName);

  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, use slot 0 */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private final XboxController m_joystick = new XboxController(0);

  private final Mechanisms m_mechanisms = new Mechanisms();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    m_fllr.setControl(new Follower(m_fx.getDeviceID(), false));
  }

  @Override
  public void robotPeriodic() {
    m_mechanisms.update(m_fx.getPosition(), m_fx.getVelocity());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double joyValue = m_joystick.getLeftY();
    if (Math.abs(joyValue) < 0.1) joyValue = 0;

    double desiredRotationsPerSecond = joyValue * 50; // Go for plus/minus 50 rotations per second

    if (m_joystick.getLeftBumper()) {
      /* Use voltage velocity */
      m_fx.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
    } else if (m_joystick.getRightBumper()) {
      /* Use torque velocity */
      m_fx.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
    } else {
      /* Disable the motor instead */
      m_fx.setControl(m_brake);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
