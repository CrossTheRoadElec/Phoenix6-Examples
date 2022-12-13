// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.StaticBrake;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TalonFX m_fx = new TalonFX(0);
  
  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, enable FOC, no feed forward, use slot 0 */
  PositionVoltage m_voltagePosition = new PositionVoltage(0, true, 0, 0);
  /* Start at position 0, no feed forward, use slot 1 */
  PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 1);
  /* Keep a brake request so we can disable the motor */
  StaticBrake m_brake = new StaticBrake();

  XboxController m_joystick = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 24; // An error of 0.5 rotations results in 12V output
    configs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    configs.Slot0.PeakOutput = 8; // Peak output of 8 volts
    
    configs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    configs.Slot1.PeakOutput = 130; // Peak output of 130 amps

    m_fx.getConfigurator().apply(configs);

    /* Make sure we start at 0 */
    m_fx.setRotorPosition(0);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double desiredRotations = m_joystick.getLeftY() * 10; // Go for plus/minus 10 rotations
    if (m_joystick.getLeftBumper())
    {
      /* Use voltage position */
      m_fx.setControl(m_voltagePosition.withPosition(desiredRotations));
    }
    else if (m_joystick.getRightBumper())
    {
      /* Use torque position */
      m_fx.setControl(m_torquePosition.withPosition(desiredRotations));
    }
    else
    {
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
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
