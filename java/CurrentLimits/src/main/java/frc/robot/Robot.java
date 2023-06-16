// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TalonFX m_fx = new TalonFX(0, "Fred");
  DutyCycleOut m_output = new DutyCycleOut(0);

  CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

  XboxController m_joystick = new XboxController(0);
  

  int printCount = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Configure the Talon to use a supply limit of 1 amps IF we exceed 4 amps for over 1 second */
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    m_currentLimits.SupplyCurrentLimit = 1; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = 4; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = 1.0; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    m_currentLimits.StatorCurrentLimit = 20; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

    toConfigure.CurrentLimits = m_currentLimits;

    m_fx.getConfigurator().apply(toConfigure);
  }

  @Override
  public void robotPeriodic() {
    /* Tie output of joystick to output of motor for current limit testing */
    m_fx.setControl(m_output.withOutput(m_joystick.getLeftY()));

    if(m_joystick.getAButtonPressed()) {
      /* Toggle the supply limit enable */
      m_currentLimits.SupplyCurrentLimitEnable ^= true;
      System.out.println("Setting supply limit to " + m_currentLimits.SupplyCurrentLimitEnable);
      m_fx.getConfigurator().apply(m_currentLimits);
    }
    if(m_joystick.getBButtonPressed()) {
      /* Toggle the stator limit enable */
      m_currentLimits.StatorCurrentLimitEnable ^= true;
      System.out.println("Setting stator limit to " + m_currentLimits.StatorCurrentLimitEnable);
      m_fx.getConfigurator().apply(m_currentLimits);
    }

    if(printCount++ > 20) {
      printCount= 0;
      System.out.println("Supply current: " + m_fx.getSupplyCurrent());
      System.out.println("Stator current: " + m_fx.getStatorCurrent());
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

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
