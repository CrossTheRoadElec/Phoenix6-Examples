// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.framework.TimedRobot;

import first.robot.PWMTalonFXS.MotorArrangement;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // Select Talon FXS on PWM port 0
  private PWMTalonFXS m_motor = new PWMTalonFXS(0);

  // Use the first xbox controller connected to the Driver Station
  private NiDsXboxController m_joy = new NiDsXboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Configure the connected motor for Talon FXS
    m_motor.setMotorArrangement(MotorArrangement.Minion_JST);

    // Set neutral mode to brake
    m_motor.setNeutralMode(true);
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
    // Command motor output
    m_motor.setThrottle(-m_joy.getLeftY());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void utilityInit() {}

  @Override
  public void utilityPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
