// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.Pigeon2;
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
  private final String CANBUS_NAME = "";
  private CANcoder m_cc = new CANcoder(0, CANBUS_NAME);
  private TalonFX m_fx = new TalonFX(0, CANBUS_NAME);
  private Pigeon2 m_p2 = new Pigeon2(0, CANBUS_NAME);
  private int m_printCount = 0;

  private DutyCycleOut m_dutycycle = new DutyCycleOut(0);

  XboxController m_joystick = new XboxController(0);

  StatusSignalValue<Double> m_ccpos = m_cc.getPosition();
  StatusSignalValue<Double> m_fxpos = m_fx.getPosition();
  StatusSignalValue<Double> m_p2yaw = m_p2.getYaw();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
  }

  @Override
  public void robotPeriodic() {
    /* Perform basic latency compensation based on latency and current derivative */
    /* First refresh the signal */
    m_ccpos.refresh();
    m_fxpos.refresh();
    m_p2yaw.refresh();

    /* Then get the rates of each of the signal */
    double ccVel = m_cc.getVelocity().getValue();
    double fxVel = m_fx.getVelocity().getValue();
    /**
     * Pigeon2 can only perform this latency compensation if the Z axis is straight up, since the
     * angular velocity Z value comes from the pre-mount orientation gyroscope.
     * For more information on what signals have what algorithms applied to them,
     * see section 1.6 of the Pigeon 2's User's Guide
     * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf 
     */
    double p2Rate = m_p2.getAngularVelocityZ().getValue();

    /* Multiply the latency (in seconds) by the rates (in seconds) to get the amount to offset by */
    /* This automatically uses the best timestamp, CANivore will perform better than RIO since its timestamp */
    /* is more accurate */
    double ccPosOffset = ccVel * m_ccpos.getTimestamp().getLatency();
    double fxPosOffset = fxVel * m_fxpos.getTimestamp().getLatency();
    double p2YawOffset = p2Rate * m_p2yaw.getTimestamp().getLatency();

    /* And add it to the current signal to get the latency-compensated signal */
    double ccCompensatedPos = m_ccpos.getValue() + ccPosOffset;
    double fxCompensatedPos = m_fxpos.getValue() + fxPosOffset;
    double p2CompensatedYaw = m_p2yaw.getValue() + p2YawOffset;

    /* Print out both values so it shows how they perform */
    if(m_printCount++ > 10 && m_joystick.getAButton()) {
      m_printCount = 0;
      System.out.printf("CANcoder: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f%n", m_ccpos.getValue(), ccCompensatedPos, ccPosOffset);
      System.out.printf("Talon FX: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f%n", m_fxpos.getValue(), fxCompensatedPos, fxPosOffset);
      System.out.printf("Pigeon2 : Yaw: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f%n", m_p2yaw.getValue(), p2CompensatedYaw, p2YawOffset);
      System.out.println();
    }
    m_fx.setControl(m_dutycycle.withOutput(m_joystick.getLeftY()));

    if(m_joystick.getLeftBumperPressed()) {
      /* Speed up the signals to reduce the latency */
      m_fxpos.setUpdateFrequency(1000); // Make it 1ms for this example
      m_ccpos.setUpdateFrequency(1000); // Make it 1ms for this example
      m_p2yaw.setUpdateFrequency(1000); // Make it 1ms for this example
    }
    if(m_joystick.getRightBumperPressed()) {
      /* Slow down the signals to increase the latency */
      m_fxpos.setUpdateFrequency(10); // Make it 100ms for this example
      m_ccpos.setUpdateFrequency(10); // Make it 100ms for this example
      m_p2yaw.setUpdateFrequency(10); // Make it 100ms for this example
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
