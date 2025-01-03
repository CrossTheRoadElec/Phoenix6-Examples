// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final CANBus kCANBus = new CANBus("canivore");

  private final CANcoder m_cc = new CANcoder(0, kCANBus);
  private final TalonFX m_fx = new TalonFX(0, kCANBus);
  private final Pigeon2 m_p2 = new Pigeon2(0, kCANBus);

  private int m_printCount = 0;

  private final DutyCycleOut m_dutycycle = new DutyCycleOut(0);

  private final XboxController m_joystick = new XboxController(0);

  private final StatusSignal<Angle> m_ccpos = m_cc.getPosition(false);
  private final StatusSignal<Angle> m_fxpos = m_fx.getPosition(false);
  private final StatusSignal<Angle> m_p2yaw = m_p2.getYaw(false);
  private final StatusSignal<AngularVelocity> m_ccvel = m_cc.getVelocity(false);
  private final StatusSignal<AngularVelocity> m_fxvel = m_fx.getVelocity(false);
  private final StatusSignal<AngularVelocity> m_p2yawRate = m_p2.getAngularVelocityZWorld(false);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

  }

  @Override
  public void robotPeriodic() {
    /* Perform basic latency compensation based on latency and current derivative */
    /* First refresh the signals */
    BaseStatusSignal.refreshAll(m_fxpos, m_fxvel, m_ccpos, m_ccvel, m_p2yaw, m_p2yawRate);

    /* Use the helper function to apply latency compensation to the signals */
    /* Since these are already refreshed we don't need to inline the refresh call */
    var ccCompensatedPos = BaseStatusSignal.getLatencyCompensatedValue(m_ccpos, m_ccvel);
    var fxCompensatedPos = BaseStatusSignal.getLatencyCompensatedValue(m_fxpos, m_fxvel);
    var p2CompensatedYaw = BaseStatusSignal.getLatencyCompensatedValue(m_p2yaw, m_p2yawRate);

    /* Print out both values so it shows how they perform */
    if (++m_printCount >= 10 && m_joystick.getAButton()) {
      m_printCount = 0;
      System.out.printf(
        "CANcoder: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f%n", 
        m_ccpos.getValue().in(Rotations),
        ccCompensatedPos.in(Rotations),
        ccCompensatedPos.minus(m_ccpos.getValue()).in(Rotations)
      );
      System.out.printf(
        "Talon FX: Pos: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f%n", 
        m_fxpos.getValue().in(Rotations), fxCompensatedPos.in(Rotations),
        fxCompensatedPos.minus(m_fxpos.getValue()).in(Rotations)
      );
      System.out.printf(
        "Pigeon2 : Yaw: %10.3f - Latency-Compensated: %10.3f - Difference: %6.5f%n", 
        m_p2yaw.getValue().in(Degrees),
        p2CompensatedYaw.in(Degrees),
        p2CompensatedYaw.minus(m_p2yaw.getValue()).in(Degrees)
      );
      System.out.println();
    }
    m_fx.setControl(m_dutycycle.withOutput(m_joystick.getLeftY()));

    if (m_joystick.getLeftBumperButtonPressed()) {
      /* Speed up the signals to reduce the latency */
      /* Make them 1000 Hz (1 ms) for this example */
      BaseStatusSignal.setUpdateFrequencyForAll(1000, m_fxpos, m_ccpos, m_p2yaw);
    }
    if (m_joystick.getRightBumperButtonPressed()) {
      /* Slow down the signals to increase the latency */
      /* Make them 10 Hz (100 ms) for this example */
      BaseStatusSignal.setUpdateFrequencyForAll(10, m_fxpos, m_ccpos, m_p2yaw);
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
