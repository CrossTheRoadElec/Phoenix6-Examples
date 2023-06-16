// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TalonFX m_fx = new TalonFX(1, "fred");
  CANcoder m_cc = new CANcoder(1, "fred");
  StatusSignal<Boolean> f_fusedSensorOutOfSync = m_fx.getFault_FusedSensorOutOfSync();
  StatusSignal<Boolean> sf_fusedSensorOutOfSync = m_fx.getStickyFault_FusedSensorOutOfSync();
  StatusSignal<Boolean> f_missingRemoteSensor = m_fx.getFault_MissingRemoteSensor();
  StatusSignal<Boolean> sf_missingRemoteSensor = m_fx.getStickyFault_MissingRemoteSensor();

  StatusSignal<Double> fx_pos = m_fx.getPosition();
  StatusSignal<Double> fx_vel = m_fx.getVelocity();
  StatusSignal<Double> cc_pos = m_cc.getPosition();
  StatusSignal<Double> cc_vel = m_cc.getVelocity();

  DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);

  XboxController m_joystick = new XboxController(0);

  int printCount = 0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = 0.4;
    m_cc.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    fx_cfg.Feedback.RotorToSensorRatio = 12.8;

    m_fx.getConfigurator().apply(fx_cfg);
  }

  @Override
  public void robotPeriodic() {
    if (printCount++ > 10) {
      printCount = 0;
      // If any faults happen, print them out. Sticky faults will always be present if live-fault occurs
      f_fusedSensorOutOfSync.refresh();
      sf_fusedSensorOutOfSync.refresh();
      f_missingRemoteSensor.refresh();
      sf_missingRemoteSensor.refresh();
      boolean anyFault = sf_fusedSensorOutOfSync.getValue() || sf_missingRemoteSensor.getValue();
      if(anyFault) {
        System.out.println("A fault has occurred:");
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if(f_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync live-faulted");
        } else if (sf_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync sticky-faulted");
        }
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if(f_missingRemoteSensor.getValue()) {
          System.out.println("Missing remote sensor live-faulted");
        } else if (sf_missingRemoteSensor.getValue()) {
          System.out.println("Missing remote sensor sticky-faulted");
        }
      }

      if(m_joystick.getAButton()) {
        /* Clear sticky faults */
        m_fx.clearStickyFaults();
      }

      /* Print out current position and velocity */
      fx_pos.refresh(); fx_vel.refresh();
      cc_pos.refresh(); cc_vel.refresh();
      System.out.println("FX Position: " + fx_pos + " FX Vel: " + fx_vel);
      System.out.println("CC Position: " + cc_pos + " CC Vel: " + cc_vel);
      System.out.println("");
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    m_fx.setControl(m_dutyCycleControl.withOutput(m_joystick.getLeftY()));
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
