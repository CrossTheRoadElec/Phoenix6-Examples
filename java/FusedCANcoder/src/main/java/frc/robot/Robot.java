// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
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
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String canBusName = "canivore";
  private final TalonFX m_fx = new TalonFX(1, canBusName);
  private final CANcoder m_cc = new CANcoder(1, canBusName);

  private final StatusSignal<Boolean> f_fusedSensorOutOfSync = m_fx.getFault_FusedSensorOutOfSync();
  private final StatusSignal<Boolean> sf_fusedSensorOutOfSync = m_fx.getStickyFault_FusedSensorOutOfSync();
  private final StatusSignal<Boolean> f_remoteSensorInvalid = m_fx.getFault_RemoteSensorDataInvalid();
  private final StatusSignal<Boolean> sf_remoteSensorInvalid = m_fx.getStickyFault_RemoteSensorDataInvalid();

  private final StatusSignal<Double> fx_pos = m_fx.getPosition();
  private final StatusSignal<Double> fx_vel = m_fx.getVelocity();
  private final StatusSignal<Double> cc_pos = m_cc.getPosition();
  private final StatusSignal<Double> cc_vel = m_cc.getVelocity();
  private final StatusSignal<Double> fx_rotorPos = m_fx.getRotorPosition();

  private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);

  private final XboxController m_joystick = new XboxController(0);

  private int printCount = 0;

  private final Mechanisms m_mechanism = new Mechanisms();

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

      BaseStatusSignal.refreshAll(
        f_fusedSensorOutOfSync,
        sf_fusedSensorOutOfSync,
        f_remoteSensorInvalid,
        sf_remoteSensorInvalid,
        fx_pos, fx_vel,
        cc_pos, cc_vel);

      // If any faults happen, print them out. Sticky faults will always be present if live-fault occurs
      boolean anyFault = sf_fusedSensorOutOfSync.getValue() || sf_remoteSensorInvalid.getValue();
      if (anyFault) {
        System.out.println("A fault has occurred:");
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if (f_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync live-faulted");
        } else if (sf_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync sticky-faulted");
        }
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if (f_remoteSensorInvalid.getValue()) {
          System.out.println("Missing remote sensor live-faulted");
        } else if (sf_remoteSensorInvalid.getValue()) {
          System.out.println("Missing remote sensor sticky-faulted");
        }
      }

      if (m_joystick.getAButton()) {
        /* Clear sticky faults */
        m_fx.clearStickyFaults();
      }

      /* Print out current position and velocity */
      System.out.println("FX Position: " + fx_pos + " FX Vel: " + fx_vel);
      System.out.println("CC Position: " + cc_pos + " CC Vel: " + cc_vel);
      System.out.println("");
    }
    m_mechanism.update(fx_rotorPos, cc_pos, fx_pos);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double output = m_joystick.getLeftY();
    if (Math.abs(output) < 0.1) output = 0;
    m_fx.setControl(m_dutyCycleControl.withOutput(output));
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
    PhysicsSim.getInstance().addTalonFX(m_fx, m_cc, 12.8, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
