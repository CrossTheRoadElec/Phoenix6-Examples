// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final TalonFXS m_fxs = new TalonFXS(0, "canivore");

  private final StatusSignal<Angle> m_quadPosSignal;
  private final StatusSignal<AngularVelocity> m_quadVelSignal;
  private final StatusSignal<Angle> m_pwmPosSignal;
  private final StatusSignal<AngularVelocity> m_pwmVelSignal;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Configure the Talon FXS motor arrangement to Minion. This is
    // required for any motor output and for rotor position/velocity getters.
    TalonFXSConfiguration toConfigure = new TalonFXSConfiguration();
    toConfigure.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

    // Use the connected quadrature encoder for any closed loop control. The
    // regular position and velocity getters will return position/velocity
    // as retrieved from the connected quadrature encoder.
    toConfigure.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Quadrature;

    // Store the result of the config apply to variable, the user can check this
    // to make sure the config apply actually succeeded.
    m_fxs.getConfigurator().apply(toConfigure);
    
    // Cache our *raw* quad and pwm signals
    // In most cases, you shouldn't use this over setting the feedback source.
    m_quadPosSignal = m_fxs.getRawQuadraturePosition();
    m_quadVelSignal = m_fxs.getRawQuadratureVelocity();
    m_pwmPosSignal = m_fxs.getRawPulseWidthPosition();
    m_pwmVelSignal = m_fxs.getRawPulseWidthVelocity();

    // The quadrature and pwm signals are disabled by default, the user must
    // set their update frequency to enable them.
    StatusSignal.setUpdateFrequencyForAll(
      Hertz.of(50), 
      m_quadPosSignal,
      m_quadVelSignal,
      m_pwmPosSignal,
      m_pwmVelSignal);
  }

  @Override
  public void robotPeriodic() {
    // Retrieves the regular position and velocity from the motor controller.
    // The source of this value is determined by the feedback source configured
    // in the ExternalFeedbackSensorSource config. On-board closed loop
    // control uses these signals for it's logic.
    var position = m_fxs.getPosition().getValueAsDouble();
    var velocity = m_fxs.getVelocity().getValueAsDouble();

    // Retrieve the position/velocity of the rotor. This is always from the motor encoder
    // when in brushed, and will return 0 for brushed motors. These signals
    // are not used for closed loop control.
    var rotorPosition = m_fxs.getRotorPosition().getValueAsDouble();
    var rotorVelocity = m_fxs.getRotorVelocity().getValueAsDouble();

    // Retrieve the *raw* position/velocity from the connected quadrature encoder.
    // This does not respect any configs except QuadratureEdgesPerRotation and users
    // should not use these signals except in advanced circumstances.
    var quadPosition = m_quadPosSignal.refresh().getValueAsDouble();
    var quadVelocity = m_quadVelSignal.refresh().getValueAsDouble();

    // Retrieve the *raw* position/velocity from the connected pulse-width encoder.
    // This does not respect any configs and users should not use these signals
    // except in advanced circumstances.
    var pwmPosition = m_pwmPosSignal.refresh().getValueAsDouble();
    var pwmVelocity = m_pwmVelSignal.refresh().getValueAsDouble();

    // Publish these signals to SmartDashboard
    SmartDashboard.putNumber("Position", position);
    SmartDashboard.putNumber("Velocity", velocity);
    SmartDashboard.putNumber("Rotor Position", rotorPosition);
    SmartDashboard.putNumber("Rotor Velocity", rotorVelocity);
    SmartDashboard.putNumber("Quadrature Position", quadPosition);
    SmartDashboard.putNumber("Quadrature Velocity", quadVelocity);
    SmartDashboard.putNumber("Pulse-width Position", pwmPosition);
    SmartDashboard.putNumber("Pulse-width Velocity", pwmVelocity);
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
