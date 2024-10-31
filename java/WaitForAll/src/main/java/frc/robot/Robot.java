// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

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
  private final TalonFX m_motor1 = new TalonFX(0, "*"); // Pick the first CANivore bus
  private final Pigeon2 m_pigdey = new Pigeon2(1, "*"); // Pick the first CANivore bus also
  private final TalonFX m_transcientMotor = new TalonFX(20, "*"); // This motor may or may not be on the bus, 
                                                                  // selectively power it to completely test this example 
  private final TalonFX m_motor2 = new TalonFX(0, "rio"); // Pick the RIO bus to force a failure we can detect

  private final StatusSignal<Angle> m_canbus1signal1 = m_motor1.getPosition(false);
  private final StatusSignal<AngularVelocity> m_canbus1signal2 = m_motor1.getVelocity(false);
  private final StatusSignal<ControlModeValue> m_canbus1signal3 = m_motor1.getControlMode(false);
  private final StatusSignal<Angle> m_canbus1signal4 = m_pigdey.getYaw(false);
  private final StatusSignal<Angle> m_canbus1signal5 = m_pigdey.getRoll(false);
  
  private final StatusSignal<Angle> m_canbus2signal1 = m_motor2.getPosition(false);
  
  private final StatusSignal<Angle> m_canbus1transcient1 = m_transcientMotor.getPosition(false);
  private final StatusSignal<AngularVelocity> m_canbus1transcient2 = m_transcientMotor.getVelocity(false);

  private final StatusSignal<?>[] m_signalsAcrossCANbuses = new StatusSignal<?>[]{
    m_canbus1signal1,
    m_canbus2signal1
  };
  private final StatusSignal<?>[] m_lotsOfSignals = new StatusSignal<?>[]{
    m_canbus1signal1,
    m_canbus1signal2,
    m_canbus1signal3,
    m_canbus1signal4,
    m_canbus1signal5
  };
  private final StatusSignal<?>[] m_noSignals = new StatusSignal<?>[]{};
  private final StatusSignal<?>[] m_tanscientSignals = new StatusSignal<?>[]{
    m_canbus1signal1,
    m_canbus1signal2,
    m_canbus1transcient1,
    m_canbus1transcient2
  };

  private final XboxController m_joystick = new XboxController(0); // Allow us to see the different errors

  private double m_waitForAllTimeout = 0.1;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    if (m_joystick.getLeftBumperButton()){
      m_waitForAllTimeout = 0.1;
      System.out.println("Timeout is now at " + m_waitForAllTimeout);
    }
    if (m_joystick.getRightBumperButton()){
      m_waitForAllTimeout = 0;
      System.out.println("Timeout is now at " + m_waitForAllTimeout);
    }

    /* If we press the A button, test what happens when we wait on lots of signals (normal use case) */
    if (m_joystick.getAButtonPressed()) {
      var status = BaseStatusSignal.waitForAll(m_waitForAllTimeout, m_lotsOfSignals);
      System.out.println("Status of waiting on signals (normal use case): " + status);
      for (var sig : m_lotsOfSignals) {
        System.out.println("Signal status: " + sig.getStatus());
      }
    }
  
    /* If we press the B button, test what happens when we wait on signals from different busses */
    if (m_joystick.getBButtonPressed()) {
      var status = BaseStatusSignal.waitForAll(m_waitForAllTimeout, m_signalsAcrossCANbuses);
      System.out.println("Status of waiting on signals across different CAN busses: " + status);
      for (var sig : m_signalsAcrossCANbuses) {
        System.out.println("Signal status: " + sig.getStatus());
      }
    }
  
    /* If we press the Y button, test what happens when we wait on no signals */
    if (m_joystick.getYButtonPressed()) {
      var status = BaseStatusSignal.waitForAll(m_waitForAllTimeout, m_noSignals);
      System.out.println("Status of waiting on no signals: " + status);
      for (var sig : m_noSignals) {
        System.out.println("Signal status: " + sig.getStatus());
      }
    }
  
    /* If we press the X button, test what happens when we wait on signals with the transcient motor controller */
    if (m_joystick.getXButtonPressed()) {
      var status = BaseStatusSignal.waitForAll(m_waitForAllTimeout, m_tanscientSignals);
      System.out.println("Status of waiting on transcient signals: " + status);
      for (var sig : m_tanscientSignals) {
        System.out.println("Signal status: " + sig.getStatus());
      }
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
