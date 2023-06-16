// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignalValue;
import com.ctre.phoenix6.StatusSignalValue;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TalonFX m_motor1 = new TalonFX(0, "*"); // Pick the first CANivore bus
  Pigeon2 m_pigdey = new Pigeon2(1, "*"); // Pick the first CANivore bus also
  TalonFX m_transcientMotor = new TalonFX(20, "*"); // This motor may or may not be on the bus, 
                                                                     // selectively power it to completely test this example 
  TalonFX m_motor2 = new TalonFX(0, "rio"); // Pick the RIO bus to force a failure we can detect

  StatusSignalValue<Double> m_canbus1signal1 = m_motor1.getPosition();
  StatusSignalValue<Double> m_canbus1signal2 = m_motor1.getVelocity();
  StatusSignalValue<ControlModeValue> m_canbus1signal3 = m_motor1.getControlMode();
  StatusSignalValue<Double> m_canbus1signal4 = m_pigdey.getYaw();
  StatusSignalValue<Double> m_canbus1signal5 = m_pigdey.getRoll();
  
  StatusSignalValue<Double> m_canbus2signal1 = m_motor2.getPosition();
  
  StatusSignalValue<Double> m_canbus1transcient1 = m_transcientMotor.getPosition();
  StatusSignalValue<Double> m_canbus1transcient2 = m_transcientMotor.getVelocity();

  StatusSignalValue<?>[] m_signalsAcrossCANbuses = new StatusSignalValue<?>[]{
    m_canbus1signal1,
    m_canbus2signal1
  };
  StatusSignalValue<?>[] m_lotsOfSignals = new StatusSignalValue<?>[]{
    m_canbus1signal1,
    m_canbus1signal2,
    m_canbus1signal3,
    m_canbus1signal5
  };
  StatusSignalValue<?>[] m_noSignals = new StatusSignalValue<?>[]{};
  StatusSignalValue<?>[] m_tanscientSignals = new StatusSignalValue<?>[]{
    m_canbus1signal1,
    m_canbus1signal2,
    m_canbus1transcient1,
    m_canbus1transcient2
  };

  XboxController m_joystick = new XboxController(0); // Allow us to see the different errors

  double m_waitForAllTimeout = 0.1;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    if(m_joystick.getLeftBumper()){
      m_waitForAllTimeout = 0.1;
      System.out.println("Timeout is now at " + m_waitForAllTimeout);
    }
    if(m_joystick.getRightBumper()){
      m_waitForAllTimeout = 0;
      System.out.println("Timeout is now at " + m_waitForAllTimeout);
    }

    /* If we press the A button, test what happens when we wait on lots of signals (normal use case) */
    if(m_joystick.getAButtonPressed()) {
      var status = BaseStatusSignalValue.waitForAll(m_waitForAllTimeout, m_lotsOfSignals);
      System.out.println("Status of waiting on signals (normal use case): " + status);
      for(var sig : m_lotsOfSignals) {
        System.out.println("Signal status: " + sig.getError());
      }
    }
    /* If we press the B button, test what happens when we wait on signals from different busses */
    if(m_joystick.getBButtonPressed()) {
      var status = BaseStatusSignalValue.waitForAll(m_waitForAllTimeout, m_signalsAcrossCANbuses);
      System.out.println("Status of waiting on signals across different CAN busses: " + status);
      for(var sig : m_signalsAcrossCANbuses) {
        System.out.println("Signal status: " + sig.getError());
      }
    }
    /* If we press the Y button, test what happens when we wait on no signals */
    if(m_joystick.getYButtonPressed()) {
      var status = BaseStatusSignalValue.waitForAll(m_waitForAllTimeout, m_noSignals);
      System.out.println("Status of waiting on no signals: " + status);
      for(var sig : m_noSignals) {
        System.out.println("Signal status: " + sig.getError());
      }
    }
    /* If we press the X button, test what happens when we wait on signals with the transcient motor controller */
    if(m_joystick.getXButtonPressed()) {
      var status = BaseStatusSignalValue.waitForAll(m_waitForAllTimeout, m_tanscientSignals);
      System.out.println("Status of waiting on transcient signals: " + status);
      for(var sig : m_tanscientSignals) {
        System.out.println("Signal status: " + sig.getError());
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
