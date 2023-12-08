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
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.sim.PhysicsSim;

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
  StatusSignal<Boolean> f_remoteSensorInvalid = m_fx.getFault_RemoteSensorDataInvalid();
  StatusSignal<Boolean> sf_remoteSensorInvalid = m_fx.getStickyFault_RemoteSensorDataInvalid();

  StatusSignal<Double> fx_pos = m_fx.getPosition();
  StatusSignal<Double> fx_vel = m_fx.getVelocity();
  StatusSignal<Double> cc_pos = m_cc.getPosition();
  StatusSignal<Double> cc_vel = m_cc.getVelocity();
  StatusSignal<Double> fx_rotorPos = m_fx.getRotorPosition();

  DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);

  XboxController m_joystick = new XboxController(0);

  int printCount = 0;

  /* Sim only */
  double HEIGHT = 1; //Controls tyhe height of the mech2d SmartDashboard
  double WIDTH = 1; //Controls tyhe height of the mech2d SmartDashboard
  double FX = 0;
  double TALON = 0;
  double CAN = 0;

  DCMotorSim m_motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 100, 0.001);
  CANcoderSimState m_ccSim = m_cc.getSimState(); // We need a sim state in order to change the values of CANcoder
  Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
  //f(x)
  MechanismLigament2d FXarm = mech.
                                getRoot("FXpivotPoint", 0.25, 0.75).
                                append(new MechanismLigament2d("FXarm", .1, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d FXside1 = FXarm.append(new MechanismLigament2d("FXside1", 0.076535, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d FXside2 = FXside1.append(new MechanismLigament2d("FXside2", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d FXside3 = FXside2.append(new MechanismLigament2d("FXside3", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d FXside4 = FXside3.append(new MechanismLigament2d("FXside4", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d FXside5 = FXside4.append(new MechanismLigament2d("FXside5", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d FXside6 = FXside5.append(new MechanismLigament2d("FXside6", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d FXside7 = FXside6.append(new MechanismLigament2d("FXside7", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d FXside8 = FXside7.append(new MechanismLigament2d("FXside8", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  //CANcoder
  MechanismLigament2d CANarm = mech.
                                getRoot("CANpivotPoint", 0.25, 0.25).
                                append(new MechanismLigament2d("CANarm", .1, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d CANside1 = CANarm.append(new MechanismLigament2d("CANside1", 0.076535, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d CANside2 = CANside1.append(new MechanismLigament2d("CANside2", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d CANside3 = CANside2.append(new MechanismLigament2d("CANside3", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d CANside4 = CANside3.append(new MechanismLigament2d("CANside4", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d CANside5 = CANside4.append(new MechanismLigament2d("CANside5", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d CANside6 = CANside5.append(new MechanismLigament2d("CANside6", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d CANside7 = CANside6.append(new MechanismLigament2d("CANside7", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d CANside8 = CANside7.append(new MechanismLigament2d("CANside8", 0.076535, 45, 6, new Color8Bit(Color.kAliceBlue)));
  //TalonFX 
  MechanismLigament2d TALONarm = mech.
                                getRoot("TALONpivotPoint", 0.75, 0.5).
                                append(new MechanismLigament2d("TALONarm", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d TALONside1 = TALONarm.append(new MechanismLigament2d("TALONside1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d TALONside2 = TALONside1.append(new MechanismLigament2d("TALONside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d TALONside3 = TALONside2.append(new MechanismLigament2d("TALONside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d TALONside4 = TALONside3.append(new MechanismLigament2d("TALONside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d TALONside5 = TALONside4.append(new MechanismLigament2d("TALONside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d TALONside6 = TALONside5.append(new MechanismLigament2d("TALONside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d TALONside7 = TALONside6.append(new MechanismLigament2d("TALONside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d TALONside8 = TALONside7.append(new MechanismLigament2d("TALONside2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  /* End sim only */
  
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
      f_remoteSensorInvalid.refresh();
      sf_remoteSensorInvalid.refresh();
      boolean anyFault = sf_fusedSensorOutOfSync.getValue() || sf_remoteSensorInvalid.getValue();
      if(anyFault) {
        System.out.println("A fault has occurred:");
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if(f_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync live-faulted");
        } else if (sf_fusedSensorOutOfSync.getValue()) {
          System.out.println("Fused sensor out of sync sticky-faulted");
        }
        /* If we're live, indicate live, otherwise if we're sticky indicate sticky, otherwise do nothing */
        if(f_remoteSensorInvalid.getValue()) {
          System.out.println("Missing remote sensor live-faulted");
        } else if (sf_remoteSensorInvalid.getValue()) {
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
    FXarm.setAngle(fx_rotorPos.refresh().getValue() * 360);
    TALONarm.setAngle(fx_pos.refresh().getValue() * 360);
    CANarm.setAngle(cc_pos.refresh().getValue() * 360);
    SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
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
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_fx, m_cc, 25, 0.001);
    cc_pos.setUpdateFrequency(10);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
