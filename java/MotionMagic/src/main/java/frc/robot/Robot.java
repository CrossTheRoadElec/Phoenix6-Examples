// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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
  TalonFX m_motor = new TalonFX(1, "Fred");
  MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  XboxController m_joystick = new XboxController(0);
  int m_printCount = 0;

  /* Sim only */
  double HEIGHT = .5; // Controls tyhe height of the mech2d smart dashboard
  double WIDTH = 1; // Controls tyhe height of the mech2d smart dashboard
  double MOTIONMAGIC = 1; // This value changes in the simulationPeriodic method

  DCMotorSim m_motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 100, .001);
  Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
  MechanismLigament2d wrist = mech.
                              getRoot("base", 0.5, 0.4).
                              append(new MechanismLigament2d("motionMagic", MOTIONMAGIC, 0, 6, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d reference = mech.
                              getRoot("Reference", 0, .1).
                              append(new MechanismLigament2d("reference", 1, 0, 6, new Color8Bit(Color.kCyan)));

  MechanismLigament2d joint = mech.
                              getRoot("joint", 0.5, .1).
                              append(new MechanismLigament2d("joint", 0.3, 90, 6, new Color8Bit(Color.kCyan)));
  /* End sim only */

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_motor, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = new MotionMagicConfigs();
    mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 50;
    cfg.MotionMagic = mm;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 60;
    slot0.kI = 0;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving
    cfg.Slot0 = slot0;

    FeedbackConfigs fdb = new FeedbackConfigs();
    fdb.SensorToMechanismRatio = 12.8;
    cfg.Feedback = fdb;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void robotPeriodic() {
    if (m_printCount++ > 10) {
      m_printCount = 0;
      System.out.println("Pos: " + m_motor.getPosition());
      System.out.println("Vel: " + m_motor.getVelocity());
      System.out.println();
    }
    MOTIONMAGIC = m_motor.getPosition().getValue();
    SmartDashboard.putData("mech2d", mech);
    wrist.setLength(MOTIONMAGIC/25); //Divide by 25 to scale motion to fit in the window
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    /* Deadband the joystick */
    double leftY = m_joystick.getLeftY();
    if(leftY > -0.1 && leftY < 0.1) leftY = 0;

    m_motor.setControl(m_mmReq.withPosition(leftY * 10).withSlot(0));
    if(m_joystick.getBButton()) {
      m_motor.setRotorPosition(1);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
