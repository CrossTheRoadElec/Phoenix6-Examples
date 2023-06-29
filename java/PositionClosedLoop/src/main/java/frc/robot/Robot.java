// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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
  private final TalonFX m_fx = new TalonFX(0);
  
  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, enable FOC, no feed forward, use slot 0 */
  private final PositionVoltage m_voltagePosition = new PositionVoltage(0, true, 0, 0, false);
  /* Start at position 0, no feed forward, use slot 1 */
  private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 1, false);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private final XboxController m_joystick = new XboxController(0);

  /* Mech2d only */
  double HEIGHT = .5; //Controls tyhe height of the mech2d SmartDashboard
  double WIDTH = 1; //Controls tyhe height of the mech2d SmartDashboard
  double PCL = 1;

  Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
  MechanismLigament2d wrist = mech.
                              getRoot("PCL", 0.5, 0.4).
                              append(new MechanismLigament2d("PCL",  PCL, 0, 6, new Color8Bit(Color.kAliceBlue)));
  
  MechanismLigament2d reference = mech.
                              getRoot("Reference", 0, .1).
                              append(new MechanismLigament2d("reference", 1, 0, 6, new Color8Bit(Color.kCyan)));
  
  MechanismLigament2d joint = mech.
                              getRoot("joint", 0.5, .1).
                              append(new MechanismLigament2d("joint", 0.3, 90, 6, new Color8Bit(Color.kCyan)));
  /* End mech2d only */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 12; // An error of 0.5 rotations results in 12V output
    configs.Slot0.kD = 1; // A change of 1 rotation per second results in 0.1 volts output
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    configs.Slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    configs.Slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 130 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 130;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at 0 */
    m_fx.setRotorPosition(0);
  }

  @Override
  public void robotPeriodic() { 
    wrist.setLength(PCL/30); // Divide by 2 to scale motion to fit in the window
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
    double desiredRotations = m_joystick.getLeftY() * 10; // Go for plus/minus 10 rotations
    if (Math.abs(desiredRotations) <= 0.5) { // Joystick deadzone
      desiredRotations = 0;
    }
    if (m_joystick.getLeftBumper()) {
      /* Use voltage position */
      m_fx.setControl(m_voltagePosition.withPosition(desiredRotations));
    }
    else if (m_joystick.getRightBumper()) {
      /* Use torque position */
      m_fx.setControl(m_torquePosition.withPosition(desiredRotations));
    }
    else {
      /* Disable the motor instead */
      m_fx.setControl(m_brake);
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

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(m_fx, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
    PCL = m_fx.getPosition().getValue();
  }
}
