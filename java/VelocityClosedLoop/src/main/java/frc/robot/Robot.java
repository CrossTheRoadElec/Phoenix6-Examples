// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  private final TalonFX m_fllr = new TalonFX(1);
  
  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, true, 0, 0, false);
  /* Start at velocity 0, no feed forward, use slot 1 */
  private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 1, false);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

  private final XboxController m_joystick = new XboxController(0);

   /* Sim only */
   double HEIGHT = 1; //Controls tyhe height of the mech2d SmartDashboard
   double WIDTH = 1; //Controls tyhe height of the mech2d SmartDashboard
   double VCL = 1;
 
   Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT);
   // Velocity
   MechanismLigament2d VelocityMech = mech.
                               getRoot("VCL", 0.75, 0.5).
                               append(new MechanismLigament2d("VCL",  VCL,90, 6, new Color8Bit(Color.kAliceBlue)));
   
  MechanismLigament2d midline = mech.
                                getRoot("midline", 0.7, 0.5).
                                append(new MechanismLigament2d("midline", 0.1, 0, 3, new Color8Bit(Color.kCyan)));
                          
  //Position                            
  MechanismLigament2d arm = mech.
                                getRoot("pivotPoint", 0.25, 0.5).
                                append(new MechanismLigament2d("arm", .2, 0, 0, new Color8Bit(Color.kAliceBlue)));

  MechanismLigament2d side1 = arm.append(new MechanismLigament2d("side1", 0.15307, 112.5, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d side2 = side1.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d side3 = side2.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d side4 = side3.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d side5 = side4.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d side6 = side5.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d side7 = side6.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
  MechanismLigament2d side8 = side7.append(new MechanismLigament2d("side2", 0.15307, 45, 6, new Color8Bit(Color.kAliceBlue)));
   /* End sim only */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    m_fllr.setControl(new Follower(m_fx.getDeviceID(), false));
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double joyValue = m_joystick.getLeftY();
    if (joyValue > -0.1 && joyValue < 0.1) joyValue = 0;

    double desiredRotationsPerSecond = joyValue * 50; // Go for plus/minus 10 rotations per second
    if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
      desiredRotationsPerSecond = 0;
    }
    if (m_joystick.getLeftBumper()) {
      /* Use voltage velocity */
      m_fx.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
    }
    else if (m_joystick.getRightBumper()) {
      double friction_torque = (joyValue > 0) ? 1 : -1; // To account for friction, we add this to the arbitrary feed forward
      /* Use torque velocity */
      m_fx.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
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
    VCL = m_fx.getRotorVelocity().getValue();
    VelocityMech.setLength(VCL/120); //Divide by 2 to scale motion to fit in the window
    double position = m_fx.getPosition().getValue() / 25; //Gear reduction by 25:1 so rotation is visible in sim
    arm.setAngle(position*360);

    SmartDashboard.putData("mech2d", mech); // Creates mech2d in SmartDashboard
  }
}
