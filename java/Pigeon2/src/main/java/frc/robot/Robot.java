// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.hardware.Pigeon2;
import com.ctre.phoenixpro.sim.Pigeon2SimState;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final double PRINT_PERIOD = 0.5; // Update every 500 ms

  private final Pigeon2 pidgey = new Pigeon2(1, "rio");
  private double currentTime = Timer.getFPGATimestamp();

  /* Sim only */
  private final double HEIGHT = 1;
  private final double WIDTH = 1;
  private final double ROOT_X = WIDTH / 2;
  private final double ROOT_Y = HEIGHT / 2;

  private final Pigeon2SimState pidgeySim = pidgey.getSimState(); // We need a sim state in order to change the values of pidgey
  private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 100, 100);
  private final XboxController controller = new XboxController(0); // Uses an Xbox controller for setting the CANcoder simulation
  private Mechanism2d mech = new Mechanism2d(WIDTH, HEIGHT); // Main mechanism object
  private MechanismLigament2d wrist = mech.
                                      getRoot("base", ROOT_X, ROOT_Y).
                                      append(new MechanismLigament2d("Wrist", .25, 90, 6, new Color8Bit(Color.kAliceBlue)));

  private MechanismLigament2d leftArrow = wrist.append(new MechanismLigament2d("LeftArrow", 0.1, 150, 6, new Color8Bit(Color.kAliceBlue)));
  private MechanismLigament2d rightArrow = wrist.append(new MechanismLigament2d("RightArrow", 0.1, -150, 6, new Color8Bit(Color.kAliceBlue)));
  /* End sim only */



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Configure Pigeon2 */
    var toApply = new Pigeon2Configuration();

    /* User can change the configs if they want, or leave it empty for factory-default */

    pidgey.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    pidgey.getYaw().setUpdateFrequency(100);
    pidgey.getGravityVectorZ().setUpdateFrequency(100);
  }

  @Override
  public void robotPeriodic() {
    if (Timer.getFPGATimestamp() - currentTime > PRINT_PERIOD) {
      currentTime += PRINT_PERIOD;

      /**
       * getYaw automatically calls refresh(), no need to manually refresh.
       * 
       * StatusSignalValues also have the toString method implemented, to provide
       * a useful print of the signal.
       */
      var yaw = pidgey.getYaw();
      System.out.println("Yaw is " + yaw.toString() + " with " + yaw.getTimestamp().getLatency() + " seconds of latency");

      /**
       * Get the gravity vector Z component StatusSignalValue
       */
      var gravityVectorZ = pidgey.getGravityVectorZ();
      /* This time wait for the signal to reduce latency */
      gravityVectorZ.waitForUpdate(PRINT_PERIOD); // Wait up to our period
      /**
       * This uses the explicit getValue and getUnits functions to print, even though it's not
       * necessary for the ostream print
       */
      System.out.println("Gravity Vector in the Z direction is " +
                         gravityVectorZ.getValue() + " " +
                         gravityVectorZ.getUnits() + " with " +
                         gravityVectorZ.getTimestamp().getLatency() + " seconds of latency");
      /**
       * Notice when running this example that the second print's latency is always shorter than the first print's latency.
       * This is because we explicitly wait for the signal using the waitForUpdate() method instead of using the refresh()
       * method, which only gets the last cached value (similar to how Phoenix v5 works).
       * This can be used to make sure we synchronously update our control loop from the CAN bus, reducing any latency or jitter in
       * CAN bus measurements.
       * When the device is on a CANivore, the reported latency is very close to the true latency of the sensor, as the CANivore
       * timestamps when it receives the frame. This can be further used for latency compensation.
       */
      System.out.println();
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    /**
     * When we teleop init, set the position of the Pigeon2 and wait for the setter to take affect.
     */
    pidgey.setYaw(144, 0.1); // Set our yaw to 144 degrees and wait up to 100 ms for the setter to take affect
    pidgey.getYaw().waitForUpdate(0.1); // And wait up to 100 ms for the position to take affect
    System.out.println("Set the position to 144 degrees, we are currently at " + pidgey.getYaw()); // Use java's implicit toString operator
  }

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
  public void simulationPeriodic() {
    double Yaxis = controller.getLeftY();
    double motorVoltage = Yaxis * 12; // scales joystick axis to motor voltage ( +-12v)
    motorSim.setInputVoltage(motorVoltage);
    motorSim.update(.02);
    double position = motorSim.getAngularPositionRotations()*360;
    pidgeySim.setRawYaw(position);

    SmartDashboard.putData("mech2d", mech);
    
    wrist.setAngle(position); //converts 1 rotation to 360 degrees
  }
}
