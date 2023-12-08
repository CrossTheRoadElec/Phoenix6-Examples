// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.sim.PhysicsSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final double PRINT_PERIOD = 0.5; // Update every 500 ms

  /* Keep a reference for a TalonFX around so we can drive the thing the Pigeon is on */
  private final TalonFX talonfx = new TalonFX(0, "rio");
  private final Pigeon2 pidgey = new Pigeon2(1, "rio");
  private double currentTime = Timer.getFPGATimestamp();

  private final XboxController joystick = new XboxController(0);
  private final DutyCycleOut control = new DutyCycleOut(0);

  private final Mechanisms mechanisms = new Mechanisms();

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
    mechanisms.update(pidgey.getYaw());
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
  public void teleopPeriodic() {
    talonfx.setControl(control.withOutput(joystick.getLeftY()));
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
    PhysicsSim.getInstance().addTalonFX(talonfx, pidgey, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
