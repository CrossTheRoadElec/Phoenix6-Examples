// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

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
  private static final double PRINT_PERIOD = 0.5; // Update every 500 ms

  private final CANBus kCANBus = new CANBus("rio");

  private final TalonFX talonFX = new TalonFX(2, kCANBus);
  private final CANdi candi = new CANdi(1, kCANBus);

  private final DutyCycleOut fwdOut = new DutyCycleOut(0);
  private final XboxController controller = new XboxController(0);

  private double currentTime = Timer.getFPGATimestamp();

  private final Mechanisms mechanism = new Mechanisms();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    /* Configure CANdi */
    var toApply = new CANdiConfiguration();

    toApply.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh; // Pulse-width sensor will drive low. Default of FloatDetect will typically work on most sensors.
    toApply.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow; // This example specifically assumes a hardware limit switch will close S2 to Ground. Default of CloseWhenNotFloating will also work

    toApply.PWM1.SensorDirection = true; // Invert the PWM1 position.
    toApply.PWM1.AbsoluteSensorDiscontinuityPoint = 0.75; // If the PWM 1 position on boot is after 0.75 rotations, treat it as x - 1 rotations.
                                                          // As an example, if the position is 0.87, it will boot to 0.87 - 1 = -0.13 rotations.

    /* User can change the configs if they want, or leave it empty for factory-default */
    candi.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    BaseStatusSignal.setUpdateFrequencyForAll(100, candi.getPWM1Position(), candi.getPWM1Velocity(), candi.getS2State());


    /* Also configure TalonFX to use CANdi as a remote sensor for PWM 1, and Limit switch for S2 */
    var fxConfigs = new TalonFXConfiguration();
    fxConfigs.Feedback.withFusedCANdiPwm1(candi);
    fxConfigs.HardwareLimitSwitch.withReverseLimitRemoteCANdiS2(candi);
    talonFX.getConfigurator().apply(fxConfigs);
  }

  @Override
  public void robotPeriodic() {
    if (Timer.getFPGATimestamp() - currentTime > PRINT_PERIOD) {
      currentTime += PRINT_PERIOD;

      /**
       * getPosition automatically calls refresh(), no need to manually refresh.
       * 
       * StatusSignalValues also have the toString method implemented, to provide
       * a useful print of the signal.
       */
      var pos = candi.getPWM1Position();
      System.out.println("Position is " + pos.toString() + " with " + pos.getTimestamp().getLatency() + " seconds of latency");

      /**
       * Get the S2 State StatusSignalValue without refreshing
       */
      var s2State = candi.getS2State(false);
      /* This time wait for the signal to reduce latency */
      s2State.waitForUpdate(PRINT_PERIOD); // Wait up to our period
      /**
       * This uses the explicit getValue and getUnits functions to print, even though it's not
       * necessary for the ostream print
       */
      System.out.println(
        "S2 State is " +
        s2State.getValue() + " " +
        s2State.getUnits() + " with " +
        s2State.getTimestamp().getLatency() + " seconds of latency"
      );
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
    mechanism.update(candi.getPWM1Position());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    /* Send control requests to control Talon that's connected to CANdi */
    talonFX.setControl(fwdOut.withOutput(controller.getLeftY()));
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
    PhysicsSim.getInstance().addTalonFX(talonFX, candi, 25, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
