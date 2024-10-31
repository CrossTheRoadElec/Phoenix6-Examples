// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private final TalonFX m_fx = new TalonFX(0, "*"); 
    private final CANcoder m_cc = new CANcoder(1, "*");

    private final DutyCycleOut m_out = new DutyCycleOut(0);

    private final XboxController m_joystick = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
    public Robot() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANcoder;
        configs.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        configs.HardwareLimitSwitch.ForwardLimitRemoteSensorID = 1;
        configs.HardwareLimitSwitch.ForwardLimitEnable = true;
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_fx.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
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
        m_fx.setControl(
            m_out.withOutput(m_joystick.getLeftY())
                .withLimitForwardMotion(m_joystick.getLeftBumperButton())
                .withLimitReverseMotion(m_joystick.getRightBumperButton())
        );
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
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {
        var fxSim = m_fx.getSimState();
        fxSim.setForwardLimit(m_joystick.getAButton());
        fxSim.setReverseLimit(m_joystick.getBButton());

        var ccSim = m_cc.getSimState();
        ccSim.setMagnetHealth(m_joystick.getYButton()
            ? MagnetHealthValue.Magnet_Green
            : MagnetHealthValue.Magnet_Red);
    }
}
