// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /* color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
    private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();

    /*
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-399 are an external strip.
     * CANdle supports 8 animation slots (0-7).
     */
    private static final int kSlot0StartIdx = 8;
    private static final int kSlot0EndIdx = 37;

    private static final int kSlot1StartIdx = 38;
    private static final int kSlot1EndIdx = 67;

    private final CANdle m_candle = new CANdle(1, "rio");

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    private AnimationType m_anim0State = AnimationType.None;
    private AnimationType m_anim1State = AnimationType.None;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
    private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<AnimationType>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        /* Configure CANdle */
        var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

        /* clear all previous animations */
        for (int i = 0; i < 8; ++i) {
            m_candle.setControl(new EmptyAnimation(i));
        }
        /* set the onboard LEDs to a solid color */
        m_candle.setControl(new SolidColor(0, 3).withColor(kGreen));
        m_candle.setControl(new SolidColor(4, 7).withColor(kWhite));

        /* add animations to chooser for slot 0 */
        m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        m_anim0Chooser.addOption("Fire", AnimationType.Fire);

        /* add animations to chooser for slot 1 */
        m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
        m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
        m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
        m_anim1Chooser.addOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", m_anim0Chooser);
        SmartDashboard.putData("Animation 1", m_anim1Chooser);
    }

    @Override
    public void robotPeriodic() {
        /* if the selection for slot 0 changes, change animations */
        final var anim0Selection = m_anim0Chooser.getSelected();
        if (m_anim0State != anim0Selection) {
            m_anim0State = anim0Selection;

            switch (m_anim0State) {
                default:
                case ColorFlow:
                    m_candle.setControl(
                        new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case Rainbow:
                    m_candle.setControl(
                        new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
                case Twinkle:
                    m_candle.setControl(
                        new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case TwinkleOff:
                    m_candle.setControl(
                        new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                            .withColor(kViolet)
                    );
                    break;
                case Fire:
                    m_candle.setControl(
                        new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    );
                    break;
            }
        }

        /* if the selection for slot 1 changes, change animations */
        final var anim1Selection = m_anim1Chooser.getSelected();
        if (m_anim1State != anim1Selection) {
            m_anim1State = anim1Selection;

            switch (m_anim1State) {
                default:
                case Larson:
                    m_candle.setControl(
                        new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                case RgbFade:
                    m_candle.setControl(
                        new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    );
                    break;
                case SingleFade:
                    m_candle.setControl(
                        new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                case Strobe:
                    m_candle.setControl(
                        new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withColor(kRed)
                    );
                    break;
                case Fire:
                    /* direction can be reversed by either the Direction parameter or switching start and end */
                    m_candle.setControl(
                        new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                            .withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.4)
                            .withSparking(0.5)
                    );
                    break;
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
