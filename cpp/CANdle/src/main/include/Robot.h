// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/phoenix6/CANdle.hpp>

class Robot : public frc::TimedRobot {
private:
    using RGBWColor = ctre::phoenix6::signals::RGBWColor;

    /* color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
    static constexpr RGBWColor kGreen{0, 217, 0, 0};
    static constexpr RGBWColor kWhite = RGBWColor{frc::Color::kWhite} * 0.5; /* half brightness */
    static constexpr RGBWColor kViolet = RGBWColor::FromHSV(270_deg, 0.9, 0.8);
    static constexpr RGBWColor kRed = RGBWColor::FromHex("#D9000000").value();

    /*
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-399 are an external strip.
     * CANdle supports 8 animation slots (0-7).
     */
    static constexpr int kSlot0StartIdx = 8;
    static constexpr int kSlot0EndIdx = 37;

    static constexpr int kSlot1StartIdx = 38;
    static constexpr int kSlot1EndIdx = 67;

    ctre::phoenix6::hardware::CANdle m_candle{1, ctre::phoenix6::CANBus::RoboRIO()};

    enum class AnimationType {
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
    };

    AnimationType m_anim0State{AnimationType::None};
    AnimationType m_anim1State{AnimationType::None};

    frc::SendableChooser<AnimationType> m_anim0Chooser;
    frc::SendableChooser<AnimationType> m_anim1Chooser;

public:
    Robot();
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void SimulationInit() override;
    void SimulationPeriodic() override;
};
