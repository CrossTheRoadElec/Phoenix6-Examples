// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "Robot.h"

using namespace ctre::phoenix6;

constexpr units::time::second_t print_period{500_ms};

Robot::Robot() {
    /* Configure CANdle */
    configs::CANdleConfiguration cfg{};
    /* set the LED strip type and brightness */
    cfg.LED.StripType = signals::StripTypeValue::GRB;
    cfg.LED.BrightnessScalar = 0.5;
    /* disable status LED when being controlled */
    cfg.CANdleFeatures.StatusLedWhenActive = signals::StatusLedWhenActiveValue::Disabled;

    m_candle.GetConfigurator().Apply(cfg);

    /* clear all previous animations */
    for (int i = 0; i < 8; ++i) {
        m_candle.SetControl(controls::EmptyAnimation{i});
    }
    /* set the onboard LEDs to a solid color */
    m_candle.SetControl(controls::SolidColor{0, 3}.WithColor(kGreen));
    m_candle.SetControl(controls::SolidColor{4, 7}.WithColor(kWhite));

    /* add animations to chooser for slot 0 */
    m_anim0Chooser.SetDefaultOption("Color Flow", AnimationType::ColorFlow);
    m_anim0Chooser.AddOption("Rainbow", AnimationType::Rainbow);
    m_anim0Chooser.AddOption("Twinkle", AnimationType::Twinkle);
    m_anim0Chooser.AddOption("Twinkle Off", AnimationType::TwinkleOff);
    m_anim0Chooser.AddOption("Fire", AnimationType::Fire);

    /* add animations to chooser for slot 1 */
    m_anim1Chooser.SetDefaultOption("Larson", AnimationType::Larson);
    m_anim1Chooser.AddOption("RGB Fade", AnimationType::RgbFade);
    m_anim1Chooser.AddOption("Single Fade", AnimationType::SingleFade);
    m_anim1Chooser.AddOption("Strobe", AnimationType::Strobe);
    m_anim1Chooser.AddOption("Fire", AnimationType::Fire);

    frc::SmartDashboard::PutData("Animation 0", &m_anim0Chooser);
    frc::SmartDashboard::PutData("Animation 1", &m_anim1Chooser);
}

void Robot::RobotPeriodic() {
    /* if the selection for slot 0 changes, change animations */
    auto const anim0Selection = m_anim0Chooser.GetSelected();
    if (m_anim0State != anim0Selection) {
        m_anim0State = anim0Selection;

        switch (m_anim0State) {
            default:
            case AnimationType::ColorFlow:
                m_candle.SetControl(
                    controls::ColorFlowAnimation{kSlot0StartIdx, kSlot0EndIdx}.WithSlot(0)
                        .WithColor(kViolet)
                );
                break;
            case AnimationType::Rainbow:
                m_candle.SetControl(
                    controls::RainbowAnimation{kSlot0StartIdx, kSlot0EndIdx}.WithSlot(0)
                );
                break;
            case AnimationType::Twinkle:
                m_candle.SetControl(
                    controls::TwinkleAnimation{kSlot0StartIdx, kSlot0EndIdx}.WithSlot(0)
                        .WithColor(kViolet)
                );
                break;
            case AnimationType::TwinkleOff:
                m_candle.SetControl(
                    controls::TwinkleOffAnimation{kSlot0StartIdx, kSlot0EndIdx}.WithSlot(0)
                        .WithColor(kViolet)
                );
                break;
            case AnimationType::Fire:
                m_candle.SetControl(
                    controls::FireAnimation{kSlot0StartIdx, kSlot0EndIdx}.WithSlot(0)
                );
                break;
        }
    }

    /* if the selection for slot 1 changes, change animations */
    auto const anim1Selection = m_anim1Chooser.GetSelected();
    if (m_anim1State != anim1Selection) {
        m_anim1State = anim1Selection;

        switch (m_anim1State) {
            default:
            case AnimationType::Larson:
                m_candle.SetControl(
                    controls::LarsonAnimation{kSlot1StartIdx, kSlot1EndIdx}.WithSlot(1)
                        .WithColor(kRed)
                );
                break;
            case AnimationType::RgbFade:
                m_candle.SetControl(
                    controls::RgbFadeAnimation{kSlot1StartIdx, kSlot1EndIdx}.WithSlot(1)
                );
                break;
            case AnimationType::SingleFade:
                m_candle.SetControl(
                    controls::SingleFadeAnimation{kSlot1StartIdx, kSlot1EndIdx}.WithSlot(1)
                        .WithColor(kRed)
                );
                break;
            case AnimationType::Strobe:
                m_candle.SetControl(
                    controls::StrobeAnimation{kSlot1StartIdx, kSlot1EndIdx}.WithSlot(1)
                        .WithColor(kRed)
                );
                break;
            case AnimationType::Fire:
                /* direction can be reversed by either the Direction parameter or switching start and end */
                m_candle.SetControl(
                    controls::FireAnimation{kSlot1StartIdx, kSlot1EndIdx}.WithSlot(1)
                        .WithDirection(signals::AnimationDirectionValue::Backward)
                        .WithCooling(0.4)
                        .WithSparking(0.5)
                );
                break;
        }
    }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
