#!/usr/bin/env python3
"""
    This is a demo program for CANdle usage in Phoenix 6
"""
import wpilib
from enum import Enum
from phoenix6 import configs, controls, hardware, signals

class MyRobot(wpilib.TimedRobot):
    """
    Example program that shows how to use CANdle
    in Phoenix 6 python
    """

    # color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex
    GREEN = signals.RGBWColor(0, 217, 0, 0)
    WHITE = signals.RGBWColor(wpilib.Color.kWhite) * 0.5 # half brightness
    VIOLET = signals.RGBWColor.from_hsv(270, 0.9, 0.8)
    RED = signals.RGBWColor.from_hex("#D9000000") or signals.RGBWColor()

    # Start and end index for LED animations.
    # 0-7 are onboard, 8-399 are an external strip.
    # CANdle supports 8 animation slots (0-7).
    SLOT_0_START_IDX = 8
    SLOT_0_END_IDX = 37

    SLOT_1_START_IDX = 38
    SLOT_1_END_IDX = 67

    class AnimationType(Enum):
        NONE = 0
        COLOR_FLOW = 1
        FIRE = 2
        LARSON = 3
        RAINBOW = 4
        RGB_FADE = 5
        SINGLE_FADE = 6
        STROBE = 7
        TWINKLE = 8
        TWINKLE_OFF = 9

    def robotInit(self):
        """Robot initialization function"""

        # Keep a reference to all the devices used
        self.candle = hardware.CANdle(1, "*")

        # Configure CANdle
        cfg = configs.CANdleConfiguration()
        # set the LED strip type and brightness
        cfg.led.strip_type = signals.StripTypeValue.GRB
        cfg.led.brightness_scalar = 0.5
        # disable status LED when being controlled
        cfg.candle_features.status_led_when_active = signals.StatusLedWhenActiveValue.DISABLED

        self.candle.configurator.apply(cfg)

        # clear all previous animations
        for i in range(0, 8):
            self.candle.set_control(controls.EmptyAnimation(i))
        # set the onboard LEDs to a solid color
        self.candle.set_control(controls.SolidColor(0, 3).with_color(self.GREEN))
        self.candle.set_control(controls.SolidColor(4, 7).with_color(self.WHITE))

        self.anim_0_state = self.AnimationType.NONE
        self.anim_1_state = self.AnimationType.NONE

        # add animations to chooser for slot 0
        self.anim_0_chooser = wpilib.SendableChooser()
        self.anim_0_chooser.setDefaultOption("Color Flow", self.AnimationType.COLOR_FLOW)
        self.anim_0_chooser.addOption("Rainbow", self.AnimationType.RAINBOW)
        self.anim_0_chooser.addOption("Twinkle", self.AnimationType.TWINKLE)
        self.anim_0_chooser.addOption("Twinkle Off", self.AnimationType.TWINKLE_OFF)
        self.anim_0_chooser.addOption("Fire", self.AnimationType.FIRE)

        # add animations to chooser for slot 1
        self.anim_1_chooser = wpilib.SendableChooser()
        self.anim_1_chooser.setDefaultOption("Larson", self.AnimationType.LARSON)
        self.anim_1_chooser.addOption("RGB Fade", self.AnimationType.RGB_FADE)
        self.anim_1_chooser.addOption("Single Fade", self.AnimationType.SINGLE_FADE)
        self.anim_1_chooser.addOption("Strobe", self.AnimationType.STROBE)
        self.anim_1_chooser.addOption("Fire", self.AnimationType.FIRE)

        wpilib.SmartDashboard.putData("Animation 0", self.anim_0_chooser)
        wpilib.SmartDashboard.putData("Animation 1", self.anim_1_chooser)

    def robotPeriodic(self):
        """Robot periodic function"""

        # if the selection for slot 0 changes, change animations
        anim_0_selection = self.anim_0_chooser.getSelected()
        if self.anim_0_state != anim_0_selection:
            self.anim_0_state = anim_0_selection

            match self.anim_0_state:
                case self.AnimationType.COLOR_FLOW:
                    self.candle.set_control(
                        controls.ColorFlowAnimation(self.SLOT_0_START_IDX, self.SLOT_0_END_IDX).with_slot(0)
                        .with_color(self.VIOLET)
                    )
                case self.AnimationType.RAINBOW:
                    self.candle.set_control(
                        controls.RainbowAnimation(self.SLOT_0_START_IDX, self.SLOT_0_END_IDX).with_slot(0)
                    )
                case self.AnimationType.TWINKLE:
                    self.candle.set_control(
                        controls.TwinkleAnimation(self.SLOT_0_START_IDX, self.SLOT_0_END_IDX).with_slot(0)
                        .with_color(self.VIOLET)
                    )
                case self.AnimationType.TWINKLE_OFF:
                    self.candle.set_control(
                        controls.TwinkleOffAnimation(self.SLOT_0_START_IDX, self.SLOT_0_END_IDX).with_slot(0)
                        .with_color(self.VIOLET)
                    )
                case self.AnimationType.FIRE:
                    self.candle.set_control(
                        controls.FireAnimation(self.SLOT_0_START_IDX, self.SLOT_0_END_IDX).with_slot(0)
                    )

        # if the selection for slot 1 changes, change animations
        anim_1_selection = self.anim_1_chooser.getSelected()
        if self.anim_1_state != anim_1_selection:
            self.anim_1_state = anim_1_selection

            match self.anim_1_state:
                case self.AnimationType.LARSON:
                    self.candle.set_control(
                        controls.LarsonAnimation(self.SLOT_1_START_IDX, self.SLOT_1_END_IDX).with_slot(1)
                        .with_color(self.RED)
                    )
                case self.AnimationType.RGB_FADE:
                    self.candle.set_control(
                        controls.RgbFadeAnimation(self.SLOT_1_START_IDX, self.SLOT_1_END_IDX).with_slot(1)
                    )
                case self.AnimationType.SINGLE_FADE:
                    self.candle.set_control(
                        controls.SingleFadeAnimation(self.SLOT_1_START_IDX, self.SLOT_1_END_IDX).with_slot(1)
                        .with_color(self.RED)
                    )
                case self.AnimationType.STROBE:
                    self.candle.set_control(
                        controls.StrobeAnimation(self.SLOT_1_START_IDX, self.SLOT_1_END_IDX).with_slot(1)
                        .with_color(self.RED)
                    )
                case self.AnimationType.FIRE:
                    # direction can be reversed by either the Direction parameter or switching start and end
                    self.candle.set_control(
                        controls.FireAnimation(self.SLOT_1_START_IDX, self.SLOT_1_END_IDX).with_slot(1)
                        .with_direction(signals.AnimationDirectionValue.BACKWARD)
                        .with_cooling(0.4)
                        .with_sparking(0.5)
                    )

if __name__ == "__main__":
    wpilib.run(MyRobot)
