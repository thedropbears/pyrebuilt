from phoenix6.controls import ColorFlowAnimation, RainbowAnimation, SolidColor
from phoenix6.hardware.candle import CANdle
from phoenix6.signals import RGBWColor

from ids import CandleId
from utilities.game import is_alliance_hub_active

"""
Avaible Animations:

SolidColor
EmptyAnimation
ColorFlowAnimation
FireAnimation
LarsonAnimation
RainbowAnimation
RgbFadeAnimation
SingleFadeAnimation
StrobeAnimation
TwinkleAnimation
TwinkleOffAnimation
"""


class LEDComponent:
    yellow = RGBWColor(255, 255, 0, 0)
    red = RGBWColor(255, 0, 0, 0)
    blue = RGBWColor(0, 0, 255, 0)
    green = RGBWColor(0, 255, 0, 0)
    white = RGBWColor(255, 255, 255, 255)
    LED_START = 0
    LED_END = 255

    def __init__(self):
        self.candle = CANdle(device_id=CandleId.LED)
        self.set_rainbow()

    def set_leds(self, animation, color):
        self.desired_command = animation(
            led_start_index=self.LED_START,
            led_end_index=self.LED_END,
            color=color,
        )

    def set_rainbow(self):
        self.desired_command = RainbowAnimation(
            led_start_index=self.LED_START,
            led_end_index=self.LED_END,
        )

    def set_test_lights(self):
        self.set_leds(ColorFlowAnimation, color=self.yellow)

    def set_disabled_lights(self):
        self.set_leds(SolidColor, color=self.white)

    def set_teleop_lights(self):
        if is_alliance_hub_active():
            self.set_leds(ColorFlowAnimation, color=self.green)
        else:
            self.set_leds(ColorFlowAnimation, color=self.white)

    def execute(self):
        self.candle.set_control(self.desired_command)
