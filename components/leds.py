from phoenix6.controls.color_flow_animation import ColorFlowAnimation
from phoenix6.controls.rainbow_animation import RainbowAnimation
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor

from ids import CandleId


class LEDComponent:
    rainbow = RainbowAnimation(
        led_start_index=0, led_end_index=7, slot=0, brightness=1.0
    )
    red = ColorFlowAnimation(
        led_start_index=0, led_end_index=7, slot=0, color=RGBWColor(255, 0, 0, 0)
    )
    yellow = ColorFlowAnimation(
        led_start_index=0, led_end_index=7, slot=0, color=RGBWColor(255, 255, 0, 0)
    )

    desired_color = rainbow

    def __init__(self):
        self.candle = CANdle(device_id=CandleId.LED)

    def set_color(self, color):
        self.desired_color = color

    def execute(self):
        self.candle.set_control(self.desired_color)
