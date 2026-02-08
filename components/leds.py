from phoenix6.controls.color_flow_animation import ColorFlowAnimation
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor

from ids import CandleId


class LEDComponent:
    yellow = RGBWColor(255, 255, 0, 0)
    red = RGBWColor(255, 0, 0, 0)
    blue = RGBWColor(0, 0, 255, 0)
    BRIGHTNESS = 1.0

    def __init__(self):
        self.candle = CANdle(device_id=CandleId.LED)
        self.desired_command = ColorFlowAnimation(
            led_start_index=0,
            led_end_index=7,
            color=LEDComponent.red,
            slot=0,
        )

    def _set_flow_colour(self, color: RGBWColor):
        self.desired_command = ColorFlowAnimation(
            led_start_index=0,
            led_end_index=7,
            color=color,
            slot=0,
        )

    def set_test_lights(self):
        self._set_flow_colour(LEDComponent.yellow)

    def set_disabled_lights(self):
        self._set_flow_colour(LEDComponent.red)

    def set_teleop_lights(self):
        self._set_flow_colour(LEDComponent.blue)

    def execute(self):
        self.candle.set_control(self.desired_command)
