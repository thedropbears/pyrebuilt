from phoenix6.controls import (
    EmptyAnimation,
    RainbowAnimation,
    SolidColor,
    StrobeAnimation,
)
from phoenix6.hardware.candle import CANdle
from phoenix6.signals import RGBWColor

from ids import CandleId


class Colors:
    yellow = RGBWColor(255, 255, 0)
    red = RGBWColor(255, 0, 0)
    blue = RGBWColor(0, 0, 255)
    green = RGBWColor(0, 255, 0)
    purple = RGBWColor(255, 0, 255)
    orange = RGBWColor(255, 105, 0)
    white = RGBWColor(255, 255, 255, 255)


class LEDComponent:
    LED_START = 0
    LED_END = 255

    FLASHING_SPEED = 10

    def __init__(self):
        self.candle = CANdle(device_id=CandleId.LED)
        self.set_rainbow()

    def _set_static_leds(self, color, specific_led=None):
        if specific_led:
            start_index, end_index = specific_led, specific_led
        else:
            start_index, end_index = self.LED_START, self.LED_END

        self.desired_command = SolidColor(
            led_start_index=start_index, led_end_index=end_index, color=color
        )

    def _set_flashing_leds(self, color, specific_led=None, speed=FLASHING_SPEED):
        if specific_led:
            start_index, end_index = specific_led, specific_led
        else:
            start_index, end_index = self.LED_START, self.LED_END

        self.desired_command = StrobeAnimation(
            led_start_index=start_index,
            led_end_index=end_index,
            color=color,
            frame_rate=speed,
        )

    def set_rainbow(self):
        self.desired_command = RainbowAnimation(
            led_start_index=self.LED_START,
            led_end_index=self.LED_END,
        )

    def set_none(self):
        self.desired_command = EmptyAnimation(0)

    def no_multitag_solution(self):
        self._set_static_leds(Colors.red)

    def no_auto(self):
        self._set_static_leds(StrobeAnimation, Colors.purple)

    def mispositioned_start(self, translation, tol):
        # 7,6,5,4 is top, l to r
        target_leds = []
        blink_speed = 25

        if translation.x < tol:
            target_leds = [7, 6, 0, 1]
            blink_speed = abs(tol / translation.x)
        elif translation.x > tol:
            target_leds = [5, 4, 3, 2]
            blink_speed = abs(tol / translation.x)

        if translation.y < tol:
            target_leds = [0, 1, 2, 3]
            blink_speed = abs(tol / translation.y)
        elif translation.y > tol:
            target_leds = [4, 5, 6, 7]
            blink_speed = abs(tol / translation.y)

        self.set_none()
        for led in target_leds:
            self._set_flashing_leds(Colors.blue, led, blink_speed)

    def turret_close_to_rotation_limit(self):
        self._set_flashing_leds(Colors.orange)

    def turret_at_rotation_limit(self):
        self._set_static_leds(Colors.orange)

    def hopper_jammed(self):
        self._set_static_leds(Colors.red)

    def too_close_to_trench_to_shoot(self):
        self._set_flashing_leds(Colors.blue)

    def driving_faster_than_shoot_speed(self):
        self._set_static_leds(Colors.blue)

    def conducter_state_machine_active(self):
        self._set_static_leds(Colors.purple)

    def conductor_state_machine_tracking(self):
        self._set_static_leds(Colors.green)

    def execute(self):
        self.candle.set_control(self.desired_command)
