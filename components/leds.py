from enum import IntEnum

from magicbot import will_reset_to
from phoenix6.controls import (
    EmptyAnimation,
    RainbowAnimation,
    SolidColor,
    StrobeAnimation,
)
from phoenix6.hardware.candle import CANdle
from phoenix6.signals import RGBWColor
from wpimath.geometry import Translation2d
from wpimath.units import hertz

from ids import CandleId


class Colors:
    yellow = RGBWColor(255, 255, 0)
    red = RGBWColor(255, 0, 0)
    blue = RGBWColor(0, 0, 255)
    green = RGBWColor(0, 255, 0)
    purple = RGBWColor(255, 0, 255)
    orange = RGBWColor(255, 105, 0)
    white = RGBWColor(255, 255, 255, 255)


class StatePriorities(IntEnum):
    HOPPER_JAM = 0
    TOO_CLOSE_TO_TRENCH_TO_SHOOT = 1
    DRIVING_FASTER_THAN_MAX_SHOOT_SPEED = 2
    TURRET_AT_ROTATION_LIMIT = 3
    TURRET_CLOSE_TO_ROTATION_LIMIT = 4
    CONDUCTOR_STATE_MACHINE_ACTIVE = 5
    CONDUCTOR_STATE_MACHINE_TRACKING = 6

    NO_AUTO = 7
    MISPOSITIONED_START = 8
    NO_MULTITAG_VISION_SOLUTION = 9

    IDLE = 10


class LEDComponent:
    LED_START = 0
    LED_END = 255

    FLASHING_SPEED = 10.0

    current_state_priority = will_reset_to(StatePriorities.IDLE)

    def __init__(self):
        self.candle = CANdle(device_id=CandleId.LED)
        self.desired_command: (
            SolidColor | StrobeAnimation | RainbowAnimation | EmptyAnimation
        )

    def setup(self):
        self._set_rainbow(StatePriorities.IDLE)

    def _set_leds(
        self,
        priority: StatePriorities,
        color: RGBWColor,
        is_flashing=False,
        speed: hertz = FLASHING_SPEED,
        specific_led=None,
    ):
        if priority >= self.current_state_priority:
            return

        self.current_state_priority = priority

        if specific_led:
            start_index, end_index = specific_led, specific_led
        else:
            start_index, end_index = self.LED_START, self.LED_END

        if is_flashing:
            self.desired_command = StrobeAnimation(
                led_start_index=start_index,
                led_end_index=end_index,
                color=color,
                frame_rate=speed,
            )

        else:
            self.desired_command = SolidColor(
                led_start_index=start_index, led_end_index=end_index, color=color
            )

    def _set_rainbow(self, priority: StatePriorities):
        if priority >= self.current_state_priority:
            return

        self.current_state_priority = priority

        self.desired_command = RainbowAnimation(
            led_start_index=self.LED_START,
            led_end_index=self.LED_END,
        )

    def _set_none(self):
        self.desired_command = EmptyAnimation(0)

    def no_multitag_solution(self):
        self._set_leds(StatePriorities.NO_MULTITAG_VISION_SOLUTION, Colors.red)

    def no_auto(self):
        self._set_leds(StatePriorities.NO_AUTO, Colors.purple)

    def mispositioned_start(self, translation: Translation2d, tol: float):
        # 7,6,5,4 is top, l to r
        target_leds = []
        blink_speed = 25.0

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

        self._set_none()
        for led in target_leds:
            self._set_leds(
                StatePriorities.MISPOSITIONED_START,
                Colors.blue,
                is_flashing=True,
                speed=blink_speed,
                specific_led=led,
            )

    def turret_close_to_rotation_limit(self):
        self._set_leds(StatePriorities.TURRET_CLOSE_TO_ROTATION_LIMIT, Colors.orange)

    def turret_at_rotation_limit(self):
        self._set_leds(StatePriorities.TURRET_AT_ROTATION_LIMIT, Colors.orange)

    def hopper_jammed(self):
        self._set_leds(StatePriorities.HOPPER_JAM, Colors.red)

    def too_close_to_trench_to_shoot(self):
        self._set_leds(
            StatePriorities.TOO_CLOSE_TO_TRENCH_TO_SHOOT, Colors.blue, is_flashing=True
        )

    def driving_faster_than_shoot_speed(self):
        self._set_leds(StatePriorities.DRIVING_FASTER_THAN_MAX_SHOOT_SPEED, Colors.blue)

    def conducter_state_machine_active(self):
        self._set_leds(StatePriorities.CONDUCTOR_STATE_MACHINE_ACTIVE, Colors.purple)

    def conductor_state_machine_tracking(self):
        self._set_leds(StatePriorities.CONDUCTOR_STATE_MACHINE_TRACKING, Colors.green)

    def idle(self):
        self._set_rainbow(StatePriorities.IDLE)

    def execute(self):
        self.candle.set_control(self.desired_command)
