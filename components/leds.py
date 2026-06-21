from enum import IntEnum, auto

from magicbot import feedback, will_reset_to
from phoenix6.controls import (
    ColorFlowAnimation,
    RainbowAnimation,
    SolidColor,
    StrobeAnimation,
)
from phoenix6.hardware.candle import CANdle
from phoenix6.signals import AnimationDirectionValue, RGBWColor
from wpimath import units
from wpimath.geometry import Translation2d

from ids import CandleId


class Colors:
    red = RGBWColor(255, 0, 0)
    green = RGBWColor(0, 255, 0)
    blue = RGBWColor(0, 0, 255)
    purple = RGBWColor(128, 0, 128)
    black = RGBWColor(0, 0, 0)
    orange = RGBWColor(255, 165, 0)


class States(IntEnum):
    CAMERA_DEAD = auto()
    NO_MULTITAG = auto()
    NO_AUTO = auto()
    AUTO_MISALIGNED = auto()
    READY_TO_RUN = auto()
<<<<<<< HEAD
    TELEOP_VISION = auto()
    TELEOP_NO_VISION = auto()
=======
    TELEOPERATED = auto()
    TURRENT_NEARLY_OUT_OF_RANGE = auto()
    TURRET_OUT_OF_RANGE = auto()
    NEARLY_OUT_OF_SHOOTING_RANGE = auto()
    OUT_OF_SHOOTING_RANGE = auto()
>>>>>>> 2eb544d (Created warning led states for turret and shooting range during teleop)


class LEDComponent:
    LED_START = 0
    LED_END = 256
    Y_SIGNAL_START = 8
    Y_SIGNAL_END = 28
    X_SIGNAL_START = 29
    X_SIGNAL_END = 50

    POSITION_UPDATE_DISTANCE: units.meters = 0.05
    ALLOWABLE_OFFSET: units.meters = 0.05

    should_update_leds = will_reset_to(False)

    def __init__(self) -> None:
        self.candle = CANdle(CandleId.LED)
        self.position_error = Translation2d()
        self.desired_state = States.NO_MULTITAG

    def setup(self) -> None:
        self.should_update_leds = True

    @feedback
    def get_desired_state(self):
        return self.desired_state.name

    def _update_led_state(self, state: States) -> None:
        if self.desired_state != state:
            self.desired_state = state
            self.should_update_leds = True

    def ready_to_run(self):
        self._update_led_state(States.READY_TO_RUN)

    def teleop_vision(self):
        self._update_led_state(States.TELEOP_VISION)

    def teleop_no_vision(self):
        self._update_led_state(States.TELEOP_NO_VISION)

    def no_auto(self):
        self._update_led_state(States.NO_AUTO)

    def no_multitag_solution(self):
        self._update_led_state(States.NO_MULTITAG)

    def camera_dead(self) -> None:
        self._update_led_state(States.CAMERA_DEAD)

    def mispositioned(self, position_error: Translation2d):
        self._update_led_state(States.AUTO_MISALIGNED)
        if not (
            (self.position_error - position_error).norm()
            > self.POSITION_UPDATE_DISTANCE
            or (
                (abs(self.position_error.X()) < self.ALLOWABLE_OFFSET)
                != (abs(position_error.X()) < self.ALLOWABLE_OFFSET)
                or (abs(self.position_error.Y()) < self.ALLOWABLE_OFFSET)
                != (abs(position_error.Y()) < self.ALLOWABLE_OFFSET)
            )
            or (
                (abs(self.position_error.X()) == 0.0)
                != (abs(position_error.X()) == 0.0)
                or (abs(self.position_error.Y()) == 0.0)
                != (abs(position_error.Y()) == 0.0)
            )
        ):
            return
        self.position_error = position_error
        self.should_update_leds = True

    def turret_nearly_out_of_range(self):
        self._update_led_state(States.TURRENT_NEARLY_OUT_OF_RANGE)

    def turret_out_of_range(self):
        self._update_led_state(States.TURRET_OUT_OF_RANGE)

    def nearly_out_of_shooting_range(self):
        self._update_led_state(States.NEARLY_OUT_OF_SHOOTING_RANGE)

    def out_of_shooting_range(self):
        self._update_led_state(States.OUT_OF_SHOOTING_RANGE)

    def _make_auto_alignment_animation_segment(
        self, error, start_index, end_index, slot
    ) -> SolidColor | ColorFlowAnimation:
        if abs(error) < self.ALLOWABLE_OFFSET:
            return SolidColor(
                start_index,
                end_index,
                Colors.green,
            )

        else:
            animation_direction = (
                AnimationDirectionValue.FORWARD
                if error > 0
                else AnimationDirectionValue.BACKWARD
            )
            animation_speed = (1 / error) * 10
            return ColorFlowAnimation(
                start_index,
                end_index,
                slot,
                Colors.blue,
                animation_direction,
                animation_speed,
            )

    def execute(self) -> None:
        if not self.should_update_leds:
            return

        self.candle.set_control(SolidColor(self.LED_START, self.LED_END, Colors.black))
        self.candle.clear_all_animations()

        match self.desired_state:
            case States.NO_MULTITAG:
                self.candle.set_control(
                    SolidColor(self.LED_START, self.LED_END, Colors.red)
                )
            case States.NO_AUTO:
                self.candle.set_control(
                    SolidColor(self.LED_START, self.LED_END, Colors.purple)
                )
            case States.AUTO_MISALIGNED:
                self.candle.set_control(
                    self._make_auto_alignment_animation_segment(
                        units.meters(self.position_error.X()),
                        self.X_SIGNAL_START,
                        self.X_SIGNAL_END,
                        0,
                    )
                )
                self.candle.set_control(
                    self._make_auto_alignment_animation_segment(
                        units.meters(self.position_error.Y()),
                        self.Y_SIGNAL_START,
                        self.Y_SIGNAL_END,
                        1,
                    )
                )

            case States.READY_TO_RUN:
                self.candle.set_control(RainbowAnimation(self.LED_START, self.LED_END))
            case States.TELEOP_VISION:
                self.candle.set_control(
                    SolidColor(self.LED_START, self.LED_END, Colors.green)
                )
<<<<<<< HEAD
            case States.TELEOP_NO_VISION:
                self.candle.set_control(
                    SolidColor(self.LED_START, self.LED_END, Colors.red)
                )

            case States.CAMERA_DEAD:
                self.candle.set_control(
                    StrobeAnimation(self.LED_START, self.LED_END, color=Colors.purple)
=======
            case States.TURRENT_NEARLY_OUT_OF_RANGE:
                self.candle.set_control(
                    ColorFlowAnimation(
                        self.LED_START,
                        self.LED_END,
                        0,
                        Colors.orange,
                        AnimationDirectionValue.FORWARD,
                        0.5,
                    )
                )
            case States.TURRET_OUT_OF_RANGE:
                self.candle.set_control(
                    SolidColor(self.LED_START, self.LED_END, Colors.orange)
                )
            case States.NEARLY_OUT_OF_SHOOTING_RANGE:
                self.candle.set_control(
                    ColorFlowAnimation(
                        self.LED_START,
                        self.LED_END,
                        0,
                        Colors.purple,
                        AnimationDirectionValue.FORWARD,
                        0.5,
                    )
                )
            case States.OUT_OF_SHOOTING_RANGE:
                self.candle.set_control(
                    SolidColor(self.LED_START, self.LED_END, Colors.purple)
>>>>>>> 2eb544d (Created warning led states for turret and shooting range during teleop)
                )
