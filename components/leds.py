from enum import IntEnum, auto

from magicbot import feedback, will_reset_to
from phoenix6.controls import ColorFlowAnimation, RainbowAnimation, SolidColor
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


class States(IntEnum):
    NO_MULTITAG = auto()
    NO_AUTO = auto()
    AUTO_MISALIGNED = auto()
    READY_TO_RUN = auto()
    TELEOPERATED = auto()


class LEDComponent:
    LED_START = 0
    LED_END = 256
    X_SIGNAL_START = 8
    X_SIGNAL_END = 28
    Y_SIGNAL_START = 29
    Y_SIGNAL_END = 50

    POSITION_UPDATE_DISTANCE = units.meters(0.01)
    ALLOWABLE_OFFSET = units.meters(0.01)

    should_update_leds = will_reset_to(False)

    def __init__(self) -> None:
        self.candle = CANdle(CandleId.LED)
        self.position_error = Translation2d()
        self.desired_state = States.NO_MULTITAG

    def setup(self) -> None:
        self.should_update_leds = True

    @feedback
    def get_desired_state(self):
        return self.desired_state

    def _update_led_state(self, state: States) -> None:
        if self.desired_state != state:
            self.desired_state = state
            self.should_update_leds = True

    def ready_to_run(self):
        self._update_led_state(States.READY_TO_RUN)

    def teleoperated(self):
        self._update_led_state(States.TELEOPERATED)

    def no_auto(self):
        self._update_led_state(States.NO_AUTO)

    def no_multitag_solution(self):
        self._update_led_state(States.NO_MULTITAG)

    def mispositioned(self, position_error: Translation2d):
        if not (
            self.position_error - position_error
        ).norm() > self.POSITION_UPDATE_DISTANCE or (
            self.position_error.norm() > self.POSITION_UPDATE_DISTANCE
            and position_error.norm() < self.POSITION_UPDATE_DISTANCE
        ):
            return
        self.position_error = position_error
        self.should_update_leds = True
        self._update_led_state(States.AUTO_MISALIGNED)

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
            animation_speed = 1 / error
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
                        self.position_error.X(),
                        self.X_SIGNAL_START,
                        self.X_SIGNAL_END,
                        0,
                    )
                )
                self.candle.set_control(
                    self._make_auto_alignment_animation_segment(
                        self.position_error.Y(),
                        self.Y_SIGNAL_START,
                        self.Y_SIGNAL_END,
                        1,
                    )
                )

            case States.READY_TO_RUN:
                self.candle.set_control(RainbowAnimation(self.LED_START, self.LED_END))
            case States.TELEOPERATED:
                self.candle.set_control(
                    SolidColor(self.LED_START, self.LED_END, Colors.green)
                )
