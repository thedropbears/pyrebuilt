from enum import IntEnum

from magicbot import will_reset_to
from phoenix6.controls import RainbowAnimation, SolidColor
from phoenix6.hardware.candle import CANdle
from phoenix6.signals import RGBWColor

from ids import CandleId


class Colors:
    red = RGBWColor(255, 0, 0)
    green = RGBWColor(0, 255, 0)


class States(IntEnum):
    HOOD_RETRACTED = 0
    HOOD_NOT_RETRACTED = 1

    IDLE = 2


class LEDComponent:
    LED_START = 0
    LED_END = 255

    desired_state = will_reset_to(States.IDLE)
    current_state = States.IDLE

    def __init__(self) -> None:
        self.candle = CANdle(CandleId.LED)

    def _set_static(
        self, color: RGBWColor, start_index=LED_START, end_index=LED_END
    ) -> SolidColor:
        return SolidColor(start_index, end_index, color)

    def _set_rainbow(
        self, start_index=LED_START, end_index=LED_END
    ) -> RainbowAnimation:
        return RainbowAnimation(start_index, end_index)

    def _get_desired_command(self, state: States) -> SolidColor | RainbowAnimation:
        if state == States.HOOD_RETRACTED:
            return self._set_static(Colors.green)
        if state == States.HOOD_NOT_RETRACTED:
            return self._set_static(Colors.red)
        if state == States.IDLE:
            return self._set_rainbow()

    def hood_is_retracted(self) -> None:
        self.desired_state = States.HOOD_RETRACTED

    def hood_is_not_retracted(self) -> None:
        self.desired_state = States.HOOD_NOT_RETRACTED
        self.desired_command = self._set_static(Colors.red)

    def execute(self) -> None:
        if self.desired_state == self.current_state:
            return

        self.current_state = self.desired_state
        desired_command = self._get_desired_command(self.current_state)

        self.candle.set_control(desired_command)
