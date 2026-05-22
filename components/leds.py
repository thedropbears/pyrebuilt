from enum import IntEnum

from magicbot import feedback
from phoenix6.controls import RainbowAnimation
from phoenix6.hardware.candle import CANdle
from phoenix6.signals import RGBWColor

from ids import CandleId


class Colors:
    red = RGBWColor(255, 0, 0)
    green = RGBWColor(0, 255, 0)


class States(IntEnum):
    TEMP = 1
    IDLE = 2


class LEDComponent:
    LED_START = 0
    LED_END = 255

    desired_state = States.IDLE
    current_state = States.TEMP

    desired_command = RainbowAnimation(LED_START, LED_END)

    def __init__(self) -> None:
        self.candle = CANdle(CandleId.LED)

    @feedback
    def get_current_state(self):
        return self.current_state

    @feedback
    def get_desired_state(self):
        return self.desired_state

    def execute(self) -> None:
        if self.desired_state == self.current_state:
            return

        self.current_state = self.desired_state
        self.candle.set_control(self.desired_command)
