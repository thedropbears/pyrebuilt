from phoenix6.controls.rainbow_animation import RainbowAnimation
from phoenix6.hardware.candle import CANdle

from ids import CANdleID


class LEDComponent:
    def __init__(self):
        self.candle = CANdle(CANdleID.LED)
        self.light = RainbowAnimation(led_start_index=0, led_end_index=255)

    def execute(self):
        self.candle.set_control(self.light)
