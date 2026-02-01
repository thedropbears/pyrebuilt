from phoenix6.controls import RainbowAnimation
from phoenix6.hardware import CANdle

from ids import CandleId


class LEDComponent:
    def __init__(self):
        self.candle = CANdle(device_id=CandleId.LED)
        self.light = RainbowAnimation(
            led_start_index=0, led_end_index=255, slot=0, brightness=1.0
        )

    def execute(self):
        self.candle.set_control(self.light)
