import math

from phoenix6.controls.color_flow_animation import ColorFlowAnimation
from phoenix6.hardware.candle import CANdle
from phoenix6.signals.rgbw_color import RGBWColor

from components.ballistics import BallisticsComponent, LookupTable
from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from ids import CandleId
from utilities.game import is_alliance_hub_active


class LEDComponent:
    climber: ClimberComponent
    ballistics: BallisticsComponent
    lookup: LookupTable
    chassis: ChassisComponent
    yellow = RGBWColor(255, 255, 0, 0)
    red = RGBWColor(255, 0, 0, 0)
    blue = RGBWColor(0, 0, 255, 0)
    green = RGBWColor(0, 255, 0, 0)
    white = RGBWColor(255, 255, 255, 255)
    black = RGBWColor(0, 0, 0, 0)
    orange = RGBWColor(255, 125, 0, 0)
    purple = RGBWColor(255, 0, 255, 0)

    def __init__(self):
        self.candle = CANdle(device_id=CandleId.LED)
        self.desired_command = ColorFlowAnimation(
            led_start_index=0,
            led_end_index=255,
            color=LEDComponent.red,
            slot=0,
        )

    def _set_flow_colour(self, color: RGBWColor):
        self.desired_command = ColorFlowAnimation(
            led_start_index=0,
            led_end_index=255,
            color=color,
            slot=0,
        )

    def set_test_lights(self):
        self._set_flow_colour(LEDComponent.yellow)

    def set_disabled_lights(self):
        self._set_flow_colour(LEDComponent.red)

    def set_teleop_lights(self):
        if is_alliance_hub_active():
            self._set_flow_colour(LEDComponent.green)
        else:
            self._set_flow_colour(LEDComponent.white)

    def set_climber_lights(self):
        if math.isclose(
            self.climber.target_pos, self.climber.EXTENDED_POS, abs_tol=0.005
        ):
            self._set_flow_colour(LEDComponent.black)
        if not math.isclose(
            self.climber.target_pos, self.climber.EXTENDED_POS, abs_tol=0.005
        ) and not math.isclose(
            self.climber.target_pos, self.climber.RETRACTED_POS, abs_tol=0.005
        ):
            self._set_flow_colour(LEDComponent.orange)
        if self.climber.at_tower_front_hook() or self.climber.at_tower_back_hook():
            self._set_flow_colour(LEDComponent.purple)

    def execute(self):
        self.candle.set_control(self.desired_command)
