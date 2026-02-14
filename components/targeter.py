from wpimath.geometry import Translation2d

from components.chassis import ChassisComponent
from utilities.game import (
    alliance_hub_pos,
    behind_alliance_hub_pos,
    is_in_alliance_zone,
    is_red,
)


class Targeter:
    chassis: ChassisComponent

    def __init__(self) -> None:
        self.target = Translation2d()

    def get_target(self) -> Translation2d:
        return self.target

    def execute(self) -> None:
        if is_in_alliance_zone(self.chassis.get_pose().translation()):
            self.target = alliance_hub_pos(is_red())
        else:
            self.target = behind_alliance_hub_pos(is_red())
