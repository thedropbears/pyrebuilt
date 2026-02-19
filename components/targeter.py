from magicbot import feedback
from wpimath.geometry import Translation2d

from components.chassis import ChassisComponent
from utilities.game import (
    alliance_hub_pos,
    behind_alliance_hub_pos,
    is_in_alliance_zone,
    is_in_neutral_zone,
    is_red,
)


class Targeter:
    chassis: ChassisComponent

    def __init__(self) -> None:
        self.target = Translation2d()

    @feedback
    def get_target(self) -> Translation2d:
        return self.target

    @feedback
    def get_optimal_target_from_alliance_zone(self) -> Translation2d:
        return alliance_hub_pos(is_red())

    @feedback
    def get_optimal_target_from_neutral_zone(self) -> Translation2d:
        # TODO Fill in the actual logic for this
        return behind_alliance_hub_pos(is_red())

    @feedback
    def get_optimal_target_from_enemy_zone(self) -> Translation2d:
        # TODO Fill in the actual logic for this
        return behind_alliance_hub_pos(is_red())

    def execute(self) -> None:
        current_pos = self.chassis.get_pose().translation()

        if is_in_alliance_zone(current_pos):
            self.target = self.get_optimal_target_from_alliance_zone()

        elif is_in_neutral_zone(current_pos):
            self.target = self.get_optimal_target_from_neutral_zone()

        else:
            self.target = self.get_optimal_target_from_enemy_zone()
