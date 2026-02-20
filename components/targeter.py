from magicbot import feedback
from wpilib import Field2d
from wpimath.geometry import Rotation2d, Translation2d

from components.chassis import ChassisComponent
from utilities.game import (
    alliance_hub_pos,
    alliance_shoot_anchor_pos,
    alliance_shoot_line,
    behind_alliance_hub_pos,
    is_in_alliance_zone,
    is_in_neutral_zone,
    is_in_transition_zone,
    is_red,
)


class Targeter:
    chassis: ChassisComponent

    target = Translation2d()

    def __init__(self, field: Field2d) -> None:
        self.target_pos_obj = field.getObject("target_pose")

    @feedback
    def get_target(self) -> Translation2d:
        return self.target

    @feedback
    def get_optimal_target_from_alliance_zone(self) -> Translation2d:
        return alliance_hub_pos(is_red())

    @feedback
    def get_optimal_target_from_transition_zone(self) -> Translation2d:
        return behind_alliance_hub_pos(is_red())

    @feedback
    def get_optimal_target_from_neutral_zone(
        self,
    ) -> Translation2d:  # Code for neutral zone and enemy zone is the same, but this MIGHT change in future, so keep it as seperate functions
        robot_pos = self.chassis.get_pose().translation()

        anchor1_pos = alliance_shoot_anchor_pos(is_red())[0]
        anchor2_pos = alliance_shoot_anchor_pos(is_red())[1]

        shot_x = alliance_shoot_line(is_red())

        shot1_y = robot_pos.y + (
            (anchor1_pos.y - robot_pos.y) / (anchor1_pos.x - robot_pos.x)
        ) * (shot_x - robot_pos.x)

        shot2_y = robot_pos.y + (
            (anchor2_pos.y - robot_pos.y) / (anchor2_pos.x - robot_pos.x)
        ) * (shot_x - robot_pos.x)

        target1 = Translation2d(shot_x, shot1_y)
        target2 = Translation2d(shot_x, shot2_y)

        if robot_pos.distance(target1) <= robot_pos.distance(target2):
            return target1
        else:
            return target2

    @feedback
    def get_optimal_target_from_enemy_zone(
        self,
    ) -> Translation2d:  # Code for neutral zone and enemy zone is the same, but this MIGHT change in future, so keep it as seperate functions
        robot_pos = self.chassis.get_pose().translation()

        anchor1_pos = alliance_shoot_anchor_pos(is_red())[0]
        anchor2_pos = alliance_shoot_anchor_pos(is_red())[1]

        shot_x = alliance_shoot_line(is_red())

        shot1_y = robot_pos.y + (
            (anchor1_pos.y - robot_pos.y) / (anchor1_pos.x - robot_pos.x)
        ) * (shot_x - robot_pos.x)

        shot2_y = robot_pos.y + (
            (anchor2_pos.y - robot_pos.y) / (anchor2_pos.x - robot_pos.x)
        ) * (shot_x - robot_pos.x)

        target1 = Translation2d(shot_x, shot1_y)
        target2 = Translation2d(shot_x, shot2_y)

        if robot_pos.distance(target1) <= robot_pos.distance(target2):
            return target1
        else:
            return target2

    def execute(self) -> None:
        current_pos = self.chassis.get_pose().translation()

        if is_in_alliance_zone(current_pos):
            self.target = self.get_optimal_target_from_alliance_zone()

        elif is_in_transition_zone(current_pos):
            self.target = self.get_optimal_target_from_transition_zone()

        elif is_in_neutral_zone(current_pos):
            self.target = self.get_optimal_target_from_neutral_zone()

        else:
            self.target = self.get_optimal_target_from_enemy_zone()

        self.target_pos_obj.setPose(self.target.x, self.target.y, Rotation2d())
