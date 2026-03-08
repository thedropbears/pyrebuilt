from magicbot import feedback
from wpilib import Field2d
from wpimath.geometry import Rotation2d, Translation2d

from components.chassis import ChassisComponent
from utilities.game import (
    alliance_hub_pos,
    alliance_shoot_anchor_pos,
    alliance_shoot_line,
    alliance_trench_shoot_pos,
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
    ) -> Translation2d:
        robot_pos = self.chassis.get_pose().translation()

        anchor_positions = alliance_shoot_anchor_pos(is_red())
        shot_x = alliance_shoot_line(is_red())

        close_anchor = (
            anchor_positions[0]
            if robot_pos.distance(anchor_positions[0])
            <= robot_pos.distance(anchor_positions[1])
            else anchor_positions[1]
        )

        target_y = robot_pos.y + (
            (close_anchor.y - robot_pos.y) / (close_anchor.x - robot_pos.x)
        ) * (shot_x - robot_pos.x)

        return Translation2d(shot_x, target_y)

    @feedback
    def get_optimal_target_from_enemy_zone(
        self,
    ) -> Translation2d:
        robot_pos = self.chassis.get_pose().translation()
        shot_positions = alliance_trench_shoot_pos(is_red())
        close_pos = (
            shot_positions[0]
            if robot_pos.distance(shot_positions[0])
            <= robot_pos.distance(shot_positions[1])
            else shot_positions[1]
        )
        return close_pos

    def get_distance_to_target(self):
        return (
            self.chassis.get_pose()
            .translation()
            .distance(self.target_pos_obj.Translation2d)
        )

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
