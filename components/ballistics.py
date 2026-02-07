from math import atan2

import numpy as np
from magicbot import feedback
from wpimath.geometry import Pose2d, Translation3d, Twist2d

from components.shooter import ShooterComponent
from components.turret import TurretComponent


class BallisticsComponent:
    shooter: ShooterComponent
    turret: TurretComponent

    # TODO setup sensible reset and tunable vars
    future_start_pos = Pose2d().translation()
    future_start_rot = Pose2d().rotation()

    target_pos = Translation3d().toTranslation2d()

    start_pos_angle_to_target: float = 0.0
    shooter_angle_to_target: float = 0.0
    start_pos_dist_to_target: float = 0.0

    desired_flywheel_speed: float = 0.0
    desired_hood_angle: float = 0.0

    # TODO Define lookup table for use

    DISTANCE_LOOKUP = [1.0, 2.0, 3.0, 4.0, 5.0]  # TODO Tune these values

    SPEED_LOOKUP = [22.0, 33.0, 44.0, 55.0, 66.0]  # TODO Tune these values

    ANGLE_LOOKUP = [80.0, 75.0, 70.0, 65.0, 60.0]  # TODO Tune these values

    def __init__(self) -> None:
        pass

    @feedback
    def get_chassis_angle_to_target(self) -> float:
        return self.start_pos_angle_to_target

    @feedback
    def get_shooter_angle_to_target(self) -> float:
        return self.shooter_angle_to_target

    @feedback
    def get_chassis_dist_to_target(self) -> float:
        return self.start_pos_dist_to_target

    def energise_flywheels(self) -> None:
        # assuming that we dont want to have the flywheel spun up all the time,
        # but the hood and turret should always run
        pass

    def calculate_for(
        self,
        target_position: Translation3d,
        current_pose: Pose2d,
        current_twist: Twist2d,
    ) -> None:
        # like components with hardware attached we dont want to perform the
        # calculation here. Just set the required vars and wait for execute.

        self.future_start_pos = current_pose.exp(current_twist).translation()
        self.future_start_rot = current_pose.exp(current_twist).rotation()
        self.target_pos = target_position.toTranslation2d()

        self.start_pos_dist_to_target = self.future_start_pos.distance(self.target_pos)

    def execute(self) -> None:
        # perform turret calculations
        self.start_pos_angle_to_target = atan2(
            self.target_pos.y - self.future_start_pos.y,
            self.target_pos.x - self.future_start_pos.x,
        )

        self.shooter_angle_to_target = (
            self.start_pos_angle_to_target - self.future_start_rot.radians()
        )

        # dispatch turret command

        # perform shooter calculations
        self.desired_hood_angle = float(
            np.interp(
                self.start_pos_dist_to_target,
                self.DISTANCE_LOOKUP,
                self.ANGLE_LOOKUP,
            )
        )

        self.desired_flywheel_speed = float(
            np.interp(
                self.start_pos_dist_to_target,
                self.DISTANCE_LOOKUP,
                self.SPEED_LOOKUP,
            )
        )

        # dispatch shooter commands
        pass
