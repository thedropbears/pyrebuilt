from math import atan2

import numpy as np
from magicbot import feedback, tunable
from wpimath.geometry import Pose2d, Translation3d, Twist2d

from components.shooter import ShooterComponent
from components.turret import TurretComponent


class BallisticsComponent:
    shooter: ShooterComponent
    turret: TurretComponent

    # TODO setup sensible reset and tunable vars
    current_pos = Pose2d().translation()
    current_rot = Pose2d().rotation()

    target_pos = Translation3d().toTranslation2d()

    angle_to_target: float = 0.0
    oriented_angle_to_target: float = 0.0
    dist_to_target: float = 0.0

    desired_flywheel_speed = tunable(0.0)
    desired_turret_angle = tunable(0.0)
    desired_hood_angle = tunable(0.0)

    # TODO Define lookup table for use
    DISTANCE_LOOKUP = [1.0, 2.0, 3.0, 4.0, 5.0]  # TODO Tune these values
    SPEED_LOOKUP = [22.0, 33.0, 44.0, 55.0, 66.0]  # TODO Tune these values
    ANGLE_LOOKUP = [80.0, 75.0, 70.0, 65.0, 60.0]  # TODO Tune these values

    override_calculations = tunable(False)

    def __init__(self) -> None:
        pass

    @feedback
    def get_angle_to_target(self) -> float:
        return self.angle_to_target

    @feedback
    def get_oriented_angle_to_target(self) -> float:
        return self.oriented_angle_to_target

    @feedback
    def get_dist_to_target(self) -> float:
        return self.dist_to_target

    def energise_flywheels(self) -> None:
        # assuming that we dont want to have the flywheel spun up all the time,
        # but the hood and turret should always run
        pass

    def calculate_for(
        self,
        target_pos: Translation3d,
        current_pose: Pose2d,
        current_twist: Twist2d,
    ) -> None:
        
        if self.override_calculations: # cancel updating calculations if we manually set these values
            return
        
        # like components with hardware attached we dont want to perform the
        # calculation here. Just set the required vars and wait for execute.

        self.current_pose = current_pose
        self.current_twist = current_twist
        self.target_pos = target_pos

        current_pos = self.current_pose.translation()

        self.distance_to_target = current_pos.distance(
            self.target_pos.toTranslation2d()
        )
        self.angle_to_target = atan2(
            self.target_pos.y - current_pos.y,
            self.target_pos.x - current_pos.x,
        )

        self.oriented_angle_to_target = (
            self.angle_to_target - self.current_pose.rotation().radians()
        )

    def execute(self) -> None:
        if not self.override_calculations:
            self.desired_turret_angle = self.oriented_angle_to_target

            self.desired_hood_angle = float(
                np.interp(
                    self.distance_to_target,
                    self.DISTANCE_LOOKUP,
                    self.ANGLE_LOOKUP,
                )
            )

            self.desired_flywheel_speed = float(
                np.interp(
                    self.distance_to_target,
                    self.DISTANCE_LOOKUP,
                    self.SPEED_LOOKUP,
                )
            )

        # dispatch shooter commands
        pass
