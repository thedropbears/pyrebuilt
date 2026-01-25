from dataclasses import dataclass
from math import atan2
from typing import ClassVar

import numpy as np
import wpiutil.wpistruct
from magicbot import feedback
from wpimath.geometry import Pose2d, Translation3d, Twist2d

from components.shooter import ShooterComponent
from components.turret import TurretComponent


@wpiutil.wpistruct.make_wpistruct
@dataclass
class BallisticsSolution:
    WPIStruct: ClassVar

    speed: float
    angle: float


class BallisticsComponent:
    shooter: ShooterComponent
    turret: TurretComponent
    current_solution: BallisticsSolution

    # TODO setup sensible reset and tunable vars

    chassis_pos = Pose2d().translation()
    target_pos = Translation3d().toTranslation2d()
    chassis_rot = Pose2d().rotation()

    chassis_angle_to_target: float
    shooter_angle_to_target: float
    chassis_dist_to_target: float

    desired_flywheel_speed: float
    desired_hood_angle: float

    # TODO Define lookup table for use

    DISTANCE_LOOKUP = [1.0, 2.0, 3.0, 4.0, 5.0]  # TODO Tune these values

    SPEED_LOOKUP = [22.0, 33.0, 44.0, 55.0, 66.0]  # TODO Tune these values

    ANGLE_LOOKUP = [80.0, 75.0, 70.0, 65.0, 60.0]  # TODO Tune these values

    def __init__(self) -> None:
        pass

    @feedback
    def get_chassis_angle_to_target(self):
        return self.chassis_angle_to_target

    @feedback
    def get_shooter_angle_to_target(self):
        return self.shooter_angle_to_target

    @feedback
    def get_chassis_dist_to_target(self):
        return self.chassis_dist_to_target

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

        self.chassis_pos = current_pose.translation()
        self.chassis_rot = current_pose.rotation()
        self.target_pos = target_position.toTranslation2d()
        self.chassis_dist_to_target = self.chassis_pos.distance(self.target_pos)

    def execute(self) -> None:
        # perform turret calculations
        self.chassis_angle_to_target = atan2(
            self.target_pos.y - self.chassis_pos.y,
            self.target_pos.x - self.chassis_pos.x,
        )

        self.shooter_angle_to_target = (
            self.chassis_angle_to_target - self.chassis_rot.radians()
        )

        # TODO account for chassis speeds when calculating the turret angle

        # dispatch turret command

        # perform shooter calculations
        self.desired_hood_angle = float(
            np.interp(
                self.chassis_dist_to_target,
                self.DISTANCE_LOOKUP,
                self.ANGLE_LOOKUP,
            )
        )

        self.desired_flywheel_speed = float(
            np.interp(
                self.chassis_dist_to_target,
                self.DISTANCE_LOOKUP,
                self.SPEED_LOOKUP,
            )
        )

        self.current_solution.speed = self.desired_flywheel_speed
        self.current_solution.angle = self.desired_hood_angle
        # dispatch shooter commands
        pass
