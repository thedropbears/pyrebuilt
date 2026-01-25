from math import atan2

from magicbot import feedback
from wpimath.geometry import Pose2d, Translation3d, Twist2d

from components.shooter import ShooterComponent
from components.turret import TurretComponent


class BallisticsComponent:
    shooter: ShooterComponent
    turret: TurretComponent

    # TODO setup sensible reset and tunable vars

    chassis_pos = Pose2d().translation()
    target_pos = Translation3d().toTranslation2d()
    chassis_rot = Pose2d().rotation()

    chassis_angle_to_target = float
    shooter_angle_to_target = float
    chassis_dist_to_target = float

    # TODO Define lookup table for use

    def __init__(self) -> None:
        # TODO Implement this
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
        # TODO Implement this
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
        # TODO Implement this

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

        # dispatch shooter commands
        pass
