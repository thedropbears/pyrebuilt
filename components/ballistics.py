from math import atan2

import numpy as np
from magicbot import will_reset_to
from wpimath import units
from wpimath.geometry import Pose2d, Translation3d, Twist2d

from components.shooter import ShooterComponent, rotations_per_second
from components.turret import TurretComponent
from utilities.functions import constrain_angle


class BallisticsComponent:
    shooter: ShooterComponent
    turret: TurretComponent

    # TODO Define lookup table for use
    DISTANCE_LOOKUP = [1.0, 2.0, 3.0, 4.0, 5.0]  # TODO Tune these values
    SPEED_LOOKUP = [22.0, 33.0, 44.0, 55.0, 66.0]  # TODO Tune these values
    ANGLE_LOOKUP = [80.0, 75.0, 70.0, 65.0, 60.0]  # TODO Tune these values

    use_ballistics = will_reset_to(True)

    should_energise_flywheels = will_reset_to(False)

    def __init__(self) -> None:
        self.current_pose = Pose2d()
        self.current_twist = Twist2d()
        self.target_position = Translation3d()

    def setup(self) -> None:
        self.target_flywheel_speed = 0.0
        self.target_turret_angle = self.turret.current_angle()
        self.target_hood_angle = self.shooter.hood_angle()

    def energise_flywheels(self) -> None:
        # assuming that we dont want to have the flywheel spun up all the time,
        # but the hood and turret should always run
        self.should_energise_flywheels = True

    def calculate_for(
        self,
        target_position: Translation3d,
        current_pose: Pose2d,
        current_twist: Twist2d,
    ) -> None:
        # like components with hardware attached we dont want to perform the
        # calculation here. Just set the required vars and wait for execute.
        self.current_pose = current_pose
        self.current_twist = current_twist
        self.target_position = target_position

    def force_solution(
        self,
        desired_flywheel_speed: rotations_per_second,
        desired_turret_angle: units.radians,
        desired_hood_angle: units.radians,
    ) -> None:
        self.target_flywheel_speed = desired_flywheel_speed
        self.target_turret_angle = desired_turret_angle
        self.target_hood_angle = desired_hood_angle
        self.use_ballistics = False

    def shoot(self) -> None:
        self.shooter.shoot()

    def execute(self) -> None:
        current_position = self.current_pose.translation()

        distance_to_target = current_position.distance(
            self.target_position.toTranslation2d()
        )
        angle_to_target = atan2(
            self.target_position.y - current_position.y,
            self.target_position.x - current_position.x,
        )

        if self.use_ballistics:
            self.target_turret_angle = constrain_angle(
                angle_to_target - self.current_pose.rotation().radians()
            )
            self.target_hood_angle = float(
                np.interp(
                    distance_to_target,
                    self.DISTANCE_LOOKUP,
                    self.ANGLE_LOOKUP,
                )
            )

            self.target_flywheel_speed = float(
                np.interp(
                    distance_to_target,
                    self.DISTANCE_LOOKUP,
                    self.SPEED_LOOKUP,
                )
            )

        if self.should_energise_flywheels:
            self.shooter.set_flywheel(self.target_flywheel_speed)

        self.shooter.pitch_to(self.target_hood_angle)
        self.turret.slew_to(self.target_turret_angle)
