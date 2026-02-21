import numpy as np
from magicbot import will_reset_to
from wpimath import units
from wpimath.geometry import Translation2d

from components.chassis import ChassisComponent
from components.shooter import ShooterComponent, rotations_per_second
from components.turret import TurretComponent


class BallisticsComponent:
    chassis: ChassisComponent
    shooter: ShooterComponent
    turret: TurretComponent

    # TODO: Tune lookup tables for use
    DISTANCE_LOOKUP_30 = np.array([1.0, 1.5, 2.0, 2.5, 3.0], dtype=float)
    SPEED_LOOKUP_30 = np.array([22.0, 33.0, 44.0, 55.0, 66.0], dtype=float)
    DISTANCE_LOOKUP_45 = np.array([2.5, 3.0, 3.5, 4.0, 4.5], dtype=float)
    SPEED_LOOKUP_45 = np.array([44.0, 55.0, 66.0, 77.0, 88.0], dtype=float)
    DISTANCE_LOOKUP_60 = np.array([4.0, 4.5, 5.0, 5.5, 6.0, 6.5], dtype=float)
    SPEED_LOOKUP_60 = np.array([66.0, 77.0, 87.0, 88.0, 89.0, 90.0], dtype=float)

    use_ballistics = will_reset_to(True)
    should_energise_flywheels = will_reset_to(False)

    def __init__(self) -> None:
        self.target_position = Translation2d()
        self.target_flywheel_speed = 0.0

    def setup(self) -> None:
        self.target_turret_angle = self.turret.get_current_angle()
        self.target_hood_angle = self.shooter.get_hood_angle()

    def energise_flywheels(self) -> None:
        # assuming that we dont want to have the flywheel spun up all the time,
        # but the hood and turret should always run
        self.should_energise_flywheels = True

    def solve_for(self, target_position: Translation2d) -> None:
        # like components with hardware attached we dont want to perform the
        # calculation here. Just set the required vars and wait for execute.
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

    def execute(self) -> None:
        current_pose = self.chassis.get_pose()

        current_position = current_pose.translation()
        current_rotation = current_pose.rotation()

        distance_to_target = current_position.distance(self.target_position)
        angle_to_target = (self.target_position - current_position).angle()

        if self.use_ballistics:
            required_turret_angle = angle_to_target - current_rotation
            self.target_turret_angle = required_turret_angle.radians()
            self.target_hood_angle = np.interp(
                distance_to_target,
                self.DISTANCE_LOOKUP,
                self.ANGLE_LOOKUP,
            )
            self.target_flywheel_speed = np.interp(
                distance_to_target,
                self.DISTANCE_LOOKUP,
                self.SPEED_LOOKUP,
            )

        if self.should_energise_flywheels:
            self.shooter.set_flywheel(self.target_flywheel_speed)

        self.shooter.pitch_to(self.target_hood_angle)
        self.turret.slew_to(self.target_turret_angle)
