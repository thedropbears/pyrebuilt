import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from magicbot import feedback, will_reset_to
from wpimath import units
from wpimath.geometry import Rotation2d, Translation2d

from components.chassis import ChassisComponent
from components.shooter import ShooterComponent, rotations_per_second
from components.turret import TurretComponent

# TODO: Tune lookup tables for use
DISTANCE_LOOKUP_30 = np.array([1.0, 1.5, 2.0, 2.5, 3.0], dtype=float)
SPEED_LOOKUP_30 = np.array([22.0, 33.0, 44.0, 55.0, 66.0], dtype=float)
DISTANCE_LOOKUP_45 = np.array([2.5, 3.0, 3.5, 4.0, 4.5], dtype=float)
SPEED_LOOKUP_45 = np.array([44.0, 55.0, 66.0, 77.0, 88.0], dtype=float)
DISTANCE_LOOKUP_60 = np.array([4.0, 4.5, 5.0, 5.5, 6.0, 6.5], dtype=float)
SPEED_LOOKUP_60 = np.array([66.0, 77.0, 87.0, 88.0, 89.0, 90.0], dtype=float)


@dataclass
class LookupTable:
    dist: npt.NDArray[np.float64]
    """The distance (m) interpolation table."""
    speed: npt.NDArray[np.float64]
    """The target flywheel speed (turn/s) interpolation table."""
    hood_angle: units.radians
    name: str

    def is_within_range(self, distance: units.meters) -> bool:
        return self.dist.min() < distance < self.dist.max()


class BallisticsComponent:
    chassis: ChassisComponent
    shooter: ShooterComponent
    turret: TurretComponent

    use_ballistics = will_reset_to(True)
    should_energise_flywheels = will_reset_to(False)

    def __init__(self) -> None:
        self.target_position = Translation2d()

        # It is safe to setup these values as 0, as there is no way for the components to be set to these default values
        # i.e. they will be overwritten before reaching the component methods
        self.target_flywheel_speed = 0.0
        self.target_hood_angle = 0.0
        self.target_turret_bearing = Rotation2d()

        self.tables = (
            LookupTable(
                DISTANCE_LOOKUP_30, SPEED_LOOKUP_30, math.radians(30), "30 degree table"
            ),
            LookupTable(
                DISTANCE_LOOKUP_45, SPEED_LOOKUP_45, math.radians(45), "45 degree table"
            ),
            LookupTable(
                DISTANCE_LOOKUP_60, SPEED_LOOKUP_60, math.radians(60), "60 degree table"
            ),
        )
        self.active_table = self.tables[0]

    @feedback
    def get_active_table(self) -> str:
        return self.active_table.name

    @feedback
    def get_flywheel_speed(self):
        return self.shooter.target_shooter_rps

    @feedback
    def get_hood_angle(self):
        return self.shooter.target_hood_angle * (180 / math.pi)

    @feedback
    def get_turret_angle(self):
        return self.turret.desired_angle * (180 / math.pi)

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
        desired_turret_bearing: Rotation2d,
        desired_hood_angle: units.radians,
    ) -> None:
        self.target_flywheel_speed = desired_flywheel_speed
        self.target_turret_bearing = desired_turret_bearing
        self.target_hood_angle = desired_hood_angle
        self.use_ballistics = False

    def execute(self) -> None:
        current_turret_pose = self.chassis.get_pose().transformBy(
            self.turret.TURRET_OFFSET_FROM_CENTER
        )

        current_position = current_turret_pose.translation()
        current_rotation = current_turret_pose.rotation()

        distance_to_target = current_position.distance(self.target_position)
        angle_to_target = (self.target_position - current_position).angle()

        if self.use_ballistics:
            target_turret_bearing = angle_to_target - current_rotation
            # Check if distance is within range of distance table and switch if necessary
            if not self.active_table.is_within_range(distance_to_target):
                for table_pair in self.tables:
                    if table_pair.is_within_range(distance_to_target):
                        self.active_table = table_pair
            target_hood_angle = self.active_table.hood_angle
            target_flywheel_speed = np.interp(
                distance_to_target,
                self.active_table.dist,
                self.active_table.speed,
            )
        else:
            target_turret_bearing = self.target_turret_bearing
            target_hood_angle = self.target_hood_angle
            target_flywheel_speed = self.target_flywheel_speed

        if self.should_energise_flywheels:
            self.shooter.set_flywheel(target_flywheel_speed)

        self.shooter.pitch_to(target_hood_angle)
        self.turret.slew_to(target_turret_bearing)
