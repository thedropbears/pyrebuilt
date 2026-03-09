import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from magicbot import feedback, will_reset_to
from wpimath import units
from wpimath.geometry import Translation2d

from components.chassis import ChassisComponent
from components.shooter import ShooterComponent
from components.turret import TurretComponent
from utilities.game import is_in_transition_zone

# TODO: Tune lookup tables for use
DISTANCE_LOOKUP_30 = np.array([1.0, 1.5, 2.0, 2.5, 3.0], dtype=float)
SPEED_LOOKUP_30 = np.array([22.0, 33.0, 44.0, 55.0, 66.0], dtype=float)
DISTANCE_LOOKUP_45 = np.array([2.5, 3.0, 3.5, 4.0, 4.5], dtype=float)
SPEED_LOOKUP_45 = np.array([44.0, 55.0, 66.0, 77.0, 88.0], dtype=float)
DISTANCE_LOOKUP_60 = np.array([4.0, 4.5, 5.0, 5.5, 6.0, 6.5], dtype=float)
SPEED_LOOKUP_60 = np.array([66.0, 77.0, 87.0, 88.0, 89.0, 90.0], dtype=float)

type ForcedSolution = tuple[units.turns_per_second, units.radians, units.radians]


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

    forced_solution = will_reset_to[ForcedSolution | None](None)
    should_energise_flywheels = will_reset_to(False)

    def __init__(self) -> None:
        self.target_position = Translation2d()
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
        desired_flywheel_speed: units.turns_per_second,
        desired_turret_bearing: units.radians,
        desired_hood_angle: units.radians,
    ) -> None:
        self.forced_solution = (
            desired_flywheel_speed,
            desired_turret_bearing,
            desired_hood_angle,
        )

    def execute(self) -> None:
        current_pose = self.chassis.get_pose()

        current_position = current_pose.translation()
        current_rotation = current_pose.rotation()

        distance_to_target = current_position.distance(self.target_position)
        angle_to_target = (self.target_position - current_position).angle()

        if self.forced_solution is None:
            target_turret_angle = (angle_to_target - current_rotation).radians()
            # Check if distance is within range of distance table and switch if necessary
            if not self.active_table.is_within_range(distance_to_target):
                for table_pair in self.tables:
                    if table_pair.is_within_range(distance_to_target):
                        self.active_table = table_pair
            target_hood_angle = self.active_table.hood_angle
            target_flywheel_speed: units.turns_per_second = np.interp(
                distance_to_target,
                self.active_table.dist,
                self.active_table.speed,
            )
        else:
            target_flywheel_speed, target_turret_angle, target_hood_angle = (
                self.forced_solution
            )

        if self.should_energise_flywheels:
            self.shooter.set_flywheel(target_flywheel_speed)

        if is_in_transition_zone(current_position):
            self.shooter.pitch_to(self.shooter.MIN_HOOD_ANGLE)
        else:
            self.shooter.pitch_to(target_hood_angle)
        self.turret.slew_to(target_turret_angle)
