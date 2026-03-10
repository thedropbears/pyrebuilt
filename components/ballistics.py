import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from magicbot import feedback, tunable, will_reset_to
from wpimath import units
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from components.shooter import ShooterComponent
from components.turret import TurretComponent
from utilities.game import is_in_transition_zone

# fmt: off
DISTANCE_LOOKUP_30 = np.array([1.5, 2.0, 2.5,   3.0,   3.5,   4.0,   4.5,   5.0,   5.5], dtype=float)
SPEED_LOOKUP_30 =    np.array([1.5, 2.0, 80.0,  84.0,  88.0,  90.0,  101.0, 112.0, 120.0], dtype=float)
TIME_LOOKUP_30 =     np.array([1.5, 2.0, 0.871, 1.004, 1.041, 1.080, 1.155, 1.212, 1.297], dtype=float)

DISTANCE_LOOKUP_45 = np.array([4.0,   4.5,   5.0,   5.5,   6.0,   6.5,   7.0], dtype=float)
SPEED_LOOKUP_45 =    np.array([80.0,  84.0,  88.0,  90.0,  101.0, 112.0, 120.0], dtype=float)
TIME_LOOKUP_45 =     np.array([0.871, 1.004, 1.041, 1.080, 1.155, 1.212, 1.297], dtype=float)

DISTANCE_LOOKUP_PASS = np.array([6.0,   7.0,   8.0,   9.0], dtype=float)
SPEED_LOOKUP_PASS =    np.array([79.0,  90.0,  101,   130], dtype=float)
TIME_LOOKUP_PASS =     np.array([1.139, 1.293, 1.376, 1.431], dtype=float)
# fmt: on

type ForcedSolution = tuple[units.turns_per_second, units.radians, units.radians]


@dataclass
class LookupTable:
    dist: npt.NDArray[np.float64]
    """The distance (m) interpolation table."""
    speed: npt.NDArray[np.float64]
    """The target flywheel speed (turn/s) interpolation table."""
    flight_time: npt.NDArray[np.float64]
    """The time the ball is in flight ie before it reaches its target.
    Keep in mind that the long and short shots have a different target
    so are not swappable without some redesign. we are commited to the
    "goal shot" and "pass shot" paradigm

    The time stamp for a goal shot should be once its inside the goal
    The flight time for a "pass shot" should be once its hit the ground
    """
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

    LEAD_SHOT_ITERATIONS = tunable(2)

    MINIMUM_LEAD_DISTANCE = 2.0

    TURRET_OFFSET = Translation2d(
        -0.170, -0.137
    )  # assuming intake is front... verify before merge

    def __init__(self) -> None:
        self.target_position = Translation2d()
        self.tables = (
            LookupTable(
                DISTANCE_LOOKUP_30,
                SPEED_LOOKUP_30,
                TIME_LOOKUP_30,
                math.radians(25),
                "Score Table 30",
            ),
            LookupTable(
                DISTANCE_LOOKUP_45,
                SPEED_LOOKUP_45,
                TIME_LOOKUP_45,
                math.radians(45),
                "Score Table 45",
            ),
            LookupTable(
                DISTANCE_LOOKUP_PASS,
                SPEED_LOOKUP_PASS,
                TIME_LOOKUP_PASS,
                math.radians(54),
                "Pass Table",
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

    def solve_moving_shot(self, current_pose: Pose2d, current_velocity: ChassisSpeeds):
        """pretty how you going but we do what we can. This worked well enough
        for us in Rapid React. We are basically iterating N times assuming
        constant velocity to determine where the equivilent static shot would
        be from"""

        predicted_translation = current_pose.translation()

        flight_time = 0.0

        for _ in range(self.LEAD_SHOT_ITERATIONS):
            distance = predicted_translation.distance(self.target_position)

            if distance < BallisticsComponent.MINIMUM_LEAD_DISTANCE:
                break

            flight_time = np.interp(
                distance,
                self.active_table.dist,
                self.active_table.flight_time,
            )

            current_twist = current_velocity.toTwist2d(flight_time)
            predicted_translation = current_pose.exp(current_twist).translation()

        return predicted_translation

    def execute(self) -> None:
        current_pose = self.chassis.get_pose()
        current_rotation = current_pose.rotation()

        current_velocity = ChassisSpeeds.fromRobotRelativeSpeeds(
            self.chassis.get_velocity(), current_rotation
        )

        predicted_position = (
            self.solve_moving_shot(current_pose, current_velocity) + self.TURRET_OFFSET
        )

        distance_to_target = predicted_position.distance(self.target_position)
        angle_to_target = (self.target_position - predicted_position).angle()

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

        if is_in_transition_zone(current_pose.translation()):
            self.shooter.pitch_to(self.shooter.MIN_HOOD_ANGLE)
        else:
            self.shooter.pitch_to(target_hood_angle)
        self.turret.slew_to(target_turret_angle)
