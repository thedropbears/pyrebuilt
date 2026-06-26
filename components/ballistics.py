from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from magicbot import feedback, tunable
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

# fmt: off
DISTANCE_LOOKUP = np.array([1.75,  2.0,   2.5,   3.0,   3.5,   4.0,   4.5,  5.0,  5.5], dtype=float)
SPEED_LOOKUP    = np.array([63.0,  67.0,  73.0,  77.0,  83.0,  87.0,  92.0, 99.0, 106.0], dtype=float)
TIME_LOOKUP     = np.array([0.765, 0.92, 1.065, 1.20, 1.31, 1.39, 1.475, 1.56, 2.01], dtype=float)
MUZZLE_VELOCITY_LOOKUP =  SPEED_LOOKUP * (np.pi * units.inchesToMeters(3))  # rps * pi * diameter
# fmt: on

HOPPER_SURFACE_SPEED: units.meters_per_second = 10


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
    muzzle_velocity: npt.NDArray[np.float64]

    def speed_for(self, distance: float) -> float:
        return np.interp(distance, self.dist, self.speed)

    def flight_time_for(self, distance: float) -> float:
        return np.interp(distance, self.dist, self.flight_time)

    def rps_to_mps(self, speed_rps: float) -> float:
        return np.interp(speed_rps, self.speed, self.muzzle_velocity)

    def mps_to_rps(self, speed_mps: float) -> float:
        return np.interp(speed_mps, self.muzzle_velocity, self.speed)

    def velocity_to_effective_distance(
        self, velocity: units.meters_per_second
    ) -> units.meters:
        return float(np.interp(velocity, self.muzzle_velocity, self.dist))

    def solution_for(
        self, distance: units.meters
    ) -> tuple[units.turns_per_second, units.seconds]:
        return (self.speed_for(distance), self.flight_time_for(distance))


TABLE = LookupTable(DISTANCE_LOOKUP, SPEED_LOOKUP, TIME_LOOKUP, MUZZLE_VELOCITY_LOOKUP)


@dataclass
class BallisticsSolution:
    feed_speed: units.meters_per_second
    flywheel_speed: units.turns_per_second
    bearing: units.radians


class BallisticsSolver:
    LATENCY_FACTOR = tunable(0.052)

    def __init__(self):
        self.target_position = Translation2d()
        self.distance_to_target = 0.0
        self.sent_rps = 0.0

    def compute_range_bearing_for(
        self, base_to_goal: Translation2d, base_velocity: Translation2d
    ) -> tuple[units.meters, Rotation2d]:
        distance_to_target = base_to_goal.norm()
        base_to_goal_direction = base_to_goal / distance_to_target
        baseline_rps, _ = TABLE.solution_for(distance_to_target)

        baseline_vel = TABLE.rps_to_mps(baseline_rps)

        target_velocity = base_to_goal_direction * baseline_vel
        shot_velocity = target_velocity - base_velocity

        required_velocity = shot_velocity.norm()

        turret_angle = shot_velocity.angle()
        effective_distance = TABLE.velocity_to_effective_distance(required_velocity)

        return effective_distance, turret_angle

    @feedback
    def raw_distance_to_target(self) -> float:
        return self.distance_to_target

    def calculate_shot_vector(
        self,
        relative_target_translation: Translation2d,
        current_velocity: ChassisSpeeds,
    ) -> Translation2d:
        distance_to_shot = relative_target_translation.norm()
        ideal_flywheel_speed = TABLE.speed_for(distance_to_shot)

        ideal_speed_mps = TABLE.rps_to_mps(ideal_flywheel_speed)

        target_vector = relative_target_translation / distance_to_shot * ideal_speed_mps

        robot_velocity = Translation2d(current_velocity.vx, current_velocity.vy)
        shot_vector = target_vector - robot_velocity

        return shot_vector

    @feedback
    def sent_flywheel_speed(self):
        return self.sent_rps

    @feedback
    def interpolated_flywheel_speed(self) -> float:
        return TABLE.speed_for(self.distance_to_target)

    def solve_for(
        self,
        initial_pose: Pose2d,
        initial_velocity: Translation2d,
        target_position: Translation2d,
    ):
        # https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
        # See heading full integration example
        future_position = (
            initial_pose.translation() + initial_velocity * self.LATENCY_FACTOR
        )
        to_goal = target_position - future_position

        effective_distance, absolute_bearing = self.compute_range_bearing_for(
            to_goal, initial_velocity
        )
        self.distance_to_target = effective_distance
        current_rotation = initial_pose.rotation()
        self.sent_rps = TABLE.speed_for(effective_distance)
        return BallisticsSolution(
            HOPPER_SURFACE_SPEED,
            self.sent_rps,
            (absolute_bearing - current_rotation).radians(),
        )

    def execute(self):
        pass
