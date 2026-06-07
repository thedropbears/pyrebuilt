from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from magicbot import feedback, tunable
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

# fmt: off
DISTANCE_LOOKUP = np.array([2.5,   3.0,   3.5,   4.0,   4.5,   5.0], dtype=float)
SPEED_LOOKUP =    np.array([70.0,  74.0,  79.0,  88.0,  98.0, 112.0], dtype=float)
TIME_LOOKUP =     np.array([1.003, 1.103, 1.147, 1.294, 1.348, 1.450], dtype=float)
# fmt: on


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
    hopper_surface_speed: units.meters_per_second
    name: str

    def speed_for(self, distance: float) -> float:
        return np.interp(distance, self.dist, self.speed)

    def flight_time_for(self, distance: float) -> float:
        return np.interp(distance, self.dist, self.flight_time)

    def rps_to_mps(self, speed_rps: float) -> float:
        return np.interp(speed_rps, self.speed, self.dist / self.flight_time)

    def mps_to_rps(self, speed_mps: float) -> float:
        return np.interp(speed_mps, self.dist / self.flight_time, self.speed)

    def velocity_to_effective_distance(
        self, velocity: units.meters_per_second
    ) -> units.meters:

        velocities = self.dist / self.flight_time
        return float(np.interp(velocity, velocities, self.dist))

    def solution_for(
        self, distance: units.meters
    ) -> tuple[units.turns_per_second, units.seconds]:
        return (self.speed_for(distance), self.flight_time_for(distance))


@dataclass
class BallisticsSolution:
    feed_speed: units.meters_per_second
    flywheel_speed: units.turns_per_second
    bearing: units.radians


class BallisticsSolver:
    LATENCY_FACTOR = tunable(0.052)

    def __init__(self):
        self.target_position = Translation2d()
        self.active_table = LookupTable(
            DISTANCE_LOOKUP, SPEED_LOOKUP, TIME_LOOKUP, 10, "Score Table"
        )
        self.distance_to_target = 0.0

    def compute_range_bearing_for(
        self, base_to_goal: Translation2d, base_velocity: Translation2d
    ) -> tuple[units.meters, Rotation2d]:
        distance_to_target = base_to_goal.norm()
        base_to_goal_direction = base_to_goal / distance_to_target
        baseline_rps, baseline_tof = self.active_table.solution_for(distance_to_target)

        baseline_vel = distance_to_target / baseline_tof

        target_velocity = base_to_goal_direction * baseline_vel
        shot_velocity = target_velocity - base_velocity

        required_velocity = shot_velocity.norm()

        turret_angle = shot_velocity.angle()
        effective_distance = self.active_table.velocity_to_effective_distance(
            required_velocity
        )

        return effective_distance, turret_angle

    @feedback
    def final_distance_to_target(self) -> float:
        return self.distance_to_target

    def calculate_shot_vector(
        self,
        relative_target_translation: Translation2d,
        current_velocity: ChassisSpeeds,
    ) -> Translation2d:
        distance_to_shot = relative_target_translation.norm()
        ideal_flywheel_speed = self.active_table.speed_for(distance_to_shot)

        ideal_speed_mps = self.active_table.rps_to_mps(ideal_flywheel_speed)

        target_vector = relative_target_translation / distance_to_shot * ideal_speed_mps

        robot_velocity = Translation2d(current_velocity.vx, current_velocity.vy)
        shot_vector = target_vector - robot_velocity

        return shot_vector

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
        self.distance_to_target = to_goal.norm()

        effective_distance, absolute_bearing = self.compute_range_bearing_for(
            to_goal, initial_velocity
        )
        current_rotation = initial_pose.rotation()
        to_goal = target_position - future_position
        required_rpm = self.active_table.speed_for(effective_distance)
        return BallisticsSolution(
            self.active_table.hopper_surface_speed,
            required_rpm,
            (absolute_bearing - current_rotation).radians(),
        )

    def execute(self):
        pass
