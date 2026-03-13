import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from magicbot import feedback, tunable, will_reset_to
from wpilib import Field2d
from wpimath import units
from wpimath.geometry import Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from components.hopper import HopperComponent
from components.leds import LEDComponent
from components.shooter import ShooterComponent
from components.turret import TurretComponent
from utilities.game import is_in_transition_zone

# fmt: off
DISTANCE_LOOKUP_25 = np.array([2.5,   3.0,   3.5,   4.0,   4.5,   5.0], dtype=float)
SPEED_LOOKUP_25 =    np.array([70.0,  74.0,  79.0,  88.0,  98.0, 112.0], dtype=float)
TIME_LOOKUP_25 =     np.array([1.003, 1.103, 1.147, 1.294, 1.348, 1.450], dtype=float)

DISTANCE_LOOKUP_45 = np.array([4.0,   4.5,   5.0,   5.5,   6.0,   6.5], dtype=float)
SPEED_LOOKUP_45 =    np.array([80.0,  84.0,  88.0,  90.0,  101.0, 112.0], dtype=float)
TIME_LOOKUP_45 =     np.array([0.950, 1.004, 1.041, 1.080, 1.155, 1.212], dtype=float)

DISTANCE_LOOKUP_PASS = np.array([6.0,   7.0,   8.0,   9.0], dtype=float)
SPEED_LOOKUP_PASS =    np.array([79.0,  90.0,  101,   130], dtype=float)
TIME_LOOKUP_PASS =     np.array([1.139, 1.293, 1.376, 1.431], dtype=float)
# fmt: on

type ForcedSolution = tuple[
    units.turns_per_second, units.radians, units.radians, units.meters_per_second
]


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
    hopper_surface_speed: units.meters_per_second
    name: str

    def is_within_range(self, distance: units.meters) -> bool:
        return self.dist.min() < distance < self.dist.max()

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


class BallisticsComponent:
    chassis: ChassisComponent
    shooter: ShooterComponent
    turret: TurretComponent
    leds: LEDComponent
    hopper: HopperComponent

    forced_solution = will_reset_to[ForcedSolution | None](None)
    should_energise_flywheels = will_reset_to(False)
    should_energise_hopper = will_reset_to(False)

    EXTRAPOLATION_TIME_FOR_HOOD_SERVO: units.seconds = 2

    LEAD_SHOT_ITERATIONS = tunable(2)

    MINIMUM_LEAD_DISTANCE = 2.0
    LATENCY_FACTOR = tunable(0.105)  # if shooting far, decrease

    TURRET_OFFSET = Transform2d(Translation2d(0.134, -0.166), Rotation2d())
    MAX_DRIVE_SPEED_FOR_SHOOTING: units.meters_per_second = 2

    is_shooting = tunable(False)

    def __init__(self, field: Field2d) -> None:
        self.target_position = Translation2d()
        self.tables = (
            LookupTable(
                DISTANCE_LOOKUP_25,
                SPEED_LOOKUP_25,
                TIME_LOOKUP_25,
                math.radians(25),
                10,
                "Score Table 25",
            ),
            LookupTable(
                DISTANCE_LOOKUP_45,
                SPEED_LOOKUP_45,
                TIME_LOOKUP_45,
                math.radians(45),
                12,
                "Score Table 45",
            ),
            LookupTable(
                DISTANCE_LOOKUP_PASS,
                SPEED_LOOKUP_PASS,
                TIME_LOOKUP_PASS,
                math.radians(54),
                12,
                "Pass Table",
            ),
        )
        self.active_table = self.tables[0]

        self.distance_to_target = 0.0

        self.turret_pose = field.getObject("Turret Pose")

    @feedback
    def get_active_table(self) -> str:
        return self.active_table.name

    def energise_flywheels(self) -> None:
        # assuming that we dont want to have the flywheel spun up all the time,
        # but the hood and turret should always run
        self.should_energise_flywheels = True

    def feed_shooter(self) -> None:
        self.should_energise_hopper = True

    def shooter_is_ready(self) -> bool:
        return self.shooter.flywheel_is_at_speed()

    def solve_for(self, target_position: Translation2d) -> None:
        # like components with hardware attached we dont want to perform the
        # calculation here. Just set the required vars and wait for execute.
        self.target_position = target_position

    def force_solution(
        self,
        desired_flywheel_speed: units.turns_per_second,
        desired_turret_bearing: units.radians,
        desired_hood_angle: units.radians,
        desired_hopper_surface_speed: units.meters_per_second,
    ) -> None:
        self.forced_solution = (
            desired_flywheel_speed,
            desired_turret_bearing,
            desired_hood_angle,
            desired_hopper_surface_speed,
        )

    def calculate_shot_velocity(
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

    def is_driving_faster_than_max_shoot_speed(self) -> bool:
        chassis_speed = self.chassis.get_velocity()
        current_velocity = math.sqrt(chassis_speed.vx**2 + chassis_speed.vy**2)
        return current_velocity > self.MAX_DRIVE_SPEED_FOR_SHOOTING

    def get_distance_to_target(self) -> units.meters:
        return self.distance_to_target

    def log_shot(self) -> None:
        self.is_shooting = True

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

    def execute(self) -> None:
        chassis_pose = self.chassis.get_pose()
        chassis_rotation = chassis_pose.rotation()

        chassis_velocity = ChassisSpeeds.fromRobotRelativeSpeeds(
            self.chassis.get_velocity(), chassis_rotation
        )

        turret_base_pose = chassis_pose.transformBy(self.TURRET_OFFSET)

        turret_offset_field = (
            turret_base_pose.translation() - chassis_pose.translation()
        )

        turret_base_velocity = Translation2d(
            chassis_velocity.vx - chassis_velocity.omega * turret_offset_field.Y(),
            chassis_velocity.vy + chassis_velocity.omega * turret_offset_field.X(),
        )

        if self.forced_solution is None:
            # https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
            # See heading full integration example
            future_position = (
                turret_base_pose.translation()
                + turret_base_velocity * self.LATENCY_FACTOR
            )
            to_goal = self.target_position - future_position

            effective_distance, turret_angle = self.compute_range_bearing_for(
                to_goal, turret_base_velocity
            )

            # if not self.active_table.is_within_range(effective_distance):
            #     # Try other tables
            #     for table in self.tables:
            #         if table.is_within_range(effective_distance):
            #             self.active_table = table
            #             # recalc effective distance with new table if needed
            #             effective_distance, turret_angle = (
            #                 self.compute_range_bearing_for(
            #                     to_goal, turret_base_velocity
            #                 )
            #             )
            #             break
            #     else:
            #         # No table fits: clamp to closest table
            #         if effective_distance < self.tables[0].dist.min():
            #             self.active_table = self.tables[0]
            #         else:
            #             self.active_table = self.tables[-1]
            #         effective_distance = max(
            #             self.active_table.dist.min(),
            #             min(effective_distance, self.active_table.dist.max()),
            #         )

            required_rpm = self.active_table.speed_for(effective_distance)

            target_turret_angle = (turret_angle - chassis_rotation).radians()
            target_flywheel_speed = required_rpm
            target_hood_angle = self.active_table.hood_angle
            target_hopper_surface_speed = self.active_table.hopper_surface_speed

        else:
            (
                target_flywheel_speed,
                target_turret_angle,
                target_hood_angle,
                target_hopper_surface_speed,
            ) = self.forced_solution

        if self.should_energise_flywheels:
            self.shooter.set_flywheel(target_flywheel_speed)

        if is_in_transition_zone(
            self.chassis.get_pose()
            .exp(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    self.chassis.get_velocity(), chassis_pose.rotation()
                ).toTwist2d(self.EXTRAPOLATION_TIME_FOR_HOOD_SERVO)
            )
            .translation()
        ):
            self.shooter.pitch_min()
            self.leds.too_close_to_trench_to_shoot()
        else:
            self.shooter.pitch_to(target_hood_angle)

        self.turret.slew_to(target_turret_angle)

        if self.is_driving_faster_than_max_shoot_speed():
            self.leds.driving_faster_than_shoot_speed()

        if self.should_energise_hopper:
            self.hopper.set_desired_surface_speed(target_hopper_surface_speed)

        self.is_shooting = False

        self.turret_pose.setPose(
            turret_base_pose.rotateAround(
                turret_base_pose.translation(),
                Rotation2d(self.turret.get_current_angle()),
            )
        )
