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
DISTANCE_LOOKUP_25 = np.array([2.5,   3.0,   3.5,   4.0,   4.5,   5.0,   5.5], dtype=float)
SPEED_LOOKUP_25 =    np.array([80.0,  84.0,  88.0,  90.0,  101.0, 112.0, 120.0], dtype=float)
TIME_LOOKUP_25 =     np.array([0.871, 1.004, 1.041, 1.080, 1.155, 1.212, 1.297], dtype=float)

DISTANCE_LOOKUP_45 = np.array([4.0,   4.5,   5.0,   5.5,   6.0,   6.5,   7.0], dtype=float)
SPEED_LOOKUP_45 =    np.array([80.0,  84.0,  88.0,  90.0,  101.0, 112.0, 120.0], dtype=float)
TIME_LOOKUP_45 =     np.array([0.871, 1.004, 1.041, 1.080, 1.155, 1.212, 1.297], dtype=float)

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


class BallisticsComponent:
    chassis: ChassisComponent
    shooter: ShooterComponent
    turret: TurretComponent
    leds: LEDComponent
    hopper: HopperComponent

    forced_solution = will_reset_to[ForcedSolution | None](None)
    should_energise_flywheels = will_reset_to(False)

    EXTRAPOLATION_TIME_FOR_HOOD_SERVO: units.seconds = 2

    LEAD_SHOT_ITERATIONS = tunable(2)

    MINIMUM_LEAD_DISTANCE = 2.0

    TURRET_OFFSET = Transform2d(Translation2d(0.134, -0.166), Rotation2d())
    MAX_DRIVE_SPEED_FOR_SHOOTING: units.meters_per_second = 2

    is_shooting = tunable(False)

    def __init__(self, field: Field2d) -> None:
        self.predicted_shot_base_visual = field.getObject("predicted_shot_base")
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

        turret_base_velocity = ChassisSpeeds(
            chassis_velocity.vx - chassis_velocity.omega * turret_offset_field.Y(),
            chassis_velocity.vy + chassis_velocity.omega * turret_offset_field.X(),
            chassis_velocity.omega,
        )

        if self.forced_solution is None:
            rel_target_trans = self.target_position - turret_base_pose.translation()

            # Get velocity vector of shot (m/s)
            shot_vector = self.calculate_shot_velocity(
                rel_target_trans, turret_base_velocity
            )

            self.distance_to_target = rel_target_trans.norm()

            # Convert shot angle to chassis relative
            target_turret_angle = (
                shot_vector.angle() - turret_base_pose.rotation()
            ).radians()

            # Check if distance is within range of distance table and switch if necessary
            if not self.active_table.is_within_range(self.distance_to_target):
                for table_pair in self.tables:
                    if table_pair.is_within_range(self.distance_to_target):
                        self.active_table = table_pair

            target_hood_angle = self.active_table.hood_angle

            # TODO add this implementation
            # Note - this currently uses the flywheel speed calculated from original active hood angle.
            # If the hood angle changes, this will be incorrect. If we encounter this case, then should
            # calculate effective distance, and compute new flywheel speed from that.
            target_flywheel_speed: units.turns_per_second = (
                self.active_table.mps_to_rps(shot_vector.norm())
            )

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
            self.shooter.pitch_to(self.shooter.MIN_HOOD_ANGLE)
            self.leds.too_close_to_trench_to_shoot()
        else:
            self.shooter.pitch_to(target_hood_angle)

        self.turret.slew_to(target_turret_angle)

        if self.is_driving_faster_than_max_shoot_speed():
            self.leds.driving_faster_than_shoot_speed()
        self.hopper.set_desired_surface_speed(target_hopper_surface_speed)

        self.is_shooting = False

        self.turret_pose.setPose(
            turret_base_pose.rotateAround(
                turret_base_pose.translation(),
                Rotation2d(self.turret.get_current_angle()),
            )
        )

        # TODO enable this again
        # self.predicted_shot_base_visual.setPose(
        #     Pose2d(predicted_shot_base, Rotation2d())
        # )
