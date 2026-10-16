import math

import wpilib
from magicbot import feedback, tunable, will_reset_to
from wpimath import units
from wpimath.geometry import Rotation2d, Translation2d

from components.chassis import ChassisComponent
from components.turret import TurretComponent
from utilities.functions import clamp
from utilities.game import field_flip_rotation2d, is_red


def sine_sweep(
    phase: units.radians,
    amplitude: units.radians,
    angular_frequency: units.radians_per_second,
) -> tuple[units.radians, units.radians_per_second]:
    return (
        amplitude * math.sin(phase),
        amplitude * angular_frequency * math.cos(phase),
    )


class ShowoffComponent:
    chassis: ChassisComponent
    turret: TurretComponent
    field: wpilib.Field2d

    is_showing_off = will_reset_to(False)

    min_sweep_angle = tunable(-45.0)  # deg
    max_sweep_angle = tunable(45.0)

    sweep_period = tunable(4.0)  # s

    target_distance = tunable(2.5)  # m

    MIN_SWEEP_PERIOD = 1.0

    HEADING_kP = 2.0

    def __init__(self) -> None:
        self.was_showing_off = False
        self.phase_anchor: units.seconds = 0.0
        self.target_bearing = Rotation2d()
        self.target = Translation2d()

    def setup(self) -> None:
        self.target_pos_obj = self.field.getObject("showoff_target")

    def on_enable(self) -> None:
        self.was_showing_off = False

    def get_sweep(self) -> tuple[Rotation2d, units.radians]:
        min_angle = math.radians(self.min_sweep_angle)
        max_angle = math.radians(self.max_sweep_angle)
        centre = Rotation2d((min_angle + max_angle) / 2)
        return (
            field_flip_rotation2d(centre) if is_red() else centre,
            (max_angle - min_angle) / 2,
        )

    def get_angular_frequency(self) -> units.radians_per_second:
        return math.tau / max(self.sweep_period, self.MIN_SWEEP_PERIOD)

    def get_phase(self) -> units.radians:
        elapsed = wpilib.Timer.getFPGATimestamp() - self.phase_anchor
        return elapsed * self.get_angular_frequency()

    def show_off(self) -> None:
        if not self.was_showing_off:
            self.start_sweep()
        self.is_showing_off = True

    def start_sweep(self) -> None:
        centre, amplitude = self.get_sweep()
        current_rotation = self.chassis.get_rotation()

        offset = (current_rotation - centre).radians()
        phase = math.asin(clamp(offset / amplitude, -1.0, 1.0)) if amplitude else 0.0

        self.phase_anchor = (
            wpilib.Timer.getFPGATimestamp() - phase / self.get_angular_frequency()
        )
        self.target_bearing = current_rotation

    @feedback
    def get_rotation_rate(self) -> units.radians_per_second:
        if not self.is_showing_off:
            return 0.0

        centre, amplitude = self.get_sweep()
        offset, rate = sine_sweep(
            self.get_phase(), amplitude, self.get_angular_frequency()
        )
        heading_error = (centre + Rotation2d(offset)) - self.chassis.get_rotation()

        return rate + self.HEADING_kP * heading_error.radians()

    @feedback
    def get_target(self) -> Translation2d:
        return self.target

    def execute(self) -> None:
        self.was_showing_off = self.is_showing_off

        if not self.is_showing_off:
            return

        self.target = self.chassis.get_pose().translation() + Translation2d(
            self.target_distance, self.target_bearing
        )
        self.target_pos_obj.setPose(self.target.x, self.target.y, self.target_bearing)

        self.turret.slew_to(
            (self.target_bearing - self.chassis.get_rotation()).radians()
        )
