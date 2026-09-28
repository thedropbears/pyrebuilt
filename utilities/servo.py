import math
from dataclasses import dataclass

import wpilib
from wpilib.simulation import (
    DutyCycleEncoderSim,
    PWMSim,
)
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.interpolation import TimeInterpolatableRotation2dBuffer

from utilities.functions import clamp
from utilities.rev import configure_through_bore_encoder


@dataclass
class TurretCalibration:
    """
    Raw encoder readings measured with the turret in three known positions.
    Read each one off raw_encoder_reading_() in test mode.
    """

    # Turret pointed forwards by hand (not using the servo). This defines turret angle 0.
    encoder_at_forward: Rotation2d
    # Servo held at neutral, with hold_servo_neutral()
    encoder_at_servo_neutral: Rotation2d
    # Servo held at the top of its travel, with hold_servo_full_range()
    encoder_at_servo_full_range: Rotation2d
    # Further limit to the servo's travel.
    encoder_at_min_limit: Rotation2d
    encoder_at_max_limit: Rotation2d


class ServoTurret:
    # Servo commands stay inside these, away from the physical end stops
    MIN_COMMAND = 0.01
    MAX_COMMAND = 0.99

    HISTORY_LENGTH = 2.0  # s

    def __init__(
        self,
        servo_channel: int,
        encoder_channel: int,
        calibration: TurretCalibration,
    ) -> None:
        self.servo = wpilib.Servo(servo_channel)
        self.encoder = wpilib.DutyCycleEncoder(encoder_channel, math.tau, 0.0)
        configure_through_bore_encoder(self.encoder)
        self.calibration = calibration

        # Servo geometry as turret angles
        self.neutral_angle = (
            calibration.encoder_at_servo_neutral - self.calibration.encoder_at_forward
        ).radians()
        measured_travel = (
            calibration.encoder_at_servo_full_range
            - calibration.encoder_at_servo_neutral
        ).radians() % math.tau
        self.half_travel = measured_travel / (2.0 * self.MAX_COMMAND - 1.0)

        # Reachable travel is limited by the command clamp, not the servo's end stops.
        self.min_angle = max(
            (
                calibration.encoder_at_min_limit - self.calibration.encoder_at_forward
            ).radians(),
            self._command_to_angle(self.MIN_COMMAND),
        )
        self.max_angle = min(
            (
                calibration.encoder_at_max_limit - self.calibration.encoder_at_forward
            ).radians(),
            self._command_to_angle(self.MAX_COMMAND),
        )

        self.target_angle = self.neutral_angle
        self._override_command: float | None = None
        self._history = TimeInterpolatableRotation2dBuffer(self.HISTORY_LENGTH)

    def set_target(self, turret_angle: units.radians) -> None:
        self.target_angle = self.clamp_angle(turret_angle)

    def clamp_angle(self, turret_angle: units.radians) -> units.radians:
        return clamp(turret_angle, self.min_angle, self.max_angle)

    def update(self) -> None:
        """Call once per loop: sends the servo command and records where the turret is."""
        if self._override_command is not None:
            self.servo.set(self._override_command)
            self._override_command = None
        else:
            self.servo.set(self._angle_to_command(self.target_angle))
        self._history.addSample(wpilib.Timer.getFPGATimestamp(), self.rotation())

    def raw_encoder_reading_(self) -> Rotation2d:
        """The encoder value before calibration is applied. Record this to fill in TurretCalibration."""
        return Rotation2d(self.encoder.get())

    def rotation(self) -> Rotation2d:
        return self.raw_encoder_reading_() - self.calibration.encoder_at_forward

    def rotation_at(self, timestamp: float) -> Rotation2d | None:
        return self._history.sample(timestamp)

    def is_connected(self) -> bool:
        return self.encoder.isConnected()

    def _angle_to_command(self, turret_angle: units.radians) -> float:
        # -1 at one end of the servo's travel, +1 at the other
        normalised = (turret_angle - self.neutral_angle) / self.half_travel
        return clamp((normalised + 1.0) / 2.0, self.MIN_COMMAND, self.MAX_COMMAND)

    def _command_to_angle(self, command: float) -> units.radians:
        """Turret angle the servo drives to for a given command. Inverse of _angle_to_command."""
        return self.neutral_angle + (2.0 * command - 1.0) * self.half_travel

    def _angle_to_raw_encoder(self, turret_angle: units.radians) -> units.radians:
        return (turret_angle + self.calibration.encoder_at_forward.radians()) % math.tau

    def hold_neutral_(self) -> None:
        """Holds the servo at neutral for this loop. Call every loop while calibrating."""
        self._override_command = 0.5

    def hold_full_range_(self) -> None:
        """Holds the servo at the top of its travel for this loop. Call every loop while calibrating."""
        self._override_command = self.MAX_COMMAND


class ServoTurretSim:
    """Physical model of a ServoTurret: servo command -> turret angle -> encoder reading."""

    def __init__(
        self,
        turret: ServoTurret,
        max_speed: units.radians_per_second,
        starting_angle: units.radians,
    ) -> None:
        self.pwm_sim = PWMSim(turret.servo.getChannel())
        self.encoder_sim = DutyCycleEncoderSim(turret.encoder)
        self.command_to_angle = turret._command_to_angle
        self.angle_to_raw_encoder = turret._angle_to_raw_encoder
        self.max_speed = max_speed
        self.angle: units.radians = starting_angle
        self._publish()

    def update(self, dt: units.seconds) -> None:
        command = self.pwm_sim.getPosition()
        target = self.command_to_angle(command)

        error = target - self.angle
        max_step = self.max_speed * dt
        self.angle += min(max(error, -max_step), max_step)

        self._publish()

    def _publish(self) -> None:
        # The real encoder reads [0, tau), counting anticlockwise with the turret.
        self.encoder_sim.set(self.angle_to_raw_encoder(self.angle))
