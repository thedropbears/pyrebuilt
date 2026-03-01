import math

import rev
from magicbot import feedback, will_reset_to
from phoenix6 import configs, controls
from phoenix6.hardware import TalonFX
from wpimath import units

from ids import SparkId, TalonId
from utilities.functions import clamp
from utilities.rev import configure_spark_reset_and_persist


class ShooterComponent:
    target_shooter_rps = will_reset_to(0.0)

    hood_error_tolerance = math.radians(1.0)
    MIN_HOOD_ANGLE = math.radians(30)
    MAX_HOOD_ANGLE = math.radians(68)

    ENCODER_ROTS_PER_HOOD_RADIAN = 4 / math.tau
    ENCODER_ZERO_OFFSET = (0.47222) + (
        ENCODER_ROTS_PER_HOOD_RADIAN * math.radians(30.0)
    )

    FLYWHEEL_GEAR_RATIO = 1 / (30 / 18)

    def __init__(self) -> None:
        self.flywheel_motor = TalonFX(device_id=TalonId.FLYWHEEL)

        flywheel_gains_cfg = (
            configs.Slot0Configs()
            .with_k_s(0.16635)
            .with_k_v(0.070258)
            .with_k_a(0.0045557)
        )
        feedback_config = configs.FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.FLYWHEEL_GEAR_RATIO
        )
        self.flywheel_motor.configurator.apply(
            configs.TalonFXConfiguration()
            .with_slot0(flywheel_gains_cfg)
            .with_feedback(feedback_config)
        )

        self.hood_motor = rev.SparkMax(SparkId.HOOD, rev.SparkMax.MotorType.kBrushless)
        self.hood_motor_controller = self.hood_motor.getClosedLoopController()

        hood_motor_cfg = rev.SparkMaxConfig()
        hood_motor_cfg.inverted(True)
        hood_motor_cfg.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        hood_motor_cfg.closedLoop.pid(0.005, 0, 0)  # TODO Tune these values
        hood_motor_cfg.closedLoop.allowedClosedLoopError(self.hood_error_tolerance)
        hood_motor_cfg.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)

        self.hood_encoder = self.hood_motor.getAbsoluteEncoder()
        hood_motor_cfg.apply(rev.AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder())
        hood_motor_cfg.absoluteEncoder.positionConversionFactor(
            1 / self.ENCODER_ROTS_PER_HOOD_RADIAN
        ).zeroOffset(self.ENCODER_ZERO_OFFSET).zeroCentered(False).inverted(False)

        configure_spark_reset_and_persist(self.hood_motor, hood_motor_cfg)

        self.target_hood_angle = self.get_hood_angle()

    @feedback
    def get_hood_angle_degrees(self) -> units.degrees:
        return math.degrees(self.get_hood_angle())

    def get_hood_angle(self) -> units.radians:
        return self.hood_encoder.getPosition()

    @feedback
    def hood_is_at_setpoint(self) -> bool:
        return math.isclose(
            self.get_hood_angle(),
            self.target_hood_angle,
            abs_tol=self.hood_error_tolerance,
        )

    @feedback
    def get_flywheel_error(self) -> units.turns_per_second:
        return self.flywheel_motor.get_closed_loop_error().value

    def pitch_relative(self, angle: units.radians):
        self.pitch_to(self.target_hood_angle + angle)

    def pitch_to(self, angle: units.radians):
        self.target_hood_angle = angle

    def set_flywheel(self, speed: units.turns_per_second):
        self.target_shooter_rps = speed

    def execute(self) -> None:
        if self.target_shooter_rps != 0.0:
            self.flywheel_motor.set_control(
                controls.VelocityVoltage(self.target_shooter_rps)
            )
        else:
            self.flywheel_motor.set_control(controls.CoastOut())

        self.target_hood_angle = clamp(
            self.target_hood_angle, self.MIN_HOOD_ANGLE, self.MAX_HOOD_ANGLE
        )

        self.hood_motor_controller.setSetpoint(
            self.target_hood_angle, rev.SparkMax.ControlType.kPosition
        )
