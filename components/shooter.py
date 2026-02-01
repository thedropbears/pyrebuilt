import math

import rev
from magicbot import feedback, tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from phoenix6 import configs, controls
from phoenix6.controls import Follower
from phoenix6.hardware import TalonFX
from phoenix6.signals import MotorAlignmentValue

from ids import SparkId, TalonId
from utilities.functions import clamp
from utilities.rev import configure_spark_reset_and_persist


class ShooterComponent:
    target_shooter_rps = will_reset_to(0.0)
    desired_shooter_rps = tunable(30)

    target_feeder_percentage = will_reset_to(0)
    desired_feeder_percentage = tunable(1)

    desired_hood_angle = tunable(36.0)
    hood_error_tolerance = 3.0
    MIN_HOOD_ANGLE = 28.9
    MAX_HOOD_ANGLE = 73.4

    ENCODER_ROTS_PER_HOOD_DEGREE = 54 / 26 / 360
    ENCODER_ZERO_OFFSET = 0.472

    def __init__(self) -> None:
        self.flywheel_motor_left = TalonFX(
            device_id=TalonId.FLYWHEEL_LEFT
        )  # Defined from behind shooter
        self.flywheel_motor_right = TalonFX(
            device_id=TalonId.FLYWHEEL_RIGHT
        )  # Defined from behind shooter

        flywheel_gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.036653)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.086321)
            .with_k_v(0.11159)
            .with_k_a(0.0038097)
        )

        self.flywheel_motor_left.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(flywheel_gains_cfg)
        )

        self.feeder_motor = TalonSRX(TalonId.FEEDER)
        self.feeder_motor.setInverted(False)

        self.hood_motor = rev.SparkMax(SparkId.HOOD, rev.SparkMax.MotorType.kBrushless)
        self.hood_motor.setInverted(True)
        self.hood_motor_controller = self.hood_motor.getClosedLoopController()

        hood_motor_cfg = rev.SparkMaxConfig()
        hood_motor_cfg.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        hood_motor_cfg.closedLoop.pid(0.005, 0, 0)  # TODO Tune these values
        hood_motor_cfg.closedLoop.allowedClosedLoopError(self.hood_error_tolerance)
        hood_motor_cfg.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)

        self.hood_encoder = self.hood_motor.getAbsoluteEncoder()
        hood_motor_cfg.apply(rev.AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder())
        hood_motor_cfg.absoluteEncoder.positionConversionFactor(
            1 / self.ENCODER_ROTS_PER_HOOD_DEGREE
        ).zeroOffset(self.ENCODER_ZERO_OFFSET).zeroCentered(True)

        configure_spark_reset_and_persist(self.hood_motor, hood_motor_cfg)

    @feedback
    def get_hood_angle(self):
        return self.hood_encoder.getPosition()

    @feedback
    def hood_is_at_setpoint(self):
        return math.isclose(
            self.get_hood_angle(),
            self.desired_hood_angle,
            abs_tol=self.hood_error_tolerance,
        )

    def pitch_hood_relative(self, angle):
        self.desired_hood_angle += angle

    def pitch_hood_absolute(self, angle):
        self.desired_hood_angle = angle

    def shoot(self) -> None:
        self.target_shooter_rps = self.desired_shooter_rps
        self.target_feeder_percentage = self.desired_feeder_percentage

    def execute(self) -> None:
        self.flywheel_motor_left.set_control(
            controls.VelocityVoltage(self.target_shooter_rps)
        )
        self.flywheel_motor_right.set_control(
            Follower(
                TalonId.FLYWHEEL_LEFT, MotorAlignmentValue(MotorAlignmentValue.OPPOSED)
            )
        )

        self.feeder_motor.set(ControlMode.PercentOutput, self.target_feeder_percentage)

        self.desired_hood_angle = clamp(
            self.desired_hood_angle, self.MIN_HOOD_ANGLE, self.MAX_HOOD_ANGLE
        )

        """self.hood_motor_controller.setSetpoint(
            self.desired_hood_angle, rev.SparkMax.ControlType.kPosition
        )"""
