import math

from magicbot import feedback, tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from phoenix6 import configs, controls
from phoenix6.controls import Follower
from phoenix6.hardware import TalonFX
from phoenix6.signals import MotorAlignmentValue
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig
from wpilib import DutyCycleEncoder

from ids import DioChannel, SparkId, TalonId
from utilities.rev import (
    configure_spark_reset_and_persist,
    configure_through_bore_encoder,
)


class ShooterComponent:
    target_shooter_rps = will_reset_to(0.0)
    desired_shooter_rps = tunable(30)

    target_feeder_percentage = will_reset_to(0)
    desired_feeder_percentage = tunable(1)

    desired_hood_angle = tunable(60)
    MAX_HOOD_ANGLE = 70  # TODO Tune this value
    MIN_HOOD_ANGLE = 10  # TODO Tune this value

    ENCODER_ROTS_PER_HOOD_DEGREE = 54 / 26 / 360
    ENCODER_ZERO_OFFSET = 0  # TODO Tune this value

    def __init__(self) -> None:
        self.flywheel_motor_left = TalonFX(
            device_id=TalonId.FLYWHEEL_LEFT
        )  # Defined from behind shooter
        self.flywheel_motor_right = TalonFX(
            device_id=TalonId.FLYWHEEL_RIGHT
        )  # Defined from behind shooter

        self.feeder_motor = TalonSRX(TalonId.FEEDER)
        self.feeder_motor.setInverted(False)

        self.hood_motor = SparkMax(SparkId.HOOD, SparkMax.MotorType.kBrushless)
        self.hood_motor_controller = self.hood_motor.getClosedLoopController()

        self.hood_encoder = DutyCycleEncoder(DioChannel.HOOD_ENCODER, math.tau, 0)
        configure_through_bore_encoder(self.hood_encoder)
        self.hood_encoder.setInverted(False)  # TODO Tune this value

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

        hood_motor_cfg = SparkMaxConfig()

        hood_motor_cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        hood_motor_cfg.closedLoop.pid(
            0.01, 0, 0, ClosedLoopSlot.kSlot1
        )  # TODO Tune these values

        configure_spark_reset_and_persist(self.hood_motor, hood_motor_cfg)

    @feedback
    def raw_encoder_value(self):
        return self.hood_encoder.get()

    @feedback
    def raw_turret_angle(self):
        return (
            self.raw_encoder_value() - self.ENCODER_ZERO_OFFSET
        ) * self.ENCODER_ROTS_PER_HOOD_DEGREE

    def articluate_relative(self, angle):
        pass

    def articluate_absolute(self, angle):
        pass

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
