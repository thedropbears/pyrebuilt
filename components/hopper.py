from math import isclose, pi

from magicbot import will_reset_to
from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import CoastOut, VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpimath import units

from ids import TalonId


class HopperComponent:
    INJECTOR_WHEEL_DIAMETER: units.meters = 0.05
    INDEXER_WHEEL_DIAMETER: units.meters = 0.137

    feed_rate = will_reset_to(0.0)

    ALLOWABLE_INJECTOR_ERROR = 0
    ALLOWABLE_INDEXER_ERROR = 0

    MOTOR_TO_INDEXER_RATIO = 1
    MOTOR_TO_INJECTOR_RATIO = 1

    def __init__(self) -> None:
        self.indexer_motor = TalonFX(TalonId.INDEXER)
        self.injector_motor = TalonFX(TalonId.INJECTOR)

        indexer_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        indexer_feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.MOTOR_TO_INDEXER_RATIO
        )

        indexer_gains_config = (
            Slot0Configs()
            .with_k_s(0.0064613)
            .with_k_v(0.11354)
            .with_k_a(0.017913)
            .with_k_p(0.015874)
        )

        self.indexer_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(indexer_output_config)
            .with_feedback(indexer_feedback_config)
            .with_slot0(indexer_gains_config)
        )

        injector_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        injector_feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.MOTOR_TO_INJECTOR_RATIO
        )

        injector_gains_config = (
            Slot0Configs()
            .with_k_s(0.12926)
            .with_k_v(0.11437)
            .with_k_a(0.0018232)
            .with_k_p(0.12691)
        )

        self.injector_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(injector_output_config)
            .with_feedback(injector_feedback_config)
            .with_slot0(injector_gains_config)
        )

    def on_disable(self) -> None:
        self.indexer_motor.set_control(CoastOut())
        self.injector_motor.set_control(CoastOut())

    def get_indexer_error(self) -> units.turns_per_second:
        return self.indexer_motor.get_closed_loop_error().value

    def get_injector_error(self) -> units.turns_per_second:
        return self.injector_motor.get_closed_loop_error().value

    def get_indexer_surface_speed(self) -> units.meters_per_second:
        return (
            self.indexer_motor.get_velocity().value * pi * self.INDEXER_WHEEL_DIAMETER
        )

    def get_injector_surface_speed(self) -> units.meters_per_second:
        return (
            self.injector_motor.get_velocity().value * pi * self.INJECTOR_WHEEL_DIAMETER
        )

    def is_jammed(self) -> bool:
        return (
            self.get_indexer_error() > self.ALLOWABLE_INDEXER_ERROR
            or self.get_injector_error() > self.ALLOWABLE_INJECTOR_ERROR
        )

    def feed(self, feed_rate: units.meters_per_second) -> None:
        self.feed_rate = feed_rate

    def execute(self) -> None:

        if not isclose(self.feed_rate, 0.0, abs_tol=0.1):
            target_indexer_rps = self.feed_rate / (pi * self.INDEXER_WHEEL_DIAMETER)
            target_injector_rps = self.feed_rate / (pi * self.INJECTOR_WHEEL_DIAMETER)

            self.indexer_motor.set_control(VelocityVoltage(target_indexer_rps))
            self.injector_motor.set_control(VelocityVoltage(target_injector_rps))
        else:
            self.indexer_motor.set_control(CoastOut())
            self.injector_motor.set_control(CoastOut())
