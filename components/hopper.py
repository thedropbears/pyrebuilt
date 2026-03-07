from math import pi

from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpimath import units

from ids import TalonId


class HopperComponent:
    desired_hopper_surface_speed = tunable(10.0)  # meters / sec

    INJECTOR_WHEEL_DIAMETER: units.meters = 0.05
    INDEXER_WHEEL_DIAMETER: units.meters = 0.137

    target_indexer_rps = will_reset_to(0.0)
    target_injector_rps = will_reset_to(0.0)

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
            .with_k_p(0.14972)
            .with_k_i(0.0)
            .with_k_d(0)
            .with_k_s(0.0050824)
            .with_k_v(0.11395)
            .with_k_a(0.01833)
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
            .with_k_p(0.016508)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.074635)
            .with_k_v(0.11359)
            .with_k_a(0.0019126)
        )

        self.injector_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(injector_output_config)
            .with_feedback(injector_feedback_config)
            .with_slot0(injector_gains_config)
        )

    @feedback
    def get_target_indexer_rps(self) -> float:
        return self.target_indexer_rps

    @feedback
    def get_target_injector_rps(self) -> float:
        return self.target_injector_rps

    def get_indexer_surface_speed(self) -> units.meters_per_second:
        return (
            self.indexer_motor.get_velocity().value * pi * self.INDEXER_WHEEL_DIAMETER
        )

    def get_injector_surface_speed(self) -> units.meters_per_second:
        return (
            self.injector_motor.get_velocity().value * pi * self.INJECTOR_WHEEL_DIAMETER
        )

    def feed(self) -> None:

        self.target_indexer_rps = self.desired_hopper_surface_speed / (
            pi * self.INDEXER_WHEEL_DIAMETER
        )
        self.target_injector_rps = self.desired_hopper_surface_speed / (
            pi * self.INJECTOR_WHEEL_DIAMETER
        )

    def execute(self) -> None:
        self.indexer_motor.set_control(VelocityVoltage(self.target_indexer_rps))
        self.injector_motor.set_control(VelocityVoltage(self.target_injector_rps))
