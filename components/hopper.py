from magicbot import tunable, will_reset_to
from phoenix6.configs import MotorOutputConfigs, TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ids import TalonId


class HopperComponent:
    desired_indexer_dutycycle = tunable(0.5)
    target_indexer_dutycycle = will_reset_to(0.0)

    desired_injector_dutycycle = tunable(0.5)
    target_injector_dutycycle = will_reset_to(0.0)

    def __init__(self) -> None:
        self.indexer_motor = TalonFX(TalonId.INDEXER)
        self.injector_motor = TalonFX(TalonId.INJECTOR)

        indexer_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        self.indexer_motor.configurator.apply(
            TalonFXConfiguration().with_motor_output(indexer_output_config)
        )

        injector_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        self.injector_motor.configurator.apply(
            TalonFXConfiguration().with_motor_output(injector_output_config)
        )

    def feed(self) -> None:
        self.target_indexer_dutycycle = self.desired_indexer_dutycycle
        self.target_injector_dutycycle = self.target_injector_dutycycle

    def execute(self) -> None:
        self.indexer_motor.set(self.target_indexer_dutycycle)
        self.injector_motor.set(self.target_injector_dutycycle)
