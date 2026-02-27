from magicbot import tunable, will_reset_to
from phoenix6 import configs
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ids import TalonId


class HopperComponent:
    desired_indexer_dutycycle = tunable(0.5)
    target_indexer_dutycycle = will_reset_to(0.0)
    target_feeder_dutycycle = will_reset_to(0)
    desired_feeder_dutycycle = tunable(1)

    def __init__(self) -> None:
        self.indexer_motor = TalonFX(TalonId.INDEXER)

        indexer_output_config = (
            configs.MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )
        self.indexer_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(indexer_output_config)
        )

        self.feeder_motor = TalonFX(TalonId.FEEDER)

        feeder_motor_config = configs.TalonFXConfiguration()
        feeder_motor_config.motor_output.with_inverted(
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        ).with_neutral_mode(NeutralModeValue.COAST)

        self.feeder_motor.configurator.apply(feeder_motor_config)

    def feed(self) -> None:
        self.target_indexer_dutycycle = self.desired_indexer_dutycycle
        self.target_feeder_dutycycle = self.desired_feeder_dutycycle

    def execute(self) -> None:
        self.indexer_motor.set(self.target_indexer_dutycycle)
        self.feeder_motor.set(self.target_feeder_dutycycle)
