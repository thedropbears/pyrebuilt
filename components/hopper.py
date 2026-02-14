from magicbot import tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from phoenix6 import configs
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ids import TalonId


class HopperComponent:
    indexer_output = tunable(0.5)
    desired_indexer = will_reset_to(0.0)
    target_feeder_percentage = will_reset_to(0)
    desired_feeder_percentage = tunable(1)
    def __init__(self) -> None:
        self.indexer_motor = TalonFX(TalonId.INDEXER)
        self.indexer_output_config = (
            configs.MotorOutputConfigs()
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )
        self.indexer_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(self.indexer_output_config))
        self.feeder_motor = TalonSRX(TalonId.FEEDER)
        self.feeder_motor.setInverted(False)
    
    def feed(self) -> None:
        self.desired_indexer = self.indexer_output
        self.target_feeder_percentage = self.desired_feeder_percentage

    def execute(self) -> None:
        self.indexer_motor.set(self.desired_indexer)
        self.feeder_motor.set(ControlMode.PercentOutput, self.target_feeder_percentage)