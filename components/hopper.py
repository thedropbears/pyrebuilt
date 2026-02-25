from magicbot import tunable, will_reset_to
from phoenix5 import WPI_TalonSRX
from rev import SparkMax, SparkMaxConfig

from ids import SparkId, TalonId
from utilities.rev import configure_spark_ephemeral


class HopperComponent:
    desired_indexer_dutycycle = tunable(0.5)
    target_indexer_dutycycle = will_reset_to(0.0)
    target_feeder_dutycycle = will_reset_to(0)
    desired_feeder_dutycycle = tunable(1)

    def __init__(self) -> None:
        self.indexer_motor = SparkMax(SparkId.INDEXER, SparkMax.MotorType.kBrushless)
        indexer_motor_config = SparkMaxConfig()
        indexer_motor_config.inverted(False)
        indexer_motor_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        configure_spark_ephemeral(self.indexer_motor, indexer_motor_config)

        self.feeder_motor = WPI_TalonSRX(TalonId.FEEDER)
        self.feeder_motor.setInverted(False)

    def feed(self) -> None:
        self.target_indexer_dutycycle = self.desired_indexer_dutycycle
        self.target_feeder_dutycycle = self.desired_feeder_dutycycle

    def execute(self) -> None:
        self.indexer_motor.set(self.target_indexer_dutycycle)
        self.feeder_motor.set(self.target_feeder_dutycycle)
