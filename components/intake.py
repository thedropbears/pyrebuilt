from magicbot import tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from phoenix6 import configs
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import SparkMax, SparkMaxConfig

from ids import SparkId, TalonId
from utilities.rev import configure_spark_ephemeral


class IntakeComponent:
    desired_output = will_reset_to(0.0)

    intake_output = tunable(0.5)

    desired_funnel = will_reset_to(0.0)

    funnel_output = tunable(1.0)

    indexer_output = tunable(0.5)

    desired_indexer = will_reset_to(0.0)

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.INTAKE, SparkMax.MotorType.kBrushless)
        self.left_funnel_motor = TalonSRX(TalonId.LEFT_FUNNEL)
        self.right_funnel_motor = TalonSRX(TalonId.RIGHT_FUNNEL)
        self.indexer_motor = TalonFX(TalonId.INDEXER)
        motor_config = SparkMaxConfig()
        motor_config.inverted(False)
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        indexer_output_config = (
            configs.MotorOutputConfigs()
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )
        self.indexer_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(indexer_output_config)
        )
        self.left_funnel_motor.setInverted(True)
        self.right_funnel_motor.setInverted(True)

        configure_spark_ephemeral(self.motor, motor_config)

    def intake(self) -> None:
        self.desired_output = self.intake_output
        self.desired_funnel = self.funnel_output

    def index(self) -> None:
        self.desired_indexer = self.indexer_output

    def execute(self) -> None:
        self.motor.set(self.desired_output)
        self.left_funnel_motor.set(ControlMode.PercentOutput, self.desired_funnel)
        self.right_funnel_motor.set(ControlMode.PercentOutput, self.desired_funnel)
        self.indexer_motor.set(self.desired_indexer)
