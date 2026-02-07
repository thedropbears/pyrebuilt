from magicbot import feedback, tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from phoenix6 import configs
from phoenix6.configs import FeedbackConfigs, Slot0Configs, TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpilib import DutyCycleEncoder

from ids import DioChannel, SparkId, TalonId


class IntakeComponent:
    desired_intake_output = tunable(0.5)
    target_intake_output = will_reset_to(0.0)

    funnel_output = tunable(1.0)

    indexer_output = tunable(0.5)

    desired_indexer = will_reset_to(0.0)
    desired_funnel_output = tunable(1.0)
    target_funnel_output = will_reset_to(0.0)

    RETRACTED_INTAKE_ANGLE = 0
    DEPLOYED_INTAKE_ANGLE = 90

    target_deployment_angle = RETRACTED_INTAKE_ANGLE

    DEPLOYER_TO_ENCODER_GEARING = 1.0

    @feedback
    def get_absolute_deployment_encoder_position(self):
        return self.deployment_encoder.get()

    def __init__(self) -> None:
        self.motor = TalonFX(TalonId.INTAKE)
        self.intake_motor = SparkMax(SparkId.INTAKE, SparkMax.MotorType.kBrushless)
        self.deployment_motor = TalonFX(TalonId.INTAKE_DEPLOYER)
        self.deployment_encoder = DutyCycleEncoder(DioChannel.INTAKE_DEPLOYMENT_ENCODER)
        self.left_funnel_motor = TalonSRX(TalonId.LEFT_FUNNEL)
        self.right_funnel_motor = TalonSRX(TalonId.RIGHT_FUNNEL)
        self.indexer_motor = TalonFX(TalonId.INDEXER)

        indexer_output_config = (
            configs.MotorOutputConfigs()
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )
        self.indexer_motor.configurator.apply(
            configs.TalonFXConfiguration().with_motor_output(indexer_output_config)
        )

        motor_config = configs.TalonFXConfiguration()
        motor_config.motor_output.with_inverted(
            InvertedValue.CLOCKWISE_POSITIVE
        ).with_neutral_mode(NeutralModeValue.COAST)

        self.left_funnel_motor.setInverted(True)
        self.right_funnel_motor.setInverted(True)

        # TODO tune these
        deployment_motor_gains = (
            Slot0Configs()
            .with_k_p(0.1)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0)
            .with_k_v(0)
            .with_k_a(0)
        )

        deployment_motor_gear_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / self.DEPLOYER_TO_ENCODER_GEARING
        )

        self.deployment_motor.configurator.apply(
            TalonFXConfiguration()
            .with_slot0(deployment_motor_gains)
            .with_feedback(deployment_motor_gear_config)
        )

        self.deployment_motor.set_position(
            self.get_absolute_deployment_encoder_position()
        )

    def intake(self) -> None:
        self.target_intake_output = self.desired_intake_output
        self.target_funnel_output = self.desired_funnel_output

    def deploy_intake(self):
        self.target_deployment_angle = self.DEPLOYED_INTAKE_ANGLE

    def retract_intake(self):
        self.target_deployment_angle = self.RETRACTED_INTAKE_ANGLE

    def index(self) -> None:
        self.desired_indexer = self.indexer_output

    def execute(self) -> None:
        self.intake_motor.set(self.target_intake_output)
        self.left_funnel_motor.set(ControlMode.PercentOutput, self.target_funnel_output)
        self.right_funnel_motor.set(
            ControlMode.PercentOutput, self.target_funnel_output
        )
        self.deployment_motor.set_position(self.target_deployment_angle)
        self.indexer_motor.set(self.desired_indexer)
