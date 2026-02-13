from magicbot import feedback, tunable, will_reset_to
from phoenix6 import configs
from phoenix6.configs import FeedbackConfigs, Slot0Configs, TalonFXConfiguration
from phoenix6.controls import Follower, PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, MotorAlignmentValue, NeutralModeValue
from wpilib import DutyCycleEncoder

from ids import DioChannel, TalonId


class IntakeComponent:
    desired_intake_output = tunable(0.5)
    target_intake_output = will_reset_to(0.0)

    indexer_output = tunable(0.5)

    desired_indexer = will_reset_to(0.0)

    RETRACTED_INTAKE_ANGLE = 0.0
    DEPLOYED_INTAKE_ANGLE = 0.25

    target_deployment_angle = RETRACTED_INTAKE_ANGLE

    DEPLOYER_TO_ENCODER_GEARING = 1.0
    ENCODER_ZERO_OFFSET = 0

    def __init__(self) -> None:
        self.motor = TalonFX(TalonId.INTAKE)
        self.deployment_motor_left = TalonFX(TalonId.INTAKE_DEPLOYER_LEFT)
        self.deployment_motor_right = TalonFX(TalonId.INTAKE_DEPLOYER_RIGHT)
        self.deployment_encoder = DutyCycleEncoder(DioChannel.INTAKE_DEPLOYMENT_ENCODER)
        self.indexer_motor = TalonFX(TalonId.INDEXER)

        self.deployment_motor_left.set_control(
            Follower(
                TalonId.INTAKE_DEPLOYER_RIGHT,
                MotorAlignmentValue(MotorAlignmentValue.OPPOSED),
            )
        )

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
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        ).with_neutral_mode(NeutralModeValue.COAST)

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

        self.deployment_motor_right.configurator.apply(
            TalonFXConfiguration()
            .with_slot0(deployment_motor_gains)
            .with_feedback(deployment_motor_gear_config)
        )

        self.deployment_motor_right.set_position(
            self.get_absolute_deployment_encoder_position()
        )
        self.motor.configurator.apply(motor_config)

    def intake(self) -> None:
        self.target_intake_output = self.desired_intake_output

    def deploy_intake(self) -> None:
        self.target_deployment_angle = self.DEPLOYED_INTAKE_ANGLE

    def retract_intake(self) -> None:
        self.target_deployment_angle = self.RETRACTED_INTAKE_ANGLE

    def index(self) -> None:
        self.desired_indexer = self.indexer_output

    def execute(self) -> None:
        self.motor.set(self.target_intake_output)
        self.deployment_motor_right.set_control(
            PositionVoltage(self.target_deployment_angle)
        )
        self.indexer_motor.set(self.desired_indexer)

    @feedback
    def get_absolute_deployment_encoder_position(self) -> float:
        return self.deployment_encoder.get() - self.ENCODER_ZERO_OFFSET

    @feedback
    def get_raw_absolute_deployment_encoder_position(self) -> float:
        return self.deployment_encoder.get()
