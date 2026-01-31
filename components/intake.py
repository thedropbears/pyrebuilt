import math

import wpilib
from magicbot import tunable, will_reset_to
from phoenix6 import configs
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import FeedbackSensor, SparkMax, SparkMaxConfig

from ids import SparkId, TalonId
from utilities.rev import (
    configure_spark_ephemeral,
    configure_spark_reset_and_persist,
)


class IntakeComponent:
    desired_output = will_reset_to(0.0)

    intake_output = tunable(0.5)

    desired_funnel = will_reset_to(0.0)

    funnel_output = tunable(1.0)

    indexer_output = tunable(0.5)

    desired_indexer = will_reset_to(0.0)
    ARM_ERROR_TOLERANCE = 3.0
    ENCODER_ROTS_PER_ARM_DEGREE = 1 / 360  # TODO: replace with real gearing
    ARM_ENCODER_ZERO_OFFSET = 0.0  # TODO: tune this

    def __init__(self, intake_mech_root: wpilib.MechanismRoot2d) -> None:
        self.intake_ligament = intake_mech_root.appendLigament(
            "intake", length=0.25, angle=90, color=wpilib.Color8Bit(wpilib.Color.kGreen)
        )
        self.motor = TalonFX(TalonId.INTAKE)
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
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        ).with_neutral_mode(NeutralModeValue.COAST)

        self.motor.configurator.apply(motor_config)

        self.arm_motor = SparkMax(SparkId.INTAKE_ARM, SparkMax.MotorType.kBrushless)
        self.arm_motor.setInverted(False)
        self.arm_motor_controller = self.arm_motor.getClosedLoopController()

        arm_cfg = SparkMaxConfig()
        arm_cfg.inverted(False)
        arm_cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        arm_cfg.closedLoop.pid(0.005, 0.0, 0.0) #TODO: Tune these values
        arm_cfg.closedLoop.allowedClosedLoopError(self.ARM_ERROR_TOLERANCE)
        arm_cfg.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)

        self.arm_encoder = self.arm_motor.getAbsoluteEncoder()
        arm_cfg.absoluteEncoder.positionConversionFactor(
            1 / self.ENCODER_ROTS_PER_ARM_DEGREE
        ).zeroOffset(self.ARM_ENCODER_ZERO_OFFSET).zeroCentered(True)

        configure_spark_reset_and_persist(self.arm_motor, arm_cfg)

    # varibles for arm simulation
    # TODO: verify these values for this years robot IMPORTANT!!
    VERTICAL_ENCODER_VALUE = 4.610450
    ARM_ENCODER_OFFSET = VERTICAL_ENCODER_VALUE - math.pi / 2.0
    DEPLOYED_ANGLE_LOWER = 3.392366 - ARM_ENCODER_OFFSET
    DEPLOYED_ANGLE_UPPER = 3.892366 - ARM_ENCODER_OFFSET
    RETRACTED_ANGLE = 4.610450 - ARM_ENCODER_OFFSET
    ARM_LENGTH = 0.22  # meters
    ARM_MOI = 0.181717788

    gear_ratio = 4.0 * 5.0 * (48.0 / 40.0)

    def intake(self) -> None:
        self.desired_output = self.intake_output
        self.desired_funnel = self.funnel_output

    def index(self) -> None:
        self.desired_indexer = self.indexer_output

    def execute(self) -> None:
        self.motor.set(self.desired_output)
        self.indexer_motor.set(self.desired_indexer)
