from math import radians, tau

from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import (
    FeedbackConfigs,
    MotionMagicConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import Follower, MotionMagicVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import (
    GravityTypeValue,
    InvertedValue,
    MotorAlignmentValue,
    NeutralModeValue,
)
from wpilib import Color, Color8Bit, DutyCycleEncoder, MechanismRoot2d

from ids import DioChannel, TalonId


class IntakeComponent:
    target_intake_output = will_reset_to(0.0)
    desired_intake_output = tunable(0.5)

    RETRACTED_INTAKE_ANGLE = radians(0)
    DEPLOYED_INTAKE_ANGLE = radians(90)

    target_deployer_angle = RETRACTED_INTAKE_ANGLE

    MAX_DEPLOYER_ACCEL = 5
    MAX_DEPLOYER_VELOCITY = 5

    DEPLOYER_TO_ENCODER_GEARING = (5 / 1) * (26 / 50)
    ENCODER_ZERO_OFFSET = 0

    # Sim
    ARM_LENGTH = 0.22  # meters
    ARM_MOI = 0.181717788

    def __init__(self, intake_mech_root: MechanismRoot2d) -> None:
        self.intake_ligament = intake_mech_root.appendLigament(
            "intake", 0.25, 90, color=Color8Bit(Color.kGreen)
        )
        self.intake_motor = TalonFX(TalonId.INTAKE)
        self.deployer_motor_left = TalonFX(TalonId.INTAKE_DEPLOYER_LEFT)
        self.deployer_motor_right = TalonFX(TalonId.INTAKE_DEPLOYER_RIGHT)
        self.deployer_encoder = DutyCycleEncoder(DioChannel.INTAKE_DEPLOYER_ENCODER)

        intake_motor_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        self.intake_motor.configurator.apply(
            TalonFXConfiguration().with_motor_output(intake_motor_output_config)
        )

        # TODO tune these
        intake_deployer_slot_config = (
            Slot0Configs()
            .with_k_p(0.02)
            .with_k_i(0)
            .with_k_d(0.75)
            .with_k_s(0)
            .with_k_v(0.05)
            .with_k_a(1.37)
            .with_k_g(6.43)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        intake_deployer_magic_config = (
            MotionMagicConfigs()
            .with_motion_magic_acceleration(self.MAX_DEPLOYER_ACCEL)
            .with_motion_magic_cruise_velocity(self.MAX_DEPLOYER_VELOCITY)
        )

        intake_deployer_feedback_config = (
            FeedbackConfigs().with_sensor_to_mechanism_ratio(
                1 / (self.DEPLOYER_TO_ENCODER_GEARING * tau)
            )
        )

        self.deployer_motor_left.configurator.apply(
            TalonFXConfiguration()
            .with_slot0(intake_deployer_slot_config)
            .with_feedback(intake_deployer_feedback_config)
            .with_motion_magic(intake_deployer_magic_config)
        )

        self.deployer_motor_left.set_position(
            self.get_absolute_deployer_encoder_position()
        )

    def intake(self) -> None:
        self.target_intake_output = self.desired_intake_output
        self.target_deployer_angle = self.DEPLOYED_INTAKE_ANGLE

    def retract(self) -> None:
        self.target_deployer_angle = self.RETRACTED_INTAKE_ANGLE

    def execute(self) -> None:
        self.deployer_motor_left.set_control(
            MotionMagicVoltage(self.target_deployer_angle)
        )
        self.deployer_motor_right.set_control(
            Follower(
                TalonId.INTAKE_DEPLOYER_RIGHT,
                MotorAlignmentValue(MotorAlignmentValue.OPPOSED),
            )
        )
        self.intake_motor.set(self.target_intake_output)

    @feedback
    def get_absolute_deployer_encoder_position(self) -> float:
        return self.deployer_encoder.get() - self.ENCODER_ZERO_OFFSET

    @feedback
    def get_raw_absolute_deployer_encoder_position(self) -> float:
        return self.deployer_encoder.get()

    @feedback
    def get_target_deployment_angle(self) -> float:
        return self.target_deployer_angle
