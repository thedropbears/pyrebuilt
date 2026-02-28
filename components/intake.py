from math import degrees, radians, tau

from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import (
    CommutationConfigs,
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,
    MotionMagicConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
    TalonFXSConfiguration,
)
from phoenix6.controls import Follower, PositionVoltage
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.signals import (
    ForwardLimitSourceValue,
    GravityTypeValue,
    InvertedValue,
    MotorAlignmentValue,
    MotorArrangementValue,
    NeutralModeValue,
    ReverseLimitSourceValue,
)
from wpilib import Color, Color8Bit, DutyCycleEncoder, MechanismRoot2d, RobotBase
from wpimath import units

from ids import DioChannel, TalonId


class IntakeComponent:
    target_intake_output = will_reset_to(0.0)
    desired_intake_output = tunable(0.5)

    RETRACTED_INTAKE_ANGLE = radians(90)
    DEPLOYED_INTAKE_ANGLE = radians(0)

    target_deployer_angle = DEPLOYED_INTAKE_ANGLE

    MAX_DEPLOYER_ACCEL = 5
    MAX_DEPLOYER_VELOCITY = 5

    DEPLOYER_TO_ENCODER_GEARING = (1 / 5) * (26 / 50)

    ENCODER_ZERO_OFFSET = 3.419406

    # Sim
    ARM_LENGTH = 0.38  # meters
    ARM_MOI = 0.398668741

    def __init__(self, mech_root: MechanismRoot2d) -> None:

        self.intake_motor = TalonFXS(TalonId.INTAKE)
        self.deployer_motor_left = TalonFX(TalonId.INTAKE_DEPLOYER_LEFT)
        self.deployer_motor_right = TalonFX(TalonId.INTAKE_DEPLOYER_RIGHT)
        self.deployer_encoder = DutyCycleEncoder(
            DioChannel.INTAKE_DEPLOYER_ENCODER, tau, 0
        )

        intake_motor_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        intake_motor_commutation_config = CommutationConfigs().with_motor_arrangement(
            MotorArrangementValue.MINION_JST
        )

        self.intake_motor.configurator.apply(
            TalonFXSConfiguration()
            .with_motor_output(intake_motor_output_config)
            .with_commutation(intake_motor_commutation_config)
        )

        # TODO verify on the bot:
        # https://www.reca.lc/arm?armMass=%7B%22s%22%3A4.894%2C%22u%22%3A%22kg%22%7D&comLength=%7B%22s%22%3A0.25%2C%22u%22%3A%22m%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A100%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22Falcon%20500%22%7D&ratio=%7B%22magnitude%22%3A9.61538461538%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        intake_deployer_slot_config = (
            Slot0Configs()
            .with_k_p(5.51)
            .with_k_i(0)
            .with_k_d(2.73)
            .with_k_s(0)
            .with_k_v(1.09)
            .with_k_a(0.26)
            .with_k_g(1.60)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        intake_deployer_output_config = (
            MotorOutputConfigs()
            .with_inverted(
                InvertedValue.CLOCKWISE_POSITIVE
                if not RobotBase.isSimulation()
                else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            )
            .with_neutral_mode(NeutralModeValue.BRAKE)
        )

        intake_deployer_magic_config = (
            MotionMagicConfigs()
            .with_motion_magic_acceleration(self.MAX_DEPLOYER_ACCEL)
            .with_motion_magic_cruise_velocity(self.MAX_DEPLOYER_VELOCITY)
        )

        intake_deployer_hard_limit_config = (
            HardwareLimitSwitchConfigs()
            .with_forward_limit_source(ForwardLimitSourceValue.LIMIT_SWITCH_PIN)
            .with_reverse_limit_source(ReverseLimitSourceValue.LIMIT_SWITCH_PIN)
        )
        intake_deployer_feedback_config = (
            FeedbackConfigs().with_sensor_to_mechanism_ratio(
                1 / (self.DEPLOYER_TO_ENCODER_GEARING)
            )
        )

        self.deployer_motor_left.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(intake_deployer_output_config)
            .with_slot0(intake_deployer_slot_config)
            .with_feedback(intake_deployer_feedback_config)
            .with_motion_magic(intake_deployer_magic_config)
            .with_hardware_limit_switch(intake_deployer_hard_limit_config)
        )

        self.intake_ligament = mech_root.appendLigament(
            "intake",
            length=0.8,
            angle=0.0,
            lineWidth=3,
            color=Color8Bit(Color.kGreen),
        )

    def _sync_encoders(self) -> None:
        self.deployer_motor_left.set_position(
            self.get_absolute_deployer_position() / tau
        )
        self.intake_ligament.setAngle(self.get_deployer_position_degrees())

    def on_disable(self) -> None:
        self._sync_encoders()

    def on_enable(self) -> None:
        self.retract()

    def intake(self) -> None:
        self.target_intake_output = self.desired_intake_output
        self.target_deployer_angle = self.DEPLOYED_INTAKE_ANGLE

    def retract(self) -> None:
        self.target_deployer_angle = self.RETRACTED_INTAKE_ANGLE

    def execute(self) -> None:
        self.deployer_motor_left.set_control(
            PositionVoltage(self.target_deployer_angle / tau)
        )
        self.deployer_motor_right.set_control(
            Follower(
                TalonId.INTAKE_DEPLOYER_LEFT,
                MotorAlignmentValue(MotorAlignmentValue.OPPOSED),
            )
        )
        self.intake_motor.set(self.target_intake_output)

    def periodic(self) -> None:
        self.intake_ligament.setAngle(self.get_deployer_position_degrees())

    def get_absolute_deployer_position(self) -> units.radians:
        return self._get_raw_absolute_deployer_position() - self.ENCODER_ZERO_OFFSET

    @feedback
    def _get_raw_absolute_deployer_position(self) -> units.radians:
        return self.deployer_encoder.get()

    @feedback
    def get_deployer_position(self) -> units.radians:
        return self.deployer_motor_left.get_position().value * tau

    @feedback
    def get_deployer_position_degrees(self) -> units.degrees:
        return degrees(self.get_deployer_position())
