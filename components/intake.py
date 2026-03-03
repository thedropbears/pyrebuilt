from math import degrees, radians, tau

from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import (
    CANcoderConfiguration,
    CommutationConfigs,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotionMagicConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXConfiguration,
    TalonFXSConfiguration,
)
from phoenix6.controls import Follower, MotionMagicVoltage
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    GravityTypeValue,
    InvertedValue,
    MotorAlignmentValue,
    MotorArrangementValue,
    NeutralModeValue,
    SensorDirectionValue,
)
from wpilib import Color, Color8Bit, MechanismRoot2d
from wpimath import units

from ids import CancoderId, TalonId


class IntakeComponent:
    target_intake_output = will_reset_to(0.0)
    desired_intake_output = tunable(0.5)

    RETRACTED_INTAKE_ANGLE = radians(119)
    DEPLOYED_INTAKE_ANGLE = radians(0)

    target_deployer_angle = DEPLOYED_INTAKE_ANGLE

    MAX_DEPLOYER_ACCEL = 9
    MAX_DEPLOYER_VELOCITY = 9

    DEPLOYER_TO_CANCODER_GEARING = (1 / 5) * (26 / 50)
    CANCODER_TO_MECHANISM_GEARING = 1

    ENCODER_ZERO_OFFSET = 0.141846  # read from phoenix tuner, negated and made to be between 0 and 1 by removing any integer component

    # Sim
    ARM_LENGTH = 0.38  # meters
    ARM_MOI = 0.398668741

    def __init__(self, mech_root: MechanismRoot2d) -> None:

        self.intake_motor = TalonFXS(TalonId.INTAKE)
        self.deployer_motor_left = TalonFX(TalonId.INTAKE_DEPLOYER_LEFT)
        self.deployer_motor_right = TalonFX(TalonId.INTAKE_DEPLOYER_RIGHT)
        self.deployer_encoder = CANcoder(CancoderId.INTAKE)

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

        # siq hand tuned gains
        intake_deployer_slot_config = (
            Slot0Configs()
            .with_k_p(80.63)
            .with_k_i(60.00)
            .with_k_d(5.05)
            .with_k_s(0.2220703125)
            .with_k_v(1.09)
            .with_k_a(0.26)
            .with_k_g(1.6)
            .with_gravity_arm_position_offset(0.00)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        intake_deployer_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.BRAKE)
        )

        intake_deployer_magic_config = (
            MotionMagicConfigs()
            .with_motion_magic_acceleration(self.MAX_DEPLOYER_ACCEL)
            .with_motion_magic_cruise_velocity(self.MAX_DEPLOYER_VELOCITY)
        )

        intake_deployer_feedback_config = (
            FeedbackConfigs()
            .with_rotor_to_sensor_ratio(1 / (self.DEPLOYER_TO_CANCODER_GEARING))
            .with_sensor_to_mechanism_ratio(1 / self.CANCODER_TO_MECHANISM_GEARING)
            .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
            .with_feedback_remote_sensor_id(self.deployer_encoder.device_id)
        )

        deployer_config = (
            TalonFXConfiguration()
            .with_motor_output(intake_deployer_output_config)
            .with_slot0(intake_deployer_slot_config)
            .with_feedback(intake_deployer_feedback_config)
            .with_motion_magic(intake_deployer_magic_config)
        )

        self.deployer_motor_left.configurator.apply(deployer_config)
        self.deployer_motor_right.configurator.apply(deployer_config)

        self.deployer_encoder.configurator.apply(
            CANcoderConfiguration().with_magnet_sensor(
                MagnetSensorConfigs()
                .with_magnet_offset(self.ENCODER_ZERO_OFFSET)
                .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
            )
        )

        self.intake_ligament = mech_root.appendLigament(
            "intake",
            length=0.8,
            angle=0.0,
            lineWidth=3,
            color=Color8Bit(Color.kGreen),
        )

    def on_enable(self) -> None:
        self.retract()

    def intake(self) -> None:
        self.target_intake_output = self.desired_intake_output
        self.target_deployer_angle = self.DEPLOYED_INTAKE_ANGLE

    def retract(self) -> None:
        self.target_deployer_angle = self.RETRACTED_INTAKE_ANGLE

    def execute(self) -> None:
        self.deployer_motor_left.set_control(
            MotionMagicVoltage(self.target_deployer_angle / tau)
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

    @feedback
    def get_deployer_position(self) -> units.radians:
        return self.deployer_encoder.get_position().value * tau

    @feedback
    def get_deployer_position_degrees(self) -> units.degrees:
        return degrees(self.get_deployer_position())
