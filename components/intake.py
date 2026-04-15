from math import degrees, isclose, radians, tau

from magicbot import MagicRobot, feedback, tunable, will_reset_to
from phoenix6.configs import (
    CANcoderConfiguration,
    CommutationConfigs,
    ExternalFeedbackConfigs,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotionMagicConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    Slot1Configs,
    TalonFXConfiguration,
    TalonFXSConfiguration,
)
from phoenix6.controls import Follower, MotionMagicVoltage, NeutralOut, VelocityVoltage
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    GravityTypeValue,
    InvertedValue,
    # MotorAlignmentValue,
    MotorArrangementValue,
    NeutralModeValue,
    SensorDirectionValue,
)
from wpilib import Color, Color8Bit, MechanismRoot2d
from wpimath import units

from ids import CancoderId, TalonId


class IntakeComponent:
    target_intake_rps = will_reset_to(0.0)
    desired_intake_rps = tunable(26.0)  # between 25 and 26 seems to be the sweet spot

    RETRACTED_INTAKE_ANGLE = radians(107.0)
    DEPLOYED_INTAKE_ANGLE = radians(-8.0)

    target_deployer_angle = will_reset_to(RETRACTED_INTAKE_ANGLE)

    MAX_DEPLOYER_VELOCITY = 3
    MAX_DEPLOYER_ACCEL = 6
    MAX_DEPLOYER_JERK = 54

    DEPLOYER_TO_CANCODER_GEARING = (1 / 5) * (26 / 50)
    CANCODER_TO_MECHANISM_GEARING = 1

    MOTOR_TO_INTAKE_GEARING = (1 / 3) * (36 / 26)

    ENCODER_ZERO_OFFSET = 0.1250  # read from phoenix tuner, negated and made to be between 0 and 1 by removing any integer component

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

        intake_motor_feedback_config = (
            ExternalFeedbackConfigs().with_sensor_to_mechanism_ratio(
                1 / self.MOTOR_TO_INTAKE_GEARING
            )
        )

        intake_gains_config = (
            Slot0Configs()
            .with_k_p(0.00067723)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.36827)
            .with_k_v(0.21305)
            .with_k_a(0.0046032)
        )

        intake_motor_commutation_config = CommutationConfigs().with_motor_arrangement(
            MotorArrangementValue.MINION_JST
        )

        self.intake_motor.configurator.apply(
            TalonFXSConfiguration()
            .with_motor_output(intake_motor_output_config)
            .with_commutation(intake_motor_commutation_config)
            .with_slot0(intake_gains_config)
            .with_external_feedback(intake_motor_feedback_config)
        )

        # siq hand tuned gains
        intake_deployer_deploy_config = (
            Slot0Configs()
            .with_k_p(30.63)
            .with_k_i(0.00)
            .with_k_d(3.05)
            .with_k_s(0.2220703125)
            .with_k_v(1.09)
            .with_k_a(0.26)
            .with_k_g(0.6)
            # .with_gravity_arm_position_offset(0.00)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        intake_deployer_hold_config = (
            Slot1Configs()
            .with_k_p(120.63)
            .with_k_i(0.00)
            .with_k_d(4.55)
            .with_k_s(0.2220703125)
            .with_k_v(1.09)
            .with_k_a(0.26)
            .with_k_g(0.6)
            # .with_gravity_arm_position_offset(0.00)
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
            .with_motion_magic_jerk(self.MAX_DEPLOYER_JERK)
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
            .with_slot0(intake_deployer_deploy_config)
            .with_slot1(intake_deployer_hold_config)
            .with_feedback(intake_deployer_feedback_config)
            .with_motion_magic(intake_deployer_magic_config)
        )

        self.deployer_motor_left.configurator.apply(deployer_config)
        self.deployer_motor_right.configurator.apply(deployer_config)

        self.deployer_encoder.configurator.apply(
            CANcoderConfiguration().with_magnet_sensor(
                MagnetSensorConfigs()
                .with_magnet_offset(self.ENCODER_ZERO_OFFSET)
                .with_sensor_direction(
                    SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
                    if MagicRobot.isSimulation()
                    else SensorDirectionValue.CLOCKWISE_POSITIVE
                )
            )
        )

        self.intake_ligament = mech_root.appendLigament(
            "intake",
            length=0.8,
            angle=0.0,
            lineWidth=3,
            color=Color8Bit(Color.kGreen),
        )

    def intake(self) -> None:
        self.drive()
        self.target_deployer_angle = self.DEPLOYED_INTAKE_ANGLE

    def backdrive(self) -> None:
        self.target_intake_rps = -self.desired_intake_rps

    def drive(self) -> None:
        self.target_intake_rps = self.desired_intake_rps

    def execute(self) -> None:
        active_slot = 1 if self.should_use_holding_config() else 0
        self.deployer_motor_left.set_control(
            MotionMagicVoltage(self.target_deployer_angle / tau, slot=active_slot)
        )
        self.deployer_motor_right.set_control(
            Follower(
                TalonId.INTAKE_DEPLOYER_LEFT,
                oppose_master_direction=True,
            )
        )

        if self.target_intake_rps == 0.0:
            self.intake_motor.set_control(NeutralOut())
        else:
            self.intake_motor.set_control(VelocityVoltage(self.target_intake_rps))

    def is_retracted(self) -> bool:
        return isclose(
            self.target_deployer_angle, self.RETRACTED_INTAKE_ANGLE, abs_tol=0.01
        ) and isclose(
            self.get_deployer_position(),
            self.RETRACTED_INTAKE_ANGLE,
            abs_tol=radians(10),
        )

    def should_use_holding_config(self) -> bool:
        return isclose(
            self.target_deployer_angle, self.DEPLOYED_INTAKE_ANGLE, abs_tol=0.01
        ) and isclose(
            self.get_deployer_position(),
            self.DEPLOYED_INTAKE_ANGLE,
            abs_tol=radians(15),
        )

    def periodic(self) -> None:
        self.intake_ligament.setAngle(self.get_deployer_position_degrees())

    @feedback
    def get_deployer_position(self) -> units.radians:
        return self.deployer_encoder.get_position().value * tau

    @feedback
    def get_deployer_position_degrees(self) -> units.degrees:
        return degrees(self.get_deployer_position())
