from math import atan2, degrees, isclose, radians, tau

from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import (
    CANcoderConfiguration,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotionMagicConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    Slot1Configs,
    TalonFXConfiguration,
)
from phoenix6.controls import (
    MotionMagicVoltage,
    NeutralOut,
    PositionVoltage,
    VelocityVoltage,
)
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import (
    FeedbackSensorSourceValue,
    GravityTypeValue,
    InvertedValue,
    NeutralModeValue,
    SensorDirectionValue,
)
from wpilib import Color, Color8Bit, MechanismRoot2d
from wpimath import units

from ids import CancoderId, TalonId


class IntakeComponent:
    desired_roller_rps = tunable(38.0)
    target_roller_rps = will_reset_to(0.0)

    RETRACTED_INTAKE_ANGLE: units.radians = radians(90.0)
    DEPLOYED_INTAKE_ANGLE: units.radians = radians(-18.0)

    target_deployer_angle = will_reset_to(RETRACTED_INTAKE_ANGLE)

    MAX_DEPLOYER_VELOCITY = 1
    MAX_DEPLOYER_ACCEL = 6.0
    MAX_DEPLOYER_JERK = 18.0

    DEPLOYER_TO_CANCODER_GEARING = (1 / 5) * (26 / 50)
    CANCODER_TO_MECHANISM_GEARING = 1

    MOTOR_TO_ROLLER_GEARING = 26 / 36

    ENCODER_ZERO_OFFSET = -0.486328125  # read from phoenix tuner, negated and made to be between 0 and 1 by removing any integer component

    # Sim
    ARM_LENGTH = 0.34  # meters
    ARM_MOI = 0.21313981

    def __init__(self, mech_root: MechanismRoot2d) -> None:
        self.roller_motor = TalonFX(TalonId.INTAKE_ROLLER)
        self.deployer_motor = TalonFX(TalonId.INTAKE_DEPLOYER)
        self.deployer_encoder = CANcoder(CancoderId.INTAKE)

        roller_slot_config = (
            Slot0Configs()
            .with_k_v(0.15879)
            .with_k_a(0.0039303)
            .with_k_s(0.31864)
            .with_k_p(0.036116)
            .with_k_d(0.0)
        )

        roller_motor_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        roller_motor_feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / self.MOTOR_TO_ROLLER_GEARING
        )

        self.roller_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(roller_motor_output_config)
            .with_feedback(roller_motor_feedback_config)
            .with_slot0(roller_slot_config)
        )

        # siq hand tuned gains
        deployer_deploy_config = (
            Slot0Configs()
            .with_k_v(2.2)
            .with_k_a(0.2)
            .with_k_s(0.12)
            .with_k_g(0.8)
            .with_gravity_arm_position_offset(-atan2(24.115, 206.87) / tau)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
            .with_k_p(60.0)
            .with_k_d(3.0)
        )

        deployer_hold_config = (
            Slot1Configs()
            .with_k_p(90.0)
            .with_k_i(0.00)
            .with_k_d(3)
            .with_k_s(0.12)
            .with_k_g(0.8)
            .with_gravity_arm_position_offset(-atan2(24.115, 206.87) / tau)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        deployer_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.BRAKE)
        )

        deployer_magic_config = (
            MotionMagicConfigs()
            .with_motion_magic_acceleration(self.MAX_DEPLOYER_ACCEL)
            .with_motion_magic_cruise_velocity(self.MAX_DEPLOYER_VELOCITY)
            .with_motion_magic_jerk(self.MAX_DEPLOYER_JERK)
        )

        deployer_feedback_config = (
            FeedbackConfigs()
            .with_rotor_to_sensor_ratio(1 / (self.DEPLOYER_TO_CANCODER_GEARING))
            .with_sensor_to_mechanism_ratio(1 / self.CANCODER_TO_MECHANISM_GEARING)
            .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
            .with_feedback_remote_sensor_id(self.deployer_encoder.device_id)
        )

        deployer_config = (
            TalonFXConfiguration()
            .with_motor_output(deployer_output_config)
            .with_slot0(deployer_deploy_config)
            .with_slot1(deployer_hold_config)
            .with_feedback(deployer_feedback_config)
            .with_motion_magic(deployer_magic_config)
        )

        self.deployer_motor.configurator.apply(deployer_config)

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

    def intake(self) -> None:
        self.drive()
        self.target_deployer_angle = self.DEPLOYED_INTAKE_ANGLE

    def backdrive(self) -> None:
        self.target_roller_rps = -self.desired_roller_rps

    def drive(self) -> None:
        self.target_roller_rps = self.desired_roller_rps

    def execute(self) -> None:
        if self.should_use_holding_config():
            self.deployer_motor.set_control(
                PositionVoltage(self.target_deployer_angle / tau, slot=1)
            )
        else:
            self.deployer_motor.set_control(
                MotionMagicVoltage(self.target_deployer_angle / tau)
            )

        if self.target_roller_rps == 0.0:
            self.roller_motor.set_control(NeutralOut())
        else:
            self.roller_motor.set_control(VelocityVoltage(self.target_roller_rps))

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
            abs_tol=radians(5.0),
        )

    def periodic(self) -> None:
        self.intake_ligament.setAngle(self.get_deployer_position_degrees())

    @feedback
    def get_deployer_position(self) -> units.radians:
        return self.deployer_encoder.get_position().value * tau

    @feedback
    def get_deployer_position_degrees(self) -> units.degrees:
        return degrees(self.get_deployer_position())

    @feedback
    def get_deployer_error(self) -> units.degrees:
        return self.deployer_motor.get_closed_loop_error().value * 360
