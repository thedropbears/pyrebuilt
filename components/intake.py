from math import degrees, isclose, radians, tau

from magicbot import MagicRobot, feedback, tunable, will_reset_to
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
from phoenix6.controls import MotionMagicVoltage, NeutralOut, VelocityVoltage
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
    # TODO tune all of these
    target_roller_rps = will_reset_to(0.0)
    desired_roller_rps = tunable(26.0)  # between 25 and 26 seems to be the sweet spot

    RETRACTED_INTAKE_ANGLE = radians(107.0)
    DEPLOYED_INTAKE_ANGLE = radians(-8.0)

    target_deployer_angle = will_reset_to(RETRACTED_INTAKE_ANGLE)

    MAX_DEPLOYER_VELOCITY = 3
    MAX_DEPLOYER_ACCEL = 6
    MAX_DEPLOYER_JERK = 54

    DEPLOYER_TO_CANCODER_GEARING = (1 / 5) * (26 / 50)
    CANCODER_TO_MECHANISM_GEARING = 1

    MOTOR_TO_ROLLER_GEARING = (1 / 3) * (36 / 26)

    ENCODER_ZERO_OFFSET = 0.1250  # read from phoenix tuner, negated and made to be between 0 and 1 by removing any integer component

    # Sim
    ARM_LENGTH = 0.34  # meters
    ARM_MOI = 0.21313981

    def __init__(self, mech_root: MechanismRoot2d) -> None:
        self.roller_motor = TalonFX(TalonId.INTAKE_ROLLER)
        self.deployer_motor = TalonFX(TalonId.INTAKE_DEPLOYER)
        self.deployer_encoder = CANcoder(CancoderId.INTAKE)

        roller_motor_output_config = (
            MotorOutputConfigs()
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            .with_neutral_mode(NeutralModeValue.COAST)
        )

        roller_motor_feedback_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / self.MOTOR_TO_ROLLER_GEARING
        )

        roller_gains_config = (
            Slot0Configs()
            .with_k_p(0.00067723)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.36827)
            .with_k_v(0.21305)
            .with_k_a(0.0046032)
        )

        self.roller_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(roller_motor_output_config)
            .with_slot0(roller_gains_config)
            .with_feedback(roller_motor_feedback_config)
        )

        # siq hand tuned gains
        deployer_deploy_config = (
            Slot0Configs()
            .with_k_p(30.63)
            .with_k_i(0.00)
            .with_k_d(3.05)
            .with_k_s(0.2220703125)
            .with_k_v(1.09)
            .with_k_a(0.26)
            .with_k_g(0.6)
            .with_gravity_arm_position_offset(0.00)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        deployer_hold_config = (
            Slot1Configs()
            .with_k_p(90.63)
            .with_k_i(0.00)
            .with_k_d(4)
            .with_k_s(0.2220703125)
            .with_k_v(1.09)
            .with_k_a(0.26)
            .with_k_g(0.6)
            .with_gravity_arm_position_offset(0.00)
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
        self.target_roller_rps = -self.desired_roller_rps

    def drive(self) -> None:
        self.target_roller_rps = self.desired_roller_rps

    def execute(self) -> None:
        active_slot = 1 if self.should_use_holding_config() else 0
        self.deployer_motor.set_control(
            MotionMagicVoltage(self.target_deployer_angle / tau, slot=active_slot)
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
