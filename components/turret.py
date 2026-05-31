import math

from magicbot import MagicRobot, feedback, tunable
from phoenix6.configs import (
    CANcoderConfiguration,
    CommutationConfigs,
    ExternalFeedbackConfigs,
    MagnetSensorConfigs,
    MotionMagicConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    SoftwareLimitSwitchConfigs,
    TalonFXSConfiguration,
)
from phoenix6.controls import MotionMagicVoltage
from phoenix6.hardware import CANcoder, TalonFXS
from phoenix6.signals import (
    ExternalFeedbackSensorSourceValue,
    InvertedValue,
    MotorArrangementValue,
    NeutralModeValue,
    SensorDirectionValue,
)
from wpilib import Mechanism2d, SmartDashboard
from wpimath import units

from ids import CancoderId, TalonId
from utilities.functions import clamp


class TurretComponent:
    MOTOR_TO_TURRET_GEARING = (1 / 5) * (40 / 200)
    TURRET_TO_ENCODER_GEARING = (200 / 56) * (14 / 50)

    ENCODER_OFFSET = -0.082275

    ALLOWABLE_ERROR = math.radians(1.0)

    MAX_VELOCITY = 3.0
    MAX_ACCELERATION = 6.0
    MAX_JERK = 18

    desired_angle = tunable(0.0).with_properties(unit="radians")

    MAX_TURRET_ROTATION = math.radians(155)
    MIN_TURRET_ROTATION = math.radians(-145)

    def __init__(self) -> None:
        # Initialise Encoder
        self.absolute_encoder = CANcoder(CancoderId.TURRET)

        self.absolute_encoder.configurator.apply(
            CANcoderConfiguration().with_magnet_sensor(
                MagnetSensorConfigs()
                .with_magnet_offset(self.ENCODER_OFFSET)
                .with_sensor_direction(SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE)
            )
        )

        self.motor = TalonFXS(TalonId.TURRET)
        # Motor gains
        motor_gains_config = (
            Slot0Configs()
            .with_k_p(40.126)
            .with_k_i(0.0)
            .with_k_d(1.797)
            .with_k_s(0.12914)
            .with_k_v(2.3284)
            .with_k_a(0.2601)
        )

        motor_output_config = (
            MotorOutputConfigs()
            .with_inverted(
                InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                if MagicRobot.isSimulation()
                else InvertedValue.CLOCKWISE_POSITIVE
            )
            .with_neutral_mode(NeutralModeValue.BRAKE)
        )

        motor_feedback_config = (
            ExternalFeedbackConfigs()
            .with_sensor_to_mechanism_ratio(
                1 / (TurretComponent.TURRET_TO_ENCODER_GEARING)
            )
            .with_rotor_to_sensor_ratio(
                1
                / (
                    TurretComponent.MOTOR_TO_TURRET_GEARING
                    * TurretComponent.TURRET_TO_ENCODER_GEARING
                )
            )
            .with_external_feedback_sensor_source(
                ExternalFeedbackSensorSourceValue.REMOTE_CANCODER
            )
            .with_feedback_remote_sensor_id(CancoderId.TURRET)
        )

        motor_motion_magic_config = (
            MotionMagicConfigs()
            .with_motion_magic_cruise_velocity(self.MAX_VELOCITY)
            .with_motion_magic_acceleration(self.MAX_ACCELERATION)
            .with_motion_magic_jerk(self.MAX_JERK)
        )

        motor_limits_config = (
            SoftwareLimitSwitchConfigs()
            .with_forward_soft_limit_enable(True)
            .with_forward_soft_limit_threshold(self.MAX_TURRET_ROTATION / math.tau)
            .with_reverse_soft_limit_enable(True)
            .with_reverse_soft_limit_threshold(self.MIN_TURRET_ROTATION / math.tau)
        )

        motor_commutation_config = CommutationConfigs().with_motor_arrangement(
            MotorArrangementValue.MINION_JST
        )

        self.motor.configurator.apply(
            TalonFXSConfiguration()
            .with_slot0(motor_gains_config)
            .with_motor_output(motor_output_config)
            .with_external_feedback(motor_feedback_config)
            .with_motion_magic(motor_motion_magic_config)
            .with_commutation(motor_commutation_config)
            .with_software_limit_switch(motor_limits_config)
        )

        mech = Mechanism2d(2, 2)
        SmartDashboard.putData("Turret", mech)
        mech_root = mech.getRoot("Turret", 1, 1)
        self.sim_pointer = mech_root.appendLigament(
            "pointer", length=1, angle=90, lineWidth=3
        )

    def setup(self) -> None:
        self.slew_to(self.get_current_angle())

    def on_enable(self) -> None:
        self.slew_to(self.get_current_angle())

    def get_current_angle(self) -> units.radians:
        return self.motor.get_position().value * math.tau

    @feedback
    def get_current_angle_degrees(self) -> units.degrees:
        return math.degrees(self.get_current_angle())

    def get_current_velocity(self) -> units.radians_per_second:
        return self.motor.get_velocity().value * math.tau

    @feedback
    def get_current_velocity_degrees_per_second(self) -> units.degrees_per_second:
        return math.degrees(self.get_current_velocity())

    def clamp_rotation(self, angle: units.radians) -> units.radians:
        return clamp(angle, self.MIN_TURRET_ROTATION, self.MAX_TURRET_ROTATION)

    def get_error(self) -> units.turns:
        return self.motor.get_closed_loop_error().value

    @feedback
    def get_error_degrees(self) -> units.degrees:
        return self.get_error() * 360.0

    def slew_relative(self, angle: units.radians) -> None:
        self.desired_angle = self.clamp_rotation(self.get_current_angle() + angle)

    def slew_to(self, angle: units.radians) -> None:
        self.desired_angle = self.clamp_rotation(angle)

    def execute(self) -> None:
        self.motor.set_control(MotionMagicVoltage(self.desired_angle / math.tau))

    def periodic(self) -> None:
        self.sim_pointer.setAngle(self.get_current_angle_degrees())

    def can_rotate_to_target(self) -> bool:
        return self.desired_angle == self.clamp_rotation(self.desired_angle)

    def is_close_to_rotation_limit(self) -> bool:
        return math.isclose(
            abs(self.desired_angle), self.MAX_TURRET_ROTATION, abs_tol=math.radians(15)
        )
