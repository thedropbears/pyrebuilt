import enum
from math import tau

from magicbot import feedback
from phoenix6.configs import (
    CANdiConfiguration,
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.controls import VoltageOut
from phoenix6.hardware import CANdi, TalonFX
from phoenix6.signals import (
    ForwardLimitSourceValue,
    InvertedValue,
    NeutralModeValue,
    ReverseLimitSourceValue,
    S1CloseStateValue,
    S2CloseStateValue,
)
from wpilib import DigitalInput

from ids import CandiId, DioChannel, TalonId


@enum.unique
class LastIndex(enum.IntEnum):
    NONE = 0
    RETRACTED = 1
    EXTENDED = 2


class ClimberComponent:
    last_index = LastIndex.NONE

    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)
    SHAFT_RADIUS = 0.00733  # m
    FUDGE_FACTOR = 0.773  # FOR THE LIFE OF ME I DONT KNOW WHY. MAYBE THE BRAKE STAGE ISNT REALLY 1:1

    RETRACTED_POS = 0.0  # m
    EXTENDED_POS = 0.22  # m

    ALLOWABLE_OVERSPOOL = 0.02  # m
    MAX_RETRACTION_LIMIT = RETRACTED_POS - ALLOWABLE_OVERSPOOL
    MAX_EXTENSION_LIMIT = EXTENDED_POS + ALLOWABLE_OVERSPOOL

    MOTION_VOLTAGE = 8.0

    def __init__(self):
        # create motor with correct forward direction sparkmax controller
        self.climber_motor = TalonFX(TalonId.CLIMBER)
        self.climber_sensor = CANdi(CandiId.CLIMBER_SENSOR)

        # Defined as driving with intake forward
        self.front_breakbeam_sensor = DigitalInput(
            DioChannel.CLIMBER_BREAKBEAM_SENSOR_FRONT
        )
        self.back_breakbeam_sensor = DigitalInput(
            DioChannel.CLIMBER_BREAKBEAM_SENSOR_BACK
        )

        climber_sensor_configs = CANdiConfiguration()
        climber_sensor_configs.digital_inputs.s1_close_state = (
            S1CloseStateValue.CLOSE_WHEN_FLOATING
        )
        climber_sensor_configs.digital_inputs.s2_close_state = (
            S2CloseStateValue.CLOSE_WHEN_FLOATING
        )
        self.climber_sensor.configurator.apply(climber_sensor_configs)

        climber_motor_output_configs = (
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.BRAKE)
            .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        )
        climber_motor_feedback_configs = (
            FeedbackConfigs().with_sensor_to_mechanism_ratio(
                (
                    1
                    / (
                        ClimberComponent.GEAR_RATIO
                        * ClimberComponent.SHAFT_RADIUS
                        * tau
                    )
                )
                * ClimberComponent.FUDGE_FACTOR
            )
        )

        climber_motor_hard_limit_configs = (
            HardwareLimitSwitchConfigs()
            .with_forward_limit_source(ForwardLimitSourceValue.REMOTE_CANDI_S2)
            .with_forward_limit_remote_sensor_id(self.climber_sensor.device_id)
            .with_reverse_limit_source(ReverseLimitSourceValue.REMOTE_CANDI_S1)
            .with_reverse_limit_remote_sensor_id(self.climber_sensor.device_id)
        )

        climber_motor_soft_limit_configs = (
            SoftwareLimitSwitchConfigs()
            .with_forward_soft_limit_threshold(self.MAX_EXTENSION_LIMIT)
            .with_forward_soft_limit_enable(True)
            .with_reverse_soft_limit_threshold(self.MAX_RETRACTION_LIMIT)
            .with_reverse_soft_limit_enable(True)
        )

        self.climber_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(climber_motor_output_configs)
            .with_feedback(climber_motor_feedback_configs)
            .with_software_limit_switch(climber_motor_soft_limit_configs)
            .with_hardware_limit_switch(climber_motor_hard_limit_configs)
        )

        self.target_output = 0.0

    def deploy(self):
        self.target_output = self.MOTION_VOLTAGE

    def retract(self):
        self.target_output = -self.MOTION_VOLTAGE

    def try_index(self) -> None:

        if (
            self.last_index != LastIndex.RETRACTED
            and self.get_retraction_limit_switch_state()
        ):
            self.climber_motor.set_position(self.RETRACTED_POS)
            self.last_index = LastIndex.RETRACTED

        if (
            self.last_index != LastIndex.EXTENDED
            and self.get_extension_limit_switch_state()
        ):
            self.climber_motor.set_position(self.EXTENDED_POS)
            self.last_index = LastIndex.EXTENDED

    def execute(self):
        self.try_index()

        if self.last_index == LastIndex.NONE:
            self.climber_motor.set_control(VoltageOut(-ClimberComponent.MOTION_VOLTAGE))
            return

        self.climber_motor.set_control(VoltageOut(self.target_output))

        if self.at_extension_limit():
            self.target_output = min(
                self.target_output, 0.0
            )  # this will be negative if retracting and used or 0.0

        if self.at_retraction_limit():
            self.target_output = max(
                self.target_output, 0.0
            )  # this will be positive if extending and used or 0.0

    @feedback
    def get_extension_limit_switch_state(self) -> bool:
        return self.climber_sensor.get_s2_closed().value

    @feedback
    def at_extension_limit(self) -> bool:
        return (
            self.get_extension_limit_switch_state()
            or self.climber_motor.get_fault_forward_soft_limit().value
        )

    @feedback
    def get_retraction_limit_switch_state(self) -> bool:
        return self.climber_sensor.get_s1_closed().value

    @feedback
    def at_retraction_limit(self) -> bool:
        return (
            self.get_retraction_limit_switch_state()
            or self.climber_motor.get_fault_reverse_soft_limit().value
        )

    @feedback
    def get_position(self) -> float:
        return self.climber_motor.get_position().value

    @feedback
    def at_tower_front_hook(self) -> bool:
        return not self.front_breakbeam_sensor.get()

    @feedback
    def at_tower_back_hook(self) -> bool:
        return not self.back_breakbeam_sensor.get()
