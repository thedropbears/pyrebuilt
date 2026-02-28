from math import tau

from magicbot import feedback, tunable
from phoenix6.configs import (
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.hardware import CANdi, TalonFX
from phoenix6.signals import (
    ForwardLimitSourceValue,
    GravityTypeValue,
    InvertedValue,
    NeutralModeValue,
    ReverseLimitSourceValue,
)
from wpilib import DigitalInput

from ids import CandiId, DioChannel, TalonId


class ClimberComponent:
    has_indexed = tunable(False)

    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)
    SHAFT_RADIUS = 0.00733  # m
    FUDGE_FACTOR = 0.773  # FOR THE LIFE OF ME I DONT KNOW WHY. MAYBE THE BRAKE STAGE ISNT REALLY 1:1

    RETRACTED_POS = 0.0  # m
    EXTENDED_POS = 0.22  # m

    target_pos = tunable(RETRACTED_POS)

    ALLOWABLE_OVERSPOOL = 0.02  # m
    MAX_RETRACTION_LIMIT = RETRACTED_POS - ALLOWABLE_OVERSPOOL
    MAX_EXTENSION_LIMIT = EXTENDED_POS + ALLOWABLE_OVERSPOOL

    INDEX_SEARCH_VOLTAGE = -8.0

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

        climber_motor_slot0_configs = (
            Slot0Configs()
            .with_k_p(594.12)
            .with_k_i(0)
            .with_k_d(230.65)
            .with_k_s(0.0030436)
            .with_k_v(72.04)
            .with_k_a(1.11881)
            .with_k_g(0.16926)
            .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
        )
        self.climber_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(climber_motor_output_configs)
            .with_feedback(climber_motor_feedback_configs)
            .with_slot0(climber_motor_slot0_configs)
            .with_software_limit_switch(climber_motor_soft_limit_configs)
            .with_hardware_limit_switch(climber_motor_hard_limit_configs)
        )

    def deploy(self):
        self.target_pos = self.EXTENDED_POS

    def retract(self):
        self.target_pos = self.RETRACTED_POS

    def try_index(self) -> None:
        if self.at_retraction_limit() and not self.has_indexed:
            self.climber_motor.set_position(self.RETRACTED_POS)
            self.has_indexed = True
            self.target_pos = self.RETRACTED_POS

    def execute(self):
        """self.try_index()

        if self.has_indexed:
            self.climber_motor.set_control(PositionVoltage(self.target_pos))
        else:
            self.climber_motor.set_control(
                VoltageOut(
                    ClimberComponent.INDEX_SEARCH_VOLTAGE, ignore_software_limits=True
                )
            )"""

    @feedback
    def at_extension_limit(self) -> bool:
        return (
            self.climber_sensor.get_s2_closed().value
            or self.climber_motor.get_fault_forward_soft_limit().value
        )

    @feedback
    def at_retraction_limit(self) -> bool:
        return (
            self.climber_sensor.get_s1_closed().value
            or self.climber_motor.get_fault_reverse_soft_limit().value
        )

    @feedback
    def get_position(self) -> float:
        return self.climber_motor.get_position().value

    @feedback
    def at_tower_front_hook(self) -> bool:
        return self.front_breakbeam_sensor.get()

    @feedback
    def at_tower_back_hook(self) -> bool:
        return self.back_breakbeam_sensor.get()
