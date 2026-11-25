import math

from phoenix6.configs import (
    CANdiConfiguration,
    DigitalInputsConfigs,
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


class ClimberComponent:
    CANDI_ID = 2
    MOTOR_ID = 12

    RETRACT_LIMIT = 0.0
    EXTEND_LIMIT = 0.22
    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)
    SHAFT_RADIUS = 0.00733  # m
    FUDGE_FACTOR = 0.773  # FOR THE LIFE OF ME I DONT KNOW WHY. MAYBE THE BRAKE STAGE ISNT REALLY 1:1

    MOTION_VOLTAGE = 6.0

    def __init__(self):

        self.switch_candi = CANdi(ClimberComponent.CANDI_ID)
        self.switch_candi.configurator.apply(
            CANdiConfiguration().with_digital_inputs(
                DigitalInputsConfigs()
                .with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING)
                .with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING)
            )
        )

        self.motor = TalonFX(ClimberComponent.MOTOR_ID)
        self.motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.BRAKE)
                .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            )
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(
                    1.0
                    / (
                        ClimberComponent.GEAR_RATIO
                        * math.tau
                        * ClimberComponent.SHAFT_RADIUS
                    )
                    * ClimberComponent.FUDGE_FACTOR
                )
            )
            .with_hardware_limit_switch(
                HardwareLimitSwitchConfigs()
                .with_forward_limit_source(ForwardLimitSourceValue.REMOTE_CANDI_S2)
                .with_forward_limit_remote_sensor_id(ClimberComponent.CANDI_ID)
                .with_reverse_limit_source(ReverseLimitSourceValue.REMOTE_CANDI_S1)
                .with_reverse_limit_remote_sensor_id(ClimberComponent.CANDI_ID)
            )
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_enable(True)
                .with_forward_soft_limit_threshold(ClimberComponent.EXTEND_LIMIT)
                .with_reverse_soft_limit_enable(True)
                .with_reverse_soft_limit_threshold(ClimberComponent.RETRACT_LIMIT)
            )
        )

        self.target_output = 0.0
        self.has_indexed = False

    def retract(self):
        self.target_output = -ClimberComponent.MOTION_VOLTAGE

    def extend(self):
        self.target_output = ClimberComponent.MOTION_VOLTAGE

    def is_retracted(self) -> bool:
        return self.switch_candi.get_s1_closed().value

    def is_extended(self) -> bool:
        return self.switch_candi.get_s2_closed().value

    def try_index(self):

        if self.is_extended():
            self.motor.set_position(ClimberComponent.EXTEND_LIMIT)
            self.has_indexed = True

        if self.is_retracted():
            self.motor.set_position(ClimberComponent.RETRACT_LIMIT)
            self.has_indexed = True

    def execute(self) -> None:

        self.try_index()

        if not self.has_indexed:
            self.retract()

        if self.is_extended():
            self.target_output = min(self.target_output, 0.0)

        if self.is_retracted():
            self.target_output = max(self.target_output, 0.0)

        self.motor.set_control(
            VoltageOut(self.target_output, ignore_software_limits=not self.has_indexed)
        )

        self.target_output = 0.0
