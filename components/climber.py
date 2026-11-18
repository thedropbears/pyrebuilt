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
    MAXIMUM_HEIGHT = 10
    MINIMUM_HEIGHT = 2
    DEVICE_ID = 12
    ENCODER_ID = 1
    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)
    SHAFT_RADIUS = 0.00733  # m
    FUDGE_FACTOR = 0.773
    CANDI_ID = 2
    EXTEND_LIMIT = 0.22
    RETRACT_LIMIT = 0.0
    SENSOR_TO_MECH_RATIO = (1.0 / 1.0) * (1.0 / 7.0) * (1.0 / 4.0)
    MOTION_VOLTAGE = 6.0
    is_deploying = False
    is_retracting = False

    def __init__(self):
        self.motor = TalonFX(self.DEVICE_ID)
        self.switch_candi = CANdi(self.CANDI_ID)
        self.candi_configs = CANdiConfiguration().with_digital_inputs(
            DigitalInputsConfigs()
            .with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING)
            .with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING)
        )
        self.switch_candi.configurator.apply(self.candi_configs)
        self.motor_configs = (
            TalonFXConfiguration()
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
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.BRAKE)
                .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
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
        self.motor.configurator.apply(self.motor_configs)
        self.target_output = 0.0
        self.desired_voltage = 0
        self.has_indexed = False
        pass

    def extend(self):
        self.target_output = self.MOTION_VOLTAGE
        self.is_deploying = True
        self.is_retracting = False

    def retract(self):
        self.target_output = -self.MOTION_VOLTAGE
        self.is_retracting = True
        self.is_deploying = False

    def is_extended(self) -> bool:
        return self.switch_candi.get_s2_closed().value

    def is_retracted(self) -> bool:
        return self.switch_candi.get_s1_closed().value

    def try_index(self):
        if self.is_extended():
            self.motor.set_position(ClimberComponent.MAXIMUM_HEIGHT)
            self.has_indexed = True
            self.target_output = 0

        if self.is_retracted():
            self.motor.set_position(ClimberComponent.RETRACT_LIMIT)
            self.has_indexed = True
            self.target_output = 0

    def execute(self) -> None:
        if self.is_retracting:
            self.target_output = -self.MOTION_VOLTAGE

        if self.is_deploying:
            self.target_output = self.MOTION_VOLTAGE

        self.try_index()

        if not self.has_indexed:
            self.retract()

        if self.is_extended():
            self.target_output = min(self.target_output, 0.0)
        if self.is_retracted() and self.target_output < 0.0:
            self.target_output = max(self.target_output, 0.0)

        self.motor.set_control(
            VoltageOut(self.target_output, ignore_software_limits=not self.has_indexed)
        )

        self.target_output = 0.0
