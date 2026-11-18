import math

from magicbot import feedback
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


class ClimberComponent:  # recipe for a thing. Primatives are a number, word etc. a class is a more complext thing.
    DEVICE_ID_MOTOR = 12  # capitalised with underscores is a global constant
    DEVICE_ID_CANDI = 2

    RETRACT_LIMIT = 0.0
    EXTEND_LIMIT = 0.22
    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)  # A 1 to 1, a 1 to 9, a 1 to 4
    SHAFT_RADIUS = 0.0073  # meters
    FUDGE_FACTOR = 0.773

    MOTION_VOLTAGE = 6.0

    def __init__(self):  # equivalent to a constructor, make this fucking thing please
        self.motor = TalonFX(self.DEVICE_ID_MOTOR)
        self.candi = CANdi(self.DEVICE_ID_CANDI)
        self.candi.configurator.apply(
            CANdiConfiguration().with_digital_inputs(
                DigitalInputsConfigs()
                .with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING)
                .with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING)
            )
        )

        self.motor = TalonFX(ClimberComponent.DEVICE_ID_MOTOR)
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
                .with_forward_limit_remote_sensor_id(ClimberComponent.DEVICE_ID_CANDI)
                .with_reverse_limit_source(ReverseLimitSourceValue.REMOTE_CANDI_S1)
                .with_forward_limit_remote_sensor_id(ClimberComponent.DEVICE_ID_CANDI)
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

    def extend(self):  # move up until you hit the limit switch
        self.target_output = ClimberComponent.MOTION_VOLTAGE

    def retract(self):  # move down until you hit the limit switch
        self.target_output = -ClimberComponent.MOTION_VOLTAGE

    @feedback
    def is_extended(self) -> bool:  # read limit switch and return its state
        return self.candi.get_s2_closed().value

    @feedback
    def is_retracted(self) -> bool:  # read limit switch and return its state
        return self.candi.get_s1_closed().value

    @feedback
    def motor_position(self) -> float:
        return self.motor.get_position().value

    def try_index(
        self,
    ) -> None:  # move down until you reach the limit switch, then set as home (relative encoder to an absolute encoder)
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
        # send motor command
        # zero next command


# this is an example of what code would look like
# homework - commit it
# look at magic bot documentation
# look at documention of hardware - pheonix 6
# git status, add, commit - m "what i am committing", push
