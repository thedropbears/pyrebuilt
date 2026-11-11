from phoenix6.configs import (
    CANdiConfiguration,
    DigitalInputsConfigs,
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.hardware import CANdi, TalonFX
from phoenix6.signals import (
    InvertedValue,
    NeutralModeValue,
    S1CloseStateValue,
    S2CloseStateValue,
)


class ClimberComponent:
    DEVICE_ID_MOTOR = 12
    DEVICE_ID_CANDI = 13
    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)

    def __init__(self):
        self.motor = TalonFX(
            self.DEVICE_ID_MOTOR
        )  # define forward dir, gear red, brake mode (brake)
        self.candi = CANdi(self.DEVICE_ID_CANDI)
        self.candi_configuration = CANdiConfiguration().with_digital_inputs(
            DigitalInputsConfigs()
            .with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING)
            .with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING)
        )
        self.motor_configuration = (
            TalonFXConfiguration()
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.BRAKE)
                .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            )
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(self.GEAR_RATIO)
            )
            .with_hardware_limit_switch(HardwareLimitSwitchConfigs())
            .with_software_limit_switch(SoftwareLimitSwitchConfigs())
        )
        self.desired_voltage = 0

    def is_extended(self) -> bool:
        # read limit switches
        return True  # placeholder

    def is_retracted(self) -> bool:
        # read limit switches
        return True  # placeholder

    def extend(self) -> None:
        pass

    def retract(self) -> None:
        pass

    def try_index(self) -> None:
        pass

    def execute(self) -> None:

        if self.is_extended():
            pass
            # prevent further extension

        if self.is_retracted():
            pass
            # prevent further retraction

        else:
            pass

        self.motor.set(self.desired_voltage)

        # zero next command

        pass
