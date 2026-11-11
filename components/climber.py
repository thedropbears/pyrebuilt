from phoenix6.configs import (
    CANdiConfiguration,
    DigitalInputsConfigs,
    FeedbackConfigs,
    HardwareLimitSwitchConfigs,  # noqa: F401
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,  # noqa: F401
    TalonFXConfiguration,
)
from phoenix6.hardware import CANdi, TalonFX
from phoenix6.signals import S1CloseStateValue, S2CloseStateValue
from wpimath.controller import PIDController


class ClimberComponent:
    MAXIMUM_HEIGHT = 10
    MINIMUM_HEIGHT = 2
    DEVICE_ID = 12
    ENCODER_ID = 1
    SENSOR_TO_MECH_RATIO = (1.0 / 1.0) * (1.0 / 7.0) * (1.0 / 4.0)
    desired_voltage = 0

    def __init__(self):
        self.motor = TalonFX(self.DEVICE_ID)
        self.encoder = CANdi(self.ENCODER_ID)
        self.pid = PIDController(1, 0, 1)
        self.motor_configs = (
            TalonFXConfiguration()
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(
                    self.SENSOR_TO_MECH_RATIO
                )
            )
            .with_motor_output(MotorOutputConfigs())
        )
        self.candi_configs = CANdiConfiguration().with_digital_inputs(
            DigitalInputsConfigs()
            .with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING)
            .with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING)
        )

        # get encoder
        # get switchers
        # Motor: Foward Dir
        # Gear reduction
        # Brake modes
        # Normally open or normally closed
        pass

    def extend(self):
        pass

    def retract(self):
        pass

    def is_extended(self) -> bool:
        return True

    def is_retracted(self) -> bool:
        return True

    def try_index(self):
        pass

    def execute(self) -> None:
        self.desired_voltage = 0
        # set motor control
        if self.is_extended() or self.is_retracted():
            self.desired_voltage = 0
        self.request = self.desired_voltage
