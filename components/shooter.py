from magicbot import tunable, will_reset_to
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ids import TalonId


class ShooterComponent:
    desired_output = will_reset_to(0.0)

    FLYWHEEL_OUTPUT = tunable(0.5)

    def __init__(self) -> None:
        self.flywheel_motor = TalonFX(device_id=TalonId.FLYWHEEL)
        flywheel_config = self.flywheel_motor.configurator
        motor_config = TalonFXConfiguration()
        motor_config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        motor_config.motor_output.neutral_mode = NeutralModeValue.COAST

        flywheel_config.apply(motor_config)

    def set_speed(self) -> None:
        self.desired_output = self.FLYWHEEL_OUTPUT

    def execute(self) -> None:
        self.flywheel_motor.set_control(DutyCycleOut(self.desired_output))
