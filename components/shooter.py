from magicbot import tunable, will_reset_to
from phoenix6.configs import MotorOutputConfigs
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue


class ShooterComponent:
    desired_output = will_reset_to(0.0)

    FLYWHEEL_OUTPUT = tunable(0.5)

    def __init__(self) -> None:
        self.flywheel_motor = TalonFX(device_id=1)
        flywheel_config = self.flywheel_motor.configurator
        motor_config = MotorOutputConfigs()
        motor_config.inverted = InvertedValue(False)
        motor_config.neutral_mode = NeutralModeValue.COAST

        flywheel_config.apply(motor_config)

    def set_speed(self) -> None:
        self.desired_output = self.FLYWHEEL_OUTPUT

    def execute(self) -> None:
        self.flywheel_motor.set_control(DutyCycleOut(self.desired_output))
