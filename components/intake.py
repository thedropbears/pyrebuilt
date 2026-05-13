from magicbot import tunable, will_reset_to
from phoenix6.configs import MotorOutputConfigs, TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue

from ids import TalonId


class IntakeComponent:
    target_intake_duty = will_reset_to(0.0)
    desired_intake_duty = tunable(0.8)

    def __init__(self) -> None:
        self.motor = TalonFX(TalonId.INTAKE_PROTO)

        motor_output_config = MotorOutputConfigs().with_inverted(
            InvertedValue.CLOCKWISE_POSITIVE
        )

        self.motor.configurator.apply(
            TalonFXConfiguration().with_motor_output(motor_output_config)
        )

    def eat(self) -> None:
        self.target_intake_duty = self.desired_intake_duty

    def execute(self) -> None:
        self.motor.set(self.target_intake_duty)
