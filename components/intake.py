from magicbot import tunable, will_reset_to
from phoenix6.hardware import TalonFX

from ids import TalonId


class IntakeComponent:
    target_intake_duty = will_reset_to(0.0)
    desired_intake_duty = tunable(0.8)

    def __init__(self) -> None:
        self.motor = TalonFX(TalonId.INTAKE_PROTO)

    def eat(self) -> None:
        self.target_intake_duty = self.desired_intake_duty

    def execute(self) -> None:
        self.motor.set(self.target_intake_duty)