from magicbot import tunable, will_reset_to
from phoenix6 import configs
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue

from ids import TalonId


class IntakeComponent:
    desired_output = will_reset_to(0.0)

    intake_output = tunable(0.5)

    desired_funnel = will_reset_to(0.0)

    funnel_output = tunable(1.0)

    def __init__(self) -> None:
        self.motor = TalonFX(TalonId.INTAKE)

        motor_config = configs.TalonFXConfiguration()
        motor_config.motor_output.with_inverted(
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        ).with_neutral_mode(NeutralModeValue.COAST)

        self.motor.configurator.apply(motor_config)

    def intake(self) -> None:
        # TODO make sure this deploys
        self.desired_output = self.intake_output
        self.desired_funnel = self.funnel_output

    def retract(self) -> None:
        # TODO make sure that this retracts the intake
        # This is a placeholder function for use by the conductor state machine
        pass

    def execute(self) -> None:
        self.motor.set(self.desired_output)
