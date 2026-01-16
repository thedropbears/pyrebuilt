from magicbot import tunable, will_reset_to
from rev import SparkMax, SparkMaxConfig

from ids import SparkId
from utilities.rev import configure_spark_ephemeral


class IntakeComponent:
    desired_output = will_reset_to(0.0)

    INTAKE_OUTPUT = tunable(0.5)

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.INTAKE, SparkMax.MotorType.kBrushless)

        motor_config = SparkMaxConfig()
        motor_config.inverted(False)
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)

        configure_spark_ephemeral(self.motor, motor_config)

    def intake(self) -> None:
        self.desired_output = IntakeComponent.INTAKE_OUTPUT

    def execute(self) -> None:
        self.motor.set(self.desired_output)
