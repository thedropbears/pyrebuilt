from magicbot import tunable, will_reset_to
from rev import SparkMax, SparkMaxConfig

from ids import SparkId
from utilities.rev import configure_spark_ephemeral


class IntakeComponent:
    desired_output = will_reset_to(0.0)

    intake_output = tunable(0.5)

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.INTAKE, SparkMax.MotorType.kBrushless)

        intake_motor_config = SparkMaxConfig()
        intake_motor_config.inverted(False)
        intake_motor_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)

        configure_spark_ephemeral(self.motor, intake_motor_config)

    def intake(self) -> None:
        # TODO make sure this deploys
        self.desired_output = self.intake_output

    def retract(self) -> None:
        # TODO make sure that this retracts the intake
        # This is a placeholder function for use by the conductor state machine
        pass

    def execute(self) -> None:
        self.motor.set(self.desired_output)
