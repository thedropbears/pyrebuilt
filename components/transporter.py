from magicbot import tunable, will_reset_to

from ids import SparkId


class _SimMotor:
    def __init__(self) -> None:
        self.output = 0.0

    def set(self, output: float) -> None:
        self.output = output


class TransporterComponent:
    desired_output = will_reset_to(0.0)

    TRANSPORTER_OUTPUT = tunable(0.5)

    def __init__(self) -> None:
        try:
            import rev

            from utilities.rev import configure_spark_ephemeral
        except ModuleNotFoundError:
            self.transporter_motor = _SimMotor()
            return

        self.transporter_motor = rev.SparkMax(
            SparkId.TRANSPORTER, rev.SparkLowLevel.MotorType.kBrushless
        )
        motor_config = rev.SparkMaxConfig()
        motor_config.inverted(False)
        motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        configure_spark_ephemeral(self.transporter_motor, motor_config)

    def set_speed(self) -> None:
        self.desired_output = self.TRANSPORTER_OUTPUT

    def execute(self) -> None:
        self.transporter_motor.set(self.desired_output)
