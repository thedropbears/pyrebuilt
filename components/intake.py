from magicbot import tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from rev import SparkMax, SparkMaxConfig

from ids import SparkId, TalonId
from utilities.rev import configure_spark_ephemeral


class IntakeComponent:
    desired_output = will_reset_to(0.0)

    intake_output = tunable(0.5)

    desired_funnel = will_reset_to(0.0)

    funnel_output = tunable(1.0)

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.INTAKE, SparkMax.MotorType.kBrushless)
        self.left_funnel_motor = TalonSRX(TalonId.LEFT_FUNNEL)
        self.right_funnel_motor = TalonSRX(TalonId.RIGHT_FUNNEL)

        motor_config = SparkMaxConfig()
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        configure_spark_ephemeral(self.motor, motor_config)

        self.left_funnel_motor.setInverted(True)


    def intake(self) -> None:
        self.desired_output = self.intake_output
        self.desired_funnel = self.funnel_output

    def execute(self) -> None:
        self.motor.set(self.desired_output)
        self.left_funnel_motor.set(ControlMode.PercentOutput, self.desired_funnel)
        self.right_funnel_motor.set(ControlMode.Follower)
