from magicbot import feedback, tunable, will_reset_to
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig

from ids import SparkId
from utilities.rev import configure_spark_reset_and_persist


class ClimberComponent:
    MAX_FORWARD_EXTENSION, MAX_REVERSE_EXTENSION = 120, -120

    current_climber_speed = will_reset_to(0.0)
    forward_climber_speed = tunable(0.8)
    reverse_climber_speed = tunable(0.8)

    def __init__(self):
        # create motor with correct forward direction sparkmax controller
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)
        self.forward_limit_switch = self.motor.getForwardLimitSwitch()
        self.reverse_limit_switch = self.motor.getReverseLimitSwitch()

        config = SparkMaxConfig()
        config.inverted(True)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        config.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        ).forwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotor
        ).reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        ).reverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
        ).reverseLimitSwitchPosition(0)

        config.softLimit.forwardSoftLimit(
            self.MAX_FORWARD_EXTENSION
        ).forwardSoftLimitEnabled(True).reverseSoftLimit(
            self.MAX_REVERSE_EXTENSION
        ).reverseSoftLimitEnabled(True)

        configure_spark_reset_and_persist(self.motor, config)

    def deploy(self):
        self.current_climber_speed = self.forward_climber_speed

    def retract(self):
        self.current_climber_speed = self.reverse_climber_speed * -1

    def execute(self):
        self.motor.set(self.current_climber_speed)

    @feedback
    def forward_limit_switch_triggered(self):
        return self.forward_limit_switch.get()

    @feedback
    def reverse_limit_switch_triggered(self):
        return self.reverse_limit_switch.get()
