from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import MotorOutputConfigs
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig

from ids import SparkId, TalonId
from utilities.rev import configure_spark_reset_and_persist


class ClimberComponent:
    MAX_FORWARD_EXTENSION, MAX_REVERSE_EXTENSION = 120, -120

    current_climber_speed = will_reset_to(0.0)
    forward_climber_speed = tunable(0.4)
    reverse_climber_speed = tunable(0.4)

    can_deploy = True
    can_retract = True

    def __init__(self):
        # create motor with correct forward direction sparkmax controller
        self.climber_motor = TalonFX(TalonId.CLIMBER)
        self.climber_sensor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)
        self.forward_limit_switch = self.climber_sensor.getForwardLimitSwitch()
        self.reverse_limit_switch = self.climber_sensor.getReverseLimitSwitch()

        sensor_config = SparkMaxConfig()
        sensor_config.inverted(True)
        sensor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        sensor_config.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        ).forwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotor
        ).reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyOpen
        ).reverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
        ).reverseLimitSwitchPosition(0)

        sensor_config.softLimit.forwardSoftLimit(
            self.MAX_FORWARD_EXTENSION
        ).forwardSoftLimitEnabled(True).reverseSoftLimit(
            self.MAX_REVERSE_EXTENSION
        ).reverseSoftLimitEnabled(True)

        configure_spark_reset_and_persist(self.climber_sensor, sensor_config)

        self.climber_motor.configurator.apply(
            MotorOutputConfigs()
            .with_neutral_mode(NeutralModeValue.BRAKE)
            .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
        )

    def deploy(self):
        if self.can_deploy:
            self.current_climber_speed = self.forward_climber_speed

    def retract(self):
        if self.can_retract:
            self.current_climber_speed = self.reverse_climber_speed * -1

    def execute(self):
        if self.at_forward_limit():
            self.can_deploy = False
        if self.at_reverse_limit():
            self.can_retract = False
            self.climber_motor.set_position(0)

        self.climber_motor.set(self.current_climber_speed)

    @feedback
    def at_forward_limit(self):
        return self.forward_limit_switch.get()

    @feedback
    def at_reverse_limit(self):
        return self.reverse_limit_switch.get()

    @feedback
    def get_climber_position(self):
        return self.climber_motor.get_position()
