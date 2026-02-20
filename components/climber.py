from math import tau

from magicbot import feedback, tunable, will_reset_to
from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig
from wpimath.units import degreesToRotations

from ids import SparkId, TalonId
from utilities.rev import configure_spark_reset_and_persist


class ClimberComponent:
    current_climber_speed = will_reset_to(0.0)
    forward_climber_speed = tunable(0.4)
    reverse_climber_speed = tunable(0.4)

    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)
    SHAFT_RADIUS = 0.00733  # m
    FUDGE_FACTOR = 0.773  # FOR THE LIFE OF ME I DONT KNOW WHY. MAYBE THE BRAKE STAGE ISNT REALLY 1:1

    ALLOWABLE_OVERSPOOL = degreesToRotations(5)
    MAX_RETRACTION_LIMIT = 0 - ALLOWABLE_OVERSPOOL
    MAX_EXTENSION_LIMIT = 0.174316 + ALLOWABLE_OVERSPOOL

    def __init__(self):
        # create motor with correct forward direction sparkmax controller
        self.climber_motor = TalonFX(TalonId.CLIMBER)
        self.climber_sensor = SparkMax(
            SparkId.CLIMBER_SENSOR, SparkMax.MotorType.kBrushless
        )

        # TODO check if you can actually configure the forward and reverse limit switches
        self.retraction_limit_switch = self.climber_sensor.getForwardLimitSwitch()
        self.extension_limit_switch = self.climber_sensor.getReverseLimitSwitch()

        sensor_config = SparkMaxConfig()
        sensor_config.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyClosed
        ).forwardLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotor
        ).reverseLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyClosed
        ).reverseLimitSwitchTriggerBehavior(
            LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition
        ).reverseLimitSwitchPosition(
            0
        )

        configure_spark_reset_and_persist(self.climber_sensor, sensor_config)

        self.climber_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.BRAKE)
                .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
            )
            .with_feedback(
                FeedbackConfigs().with_sensor_to_mechanism_ratio(
                    (
                        1
                        / (
                            ClimberComponent.GEAR_RATIO
                            * ClimberComponent.SHAFT_RADIUS
                            * tau
                        )
                    )
                    * ClimberComponent.FUDGE_FACTOR
                )
            )
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_threshold(self.MAX_EXTENSION_LIMIT)
                .with_forward_soft_limit_enable(True)
                .with_reverse_soft_limit_threshold(self.MAX_RETRACTION_LIMIT)
                .with_reverse_soft_limit_enable(True)
            )
        )

    def deploy(self):
        if not self.at_extension_limit():
            self.current_climber_speed = self.forward_climber_speed

    def retract(self):
        if not self.at_retraction_limit():
            self.current_climber_speed = self.reverse_climber_speed * -1

    def try_index(self) -> None:
        if self.at_retraction_limit():
            self.climber_motor.set_position(0)

    def execute(self):
        self.try_index()
        self.climber_motor.set(self.current_climber_speed)

    @feedback
    def at_extension_limit(self) -> bool:
        return (
            self.extension_limit_switch.get()
            or self.climber_motor.get_fault_forward_soft_limit().value
        )

    @feedback
    def at_retraction_limit(self) -> bool:
        return (
            self.retraction_limit_switch.get()
            or self.climber_motor.get_fault_reverse_soft_limit().value
        )

    @feedback
    def get_climber_position(self) -> float:
        return self.climber_motor.get_position().value
