from math import tau

from magicbot import feedback, tunable
from phoenix6.configs import (
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    SoftwareLimitSwitchConfigs,
    TalonFXConfiguration,
)
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import GravityTypeValue, InvertedValue, NeutralModeValue
from rev import LimitSwitchConfig, SparkMax, SparkMaxConfig

from ids import SparkId, TalonId
from utilities.rev import configure_spark_reset_and_persist


class ClimberComponent:
    has_indexed = False

    GEAR_RATIO = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)
    SHAFT_RADIUS = 0.00733  # m
    FUDGE_FACTOR = 0.773  # FOR THE LIFE OF ME I DONT KNOW WHY. MAYBE THE BRAKE STAGE ISNT REALLY 1:1

    RETRACTED_POS = 0.0  # m
    EXTENDED_POS = 0.22  # m

    target_pos = tunable(RETRACTED_POS)

    ALLOWABLE_OVERSPOOL = 0.02  # m
    MAX_RETRACTION_LIMIT = RETRACTED_POS - ALLOWABLE_OVERSPOOL
    MAX_EXTENSION_LIMIT = EXTENDED_POS + ALLOWABLE_OVERSPOOL

    INDEX_SEARCH_VOLTAGE = -8.0

    def __init__(self):
        # create motor with correct forward direction sparkmax controller
        self.climber_motor = TalonFX(TalonId.CLIMBER)
        self.climber_sensor = SparkMax(
            SparkId.CLIMBER_SENSOR, SparkMax.MotorType.kBrushless
        )

        self.retraction_limit_switch = self.climber_sensor.getForwardLimitSwitch()
        self.extension_limit_switch = self.climber_sensor.getReverseLimitSwitch()

        sensor_config = SparkMaxConfig()
        sensor_config.limitSwitch.forwardLimitSwitchType(
            LimitSwitchConfig.Type.kNormallyClosed
        ).reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)

        configure_spark_reset_and_persist(self.climber_sensor, sensor_config)

        self.climber_motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.BRAKE)
                .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
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
            .with_slot0(
                Slot0Configs()
                .with_k_p(594.12)
                .with_k_i(0)
                .with_k_d(230.65)
                .with_k_s(0.0030436)
                .with_k_v(72.04)
                .with_k_a(1.11881)
                .with_k_g(0.16926)
                .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
            )
        )

    def deploy(self):
        self.target_pos = self.EXTENDED_POS

    def retract(self):
        self.target_pos = self.RETRACTED_POS

    def try_index(self) -> None:

        if self.at_retraction_limit() and not self.has_indexed:
            self.climber_motor.set_position(self.RETRACTED_POS)
            self.has_indexed = True
            self.target_pos = self.RETRACTED_POS

    def execute(self):
        self.try_index()

        if self.has_indexed:
            self.climber_motor.set_control(
                PositionVoltage(
                    self.target_pos,
                    limit_forward_motion=self.at_extension_limit(),
                    limit_reverse_motion=self.at_retraction_limit(),
                )
            )
        else:
            self.climber_motor.set_control(
                VoltageOut(
                    ClimberComponent.INDEX_SEARCH_VOLTAGE, ignore_software_limits=True
                )
            )

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
    def get_position(self) -> float:
        return self.climber_motor.get_position().value

    @feedback  # the naming between the variable and function is confusing
    def get_indexed_state(self) -> bool:
        return self.has_indexed
