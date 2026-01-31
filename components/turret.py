import math

from magicbot import feedback
from rev import SparkMax, SparkMaxConfig
from wpilib import DutyCycleEncoder, Mechanism2d, SmartDashboard
from wpimath import units

from ids import DioChannel, SparkId
from utilities.rev import (
    configure_spark_reset_and_persist,
    configure_through_bore_encoder,
)


class TurretComponent:
    MOTOR_TO_TURRET_GEARING = 25 / 145
    TURRET_TO_ENCODER_GEARING = (145 / 40) * (16 / 70)

    ENCODER_OFFSET = 0.359977

    MAX_VELOCITY = 1.0
    MAX_ACCELERATION = 1.0

    def __init__(self) -> None:
        # Initialise Motor
        self.motor = SparkMax(SparkId.TURRET, SparkMax.MotorType.kBrushless)
        config = SparkMaxConfig()
        config.inverted(True)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        config.closedLoop.pid(1.0, 0.0, 0.0)
        config.closedLoop.maxMotion.maxAcceleration(TurretComponent.MAX_ACCELERATION)
        config.closedLoop.maxMotion.cruiseVelocity(TurretComponent.MAX_VELOCITY)
        config.closedLoop.maxMotion.allowedClosedLoopError(math.radians(5))
        config.encoder.positionConversionFactor(
            TurretComponent.MOTOR_TO_TURRET_GEARING * math.tau
        )
        config.encoder.velocityConversionFactor(
            1 / 60 * TurretComponent.MOTOR_TO_TURRET_GEARING * math.tau
        )
        configure_spark_reset_and_persist(self.motor, config)

        self.relative_encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        # Initialise Encoder
        self.absolute_encoder = DutyCycleEncoder(DioChannel.TURRET_ENCODER)
        configure_through_bore_encoder(self.absolute_encoder)
        self.absolute_encoder.setInverted(True)

        self.desired_angle = 0.0

        mech = Mechanism2d(2, 2)
        SmartDashboard.putData("Turret", mech)
        mech_root = mech.getRoot("Turret", 1, 1)
        self.sim_pointer = mech_root.appendLigament(
            "pointer", length=1, angle=90, lineWidth=3
        )

    def setup(self) -> None:
        self._sync_encoder()
        self.slew_to(self.current_angle())

    def on_enable(self) -> None:
        self._sync_encoder()
        self.slew_to(self.current_angle())

    @feedback
    def raw_absolute_encoder(self) -> float:
        return self.absolute_encoder.get()

    @feedback
    def _get_absolute_encoder_position(self) -> units.radians:
        return (
            (self.absolute_encoder.get() - TurretComponent.ENCODER_OFFSET)
            / TurretComponent.TURRET_TO_ENCODER_GEARING
            * math.tau
        )

    def _sync_encoder(self) -> None:
        self.relative_encoder.setPosition(self._get_absolute_encoder_position())

    @feedback
    def current_angle(self) -> units.radians:
        return self.relative_encoder.getPosition()

    @feedback
    def current_velocity(self) -> units.radians_per_second:
        return self.relative_encoder.getVelocity()

    @feedback
    def error(self) -> units.radians:
        return self.controller.getSetpoint() - self.current_angle()

    def slew_relative(self, angle: units.radians) -> None:
        self.slew_to(self.current_angle() + angle)

    def slew_to(self, angle: units.radians) -> None:
        # update setpoint
        # TODO wrap angle
        self.desired_angle = angle

    def execute(self) -> None:
        self.controller.setReference(
            self.desired_angle, SparkMax.ControlType.kMAXMotionPositionControl
        )

    def perioidic(self) -> None:
        self.sim_pointer.setAngle(math.degrees(self.current_angle()))
