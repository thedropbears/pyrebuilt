import math

from magicbot import feedback
from rev import FeedbackSensor, SparkMax, SparkMaxConfig
from wpilib import DutyCycleEncoder, Mechanism2d, SmartDashboard
from wpimath import units

from ids import DioChannel, SparkId
from utilities.rev import (
    configure_spark_reset_and_persist,
    configure_through_bore_encoder,
)


class TurretComponent:
    MOTOR_TO_TURRET_GEARING = (1 / 5) * (25 / 145)
    TURRET_TO_ENCODER_GEARING = (145 / 40) * (16 / 70)

    _ENCODER_OFFSET_DUTY_CYCLE = 0.359977
    ENCODER_OFFSET = _ENCODER_OFFSET_DUTY_CYCLE * math.tau / TURRET_TO_ENCODER_GEARING

    ALLOWABLE_ERROR = math.radians(1.0)

    MAX_VELOCITY = 24.0
    MAX_ACCELERATION = 50.0

    MAX_TURRET_ROTATION = math.radians(200)
    MIN_TURRET_ROTATION = math.radians(-200)
    TURRET_MOTION_RANGE = MAX_TURRET_ROTATION - MIN_TURRET_ROTATION

    def __init__(self) -> None:
        # Initialise Motor
        self.motor = SparkMax(SparkId.TURRET, SparkMax.MotorType.kBrushless)
        config = SparkMaxConfig()
        config.inverted(True)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        config.closedLoop.pid(2.8421, 0.0, 1.3842)
        config.closedLoop.feedForward.kS(0.091518)
        config.closedLoop.feedForward.kV(0.57874)
        config.closedLoop.feedForward.kA(0.067156)
        config.closedLoop.maxMotion.maxAcceleration(TurretComponent.MAX_ACCELERATION)
        config.closedLoop.maxMotion.cruiseVelocity(TurretComponent.MAX_VELOCITY)
        config.closedLoop.maxMotion.allowedProfileError(math.radians(5))
        config.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        config.encoder.positionConversionFactor(
            TurretComponent.MOTOR_TO_TURRET_GEARING * math.tau
        )
        config.encoder.velocityConversionFactor(
            1 / 60 * TurretComponent.MOTOR_TO_TURRET_GEARING * math.tau
        )
        config.closedLoop.allowedClosedLoopError(TurretComponent.ALLOWABLE_ERROR)
        configure_spark_reset_and_persist(self.motor, config)

        self.relative_encoder = self.motor.getEncoder()
        self.controller = self.motor.getClosedLoopController()

        # Initialise Encoder
        self.absolute_encoder = DutyCycleEncoder(
            DioChannel.TURRET_ENCODER,
            math.tau / TurretComponent.TURRET_TO_ENCODER_GEARING,
            0.0,
        )
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
        self.slew_to(self.get_current_angle())

    def on_enable(self) -> None:
        self._sync_encoder()
        self.slew_to(self.get_current_angle())

    def wrap_range(self, target_angle: units.radians) -> units.radians:
        if target_angle < self.MIN_TURRET_ROTATION:
            target_angle += self.TURRET_MOTION_RANGE
        elif target_angle > self.MAX_TURRET_ROTATION:
            target_angle -= self.TURRET_MOTION_RANGE

        return target_angle

    @feedback
    def get_raw_absolute_encoder(self) -> float:
        return self.absolute_encoder.get()

    def _get_absolute_encoder_position(self) -> units.radians:
        return self.absolute_encoder.get() - TurretComponent.ENCODER_OFFSET

    @feedback
    def _get_absolute_encoder_position_degrees(self) -> units.degrees:
        return math.degrees(self._get_absolute_encoder_position())

    def _sync_encoder(self) -> None:
        self.relative_encoder.setPosition(self._get_absolute_encoder_position())

    def get_current_angle(self) -> units.radians:
        return self.relative_encoder.getPosition()

    @feedback
    def get_current_angle_degrees(self) -> units.degrees:
        return math.degrees(self.get_current_angle())

    def get_current_velocity(self) -> units.radians_per_second:
        return self.relative_encoder.getVelocity()

    @feedback
    def get_current_velocity_degrees_per_second(self) -> units.degrees_per_second:
        return math.degrees(self.get_current_velocity())

    def get_error(self) -> units.radians:
        return self.controller.getMAXMotionSetpointPosition() - self.get_current_angle()

    @feedback
    def get_error_degrees(self) -> units.degrees:
        return math.degrees(self.get_error())

    def slew_relative(self, angle: units.radians) -> None:
        self.slew_to(self.get_current_angle() + angle)

    def slew_to(self, angle: units.radians) -> None:
        # update setpoint
        # TODO wrap angle
        self.desired_angle = self.wrap_range(angle)

    def execute(self) -> None:
        self.controller.setSetpoint(
            self.desired_angle, SparkMax.ControlType.kMAXMotionPositionControl
        )

    def periodic(self) -> None:
        self.sim_pointer.setAngle(self.get_current_angle_degrees())
