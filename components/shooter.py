import math

from magicbot import feedback, tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from phoenix6 import configs, controls
from phoenix6.controls import Follower
from phoenix6.hardware import TalonFX
from phoenix6.signals import MotorAlignmentValue
from rev import ClosedLoopSlot, FeedbackSensor, SparkMax, SparkMaxConfig

from ids import SparkId, TalonId
from utilities.rev import configure_spark_reset_and_persist


class ShooterComponent:
    hood_move_speed = tunable(0.01)

    target_shooter_rps = will_reset_to(0.0)
    desired_shooter_rps = tunable(30)

    target_feeder_percentage = will_reset_to(0)
    desired_feeder_percentage = tunable(1)

    desired_hood_angle = tunable(60)
    hood_error_tolerance = 5.0
    MAX_HOOD_ANGLE = 70  # TODO Tune this value
    MIN_HOOD_ANGLE = 10  # TODO Tune this value

    MOTOR_GEAR_RATIO = 24 / 22
    ENCODER_ROTS_PER_HOOD_DEGREE = 54 / 26 / 360
    MOTOR_ROTS_PER_HOOD_DEGREE = MOTOR_GEAR_RATIO * ENCODER_ROTS_PER_HOOD_DEGREE
    ENCODER_ZERO_OFFSET = 0  # TODO Tune this value

    def __init__(self) -> None:
        self.flywheel_motor_left = TalonFX(
            device_id=TalonId.FLYWHEEL_LEFT
        )  # Defined from behind shooter
        self.flywheel_motor_right = TalonFX(
            device_id=TalonId.FLYWHEEL_RIGHT
        )  # Defined from behind shooter

        flywheel_gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.036653)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.086321)
            .with_k_v(0.11159)
            .with_k_a(0.0038097)
        )

        self.flywheel_motor_left.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(flywheel_gains_cfg)
        )

        self.feeder_motor = TalonSRX(TalonId.FEEDER)
        self.feeder_motor.setInverted(False)

        self.hood_motor = SparkMax(SparkId.HOOD, SparkMax.MotorType.kBrushless)
        self.hood_motor_controller = self.hood_motor.getClosedLoopController()

        hood_motor_cfg = SparkMaxConfig()
        hood_motor_cfg.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        hood_motor_cfg.closedLoop.pid(
            0.01, 0, 0, ClosedLoopSlot.kSlot1
        )  # TODO Tune these values
        hood_motor_cfg.closedLoop.allowedClosedLoopError(self.hood_error_tolerance)
        hood_motor_cfg.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)

        configure_spark_reset_and_persist(self.hood_motor, hood_motor_cfg)

        self.hood_encoder = self.hood_motor.getAbsoluteEncoder()
        hood_encoder_cfg = hood_motor_cfg.absoluteEncoder  # TODO Re-check this line
        hood_encoder_cfg.zeroOffset(self.ENCODER_ZERO_OFFSET)
        hood_encoder_cfg.positionConversionFactor(self.ENCODER_ROTS_PER_HOOD_DEGREE)

    @feedback
    def raw_encoder_angle(self):
        return self.hood_encoder.getPosition()

    @feedback
    def get_hood_angle(self):
        return self.hood_encoder.getPosition()

    @feedback
    def hood_is_at_setpoint(self):
        return math.isclose(
            self.get_hood_angle(),
            self.desired_hood_angle,
            rel_tol=self.hood_error_tolerance,
        )

    def change_pitch_relative(self, angle):
        self.desired_hood_angle = self.get_hood_angle() - angle

    def change_pitch_absolute(self, angle):
        self.desired_hood_angle = 90 - angle

    def shoot(self) -> None:
        self.target_shooter_rps = self.desired_shooter_rps
        self.target_feeder_percentage = self.desired_feeder_percentage

    def execute(self) -> None:
        self.flywheel_motor_left.set_control(
            controls.VelocityVoltage(self.target_shooter_rps)
        )
        self.flywheel_motor_right.set_control(
            Follower(
                TalonId.FLYWHEEL_LEFT, MotorAlignmentValue(MotorAlignmentValue.OPPOSED)
            )
        )

        self.feeder_motor.set(ControlMode.PercentOutput, self.target_feeder_percentage)

        self.hood_motor.set(self.hood_move_speed)

        # TODO This code is commented until basic hood function has been tested
        """
        self.hood_motor_controller.setSetpoint(
            self.desired_hood_angle * self.MOTOR_GEAR_RATIO,
            SparkMax.ControlType.kPosition,
        )
        """
