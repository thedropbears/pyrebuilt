import math

import wpilib
from magicbot import feedback, will_reset_to
from phoenix6 import configs, controls, signals
from phoenix6.configs import CANcoderConfiguration, MagnetSensorConfigs
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import SensorDirectionValue
from wpilib import PWM
from wpimath import units
from wpimath.controller import PIDController

from ids import CancoderId, PwmChannel, TalonId
from utilities.functions import clamp

hood_controller = PIDController(100.0, 0.0, 0.0)

wpilib.SmartDashboard.putData("Hood PID", hood_controller)


class ShooterComponent:
    target_shooter_rps = will_reset_to(0.0)

    hood_error_tolerance = math.radians(1.0)
    # 0 deg   = shooting straight up
    # 90 deg  = shooting horizontal
    # mechanical range is currently 23 - 50 deg in this frame
    MIN_HOOD_ANGLE: units.turns = 25.0 / 360.0
    MAX_HOOD_ANGLE: units.turns = 52.0 / 360.0

    ENCODER_ZERO_OFFSET = -0.168045

    HOOD_SERVO_MAX_SPEED: units.turns_per_second = (
        55.0 / 60.0
    )  # rot/s https://www.amazon.com.au/Digital-Servo-Continuous-Rotation-Metal/dp/B0DNM1BFCR?source=ps-sl-shoppingads-lpcontext&psc=1&smid=A3LYAXKT5J9O5W

    FLYWHEEL_GEAR_RATIO = 1 / (36 / 24)

    def __init__(self) -> None:
        self.flywheel_motor = TalonFX(device_id=TalonId.FLYWHEEL)

        motor_output_config = (
            configs.MotorOutputConfigs()
            .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
            .with_neutral_mode(signals.NeutralModeValue.COAST)
        )

        flywheel_gains_cfg = (
            configs.Slot0Configs()
            .with_k_s(0.19674)
            .with_k_v(0.078402)
            .with_k_a(0.0069887)
            .with_k_p(0.33451)
        )
        feedback_config = configs.FeedbackConfigs().with_sensor_to_mechanism_ratio(
            self.FLYWHEEL_GEAR_RATIO
        )
        self.flywheel_motor.configurator.apply(
            configs.TalonFXConfiguration()
            .with_slot0(flywheel_gains_cfg)
            .with_feedback(feedback_config)
            .with_motor_output(motor_output_config)
        )

        self.hood_servo = PWM(PwmChannel.HOOD_SERVO)
        self.hood_servo.setBounds(2000, 1550, 1500, 1450, 1000)
        self.hood_servo.setPeriodMultiplier(PWM.PeriodMultiplier.kPeriodMultiplier_1X)

        self.hood_encoder = CANcoder(CancoderId.HOOD)
        self.hood_encoder.configurator.apply(
            CANcoderConfiguration().with_magnet_sensor(
                MagnetSensorConfigs()
                .with_absolute_sensor_discontinuity_point(0.5)
                .with_sensor_direction(SensorDirectionValue.CLOCKWISE_POSITIVE)
                .with_magnet_offset(self.ENCODER_ZERO_OFFSET)
            )
        )

        self.target_hood_angle = self.get_hood_angle()

    def on_enable(self) -> None:
        self.target_hood_angle = self.get_hood_angle()

    @feedback
    def get_hood_angle_degrees(self) -> units.degrees:
        return math.degrees(self.get_hood_angle())

    def get_hood_angle(self) -> units.radians:
        return self.get_hood_angle_rotations() * math.tau

    def get_hood_angle_rotations(self) -> units.turns:
        return self.hood_encoder.get_position().value

    @feedback
    def hood_is_at_setpoint(self) -> bool:
        return math.isclose(
            self.get_hood_angle(),
            self.target_hood_angle,
            abs_tol=self.hood_error_tolerance,
        )

    @feedback
    def get_flywheel_error(self) -> units.turns_per_second:
        return self.flywheel_motor.get_closed_loop_error().value

    def pitch_relative(self, angle: units.radians):
        self.pitch_to(self.target_hood_angle + angle)

    def pitch_to(self, angle: units.radians):
        self.target_hood_angle = angle

        self.target_hood_angle = clamp(
            self.target_hood_angle / math.tau, self.MIN_HOOD_ANGLE, self.MAX_HOOD_ANGLE
        )

    def set_flywheel(self, speed: units.turns_per_second):
        self.target_shooter_rps = speed

    def execute(self) -> None:
        if self.target_shooter_rps != 0.0:
            self.flywheel_motor.set_control(
                controls.VelocityVoltage(self.target_shooter_rps)
            )
        else:
            self.flywheel_motor.set_control(controls.CoastOut())

        self.hood_velocity = (
            clamp(
                hood_controller.calculate(
                    self.get_hood_angle_rotations(), self.target_hood_angle
                ),
                -ShooterComponent.HOOD_SERVO_MAX_SPEED,
                ShooterComponent.HOOD_SERVO_MAX_SPEED,
            )
            / ShooterComponent.HOOD_SERVO_MAX_SPEED
        )
        self.hood_servo.setSpeed(self.hood_velocity)
