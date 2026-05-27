import math

from magicbot import feedback, will_reset_to
from phoenix6 import configs, controls, signals
from phoenix6.hardware import TalonFX
from wpimath import units

from components.leds import LEDComponent
from ids import TalonId


class ShooterComponent:
    leds: LEDComponent
    target_shooter_rps = will_reset_to(0.0)

    FLYWHEEL_GEAR_RATIO = 1 / (36 / 24)

    FLYWHEEL_SETPOINT_TOLERANCE = 3.0

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

    @feedback
    def get_flywheel_error(self) -> units.turns_per_second:
        return self.flywheel_motor.get_closed_loop_error().value

    @feedback
    def get_flywheel_speed(self) -> units.turns_per_second:
        return self.flywheel_motor.get_velocity().value

    @feedback
    def flywheel_is_at_speed(self) -> bool:
        return (
            not math.isclose(
                self.flywheel_motor.get_closed_loop_reference().value, 0.0, abs_tol=0.1
            )
            and abs(self.flywheel_motor.get_closed_loop_error().value)
            < self.FLYWHEEL_SETPOINT_TOLERANCE
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
