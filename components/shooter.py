from magicbot import tunable, will_reset_to
from phoenix5 import ControlMode, TalonSRX
from phoenix6 import configs, controls
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, MotorAlignmentValue

from ids import TalonId


class ShooterComponent:
    target_shooter_rps = will_reset_to(0.0)
    desired_shooter_rps = tunable(30)

    target_feeder_percentage = will_reset_to(0)
    desired_feeder_percentage = tunable(1)

    def __init__(self) -> None:
        self.flywheel_motor_left = TalonFX(TalonId.FLYWHEEL_LEFT)
        self.flywheel_motor_right = TalonFX(TalonId.FLYWHEEL_RIGHT)
        self.feeder_motor = TalonSRX(TalonId.FEEDER)

        gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.036653)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.086321)
            .with_k_v(0.11159)
            .with_k_a(0.0038097)
        )

        self.flywheel_motor_left.configurator.apply(
            configs.TalonFXConfiguration()
            .with_slot0(gains_cfg)
            .with_motor_output(
                configs.MotorOutputConfigs().with_inverted(
                    InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                )
            )
        )

    def shoot(self) -> None:
        self.target_shooter_rps = self.desired_shooter_rps
        self.target_feeder_percentage = self.desired_feeder_percentage

    def execute(self) -> None:
        self.flywheel_motor_left.set_control(
            controls.VelocityVoltage(self.target_shooter_rps)
        )
        self.flywheel_motor_right.set_control(
            controls.Follower(
                TalonId.FLYWHEEL_LEFT, MotorAlignmentValue(MotorAlignmentValue.OPPOSED)
            )
        )
        self.feeder_motor.set(ControlMode.PercentOutput, self.target_feeder_percentage)
