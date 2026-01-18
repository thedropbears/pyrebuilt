from magicbot import tunable, will_reset_to
from phoenix6 import configs, controls
from phoenix6.controls import Follower
from phoenix6.hardware import TalonFX
from phoenix6.signals import MotorAlignmentValue

from ids import TalonId


class ShooterComponent:
    target_rps = will_reset_to(0.0)
    desired_rps = tunable(30)

    def __init__(self) -> None:
        self.flywheel_motor_left = TalonFX(
            device_id=TalonId.FLYWHEEL_LEFT
        )  # Defined from behind shooter
        self.flywheel_motor_right = TalonFX(
            device_id=TalonId.FLYWHEEL_RIGHT
        )  # Defined from behind shooter

        gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.057491)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.0511005)
            .with_k_v(0.10978)
            .with_k_a(0.0053959)
        )

        self.flywheel_motor_left.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(gains_cfg)
        )

        self.flywheel_motor_right.set_control(
            Follower(
                TalonId.FLYWHEEL_LEFT, MotorAlignmentValue(MotorAlignmentValue.OPPOSED)
            )
        )

    def shoot(self) -> None:
        self.set_rps = self.target_rps

    def execute(self) -> None:
        self.flywheel_motor_left.set_control(controls.VelocityVoltage(self.desired_rps))
