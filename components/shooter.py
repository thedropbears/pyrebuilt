from magicbot import tunable, will_reset_to
from phoenix6 import configs, controls
from phoenix6.hardware import TalonFX

from ids import TalonId


class ShooterComponent:
    target_rps = will_reset_to(0.0)
    desired_rps = tunable(30)

    def __init__(self) -> None:
        self.flywheel_motor = TalonFX(device_id=TalonId.FLYWHEEL)

        gains_cfg = (
            configs.Slot0Configs()
            .with_k_p(0.057491)
            .with_k_i(0)
            .with_k_d(0)
            .with_k_s(0.0511005)
            .with_k_v(0.10978)
            .with_k_a(0.0053959)
        )

        self.flywheel_motor.configurator.apply(
            configs.TalonFXConfiguration().with_slot0(gains_cfg)
        )

    def shoot(self) -> None:
        self.set_rps = self.target_rps

    def execute(self) -> None:
        self.flywheel_motor.set_control(controls.VelocityVoltage(self.desired_rps))
