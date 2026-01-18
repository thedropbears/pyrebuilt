from magicbot import tunable, will_reset_to
from phoenix6 import configs, controls
from phoenix6.hardware import TalonFX

from ids import TalonId


class ShooterComponent:
    set_rps = will_reset_to(0.0)
    desired_rps = tunable(1)

    k_p = 0.057491
    k_i = 0
    k_d = 0
    k_s = 0.0511005
    k_v = 0.10978
    k_a = 0.0053959

    def __init__(self) -> None:
        self.flywheel_motor = TalonFX(device_id=TalonId.FLYWHEEL)
        flywheel_cfg = configs.TalonFXConfiguration()

        self.velocity_voltage = controls.VelocityVoltage(0).with_slot(0)

        flywheel_cfg.slot0.k_p = self.k_p
        flywheel_cfg.slot0.k_i = self.k_i
        flywheel_cfg.slot0.k_d = self.k_d
        flywheel_cfg.slot0.k_s = self.k_s
        flywheel_cfg.slot0.k_v = self.k_v
        flywheel_cfg.slot0.k_a = self.k_a

        flywheel_cfg.voltage.peak_forward_voltage = 12
        flywheel_cfg.voltage.peak_reverse_voltage = -12

        self.flywheel_motor.configurator.apply(flywheel_cfg)

    def shoot(self) -> None:
        self.set_rps = self.desired_rps

    def execute(self) -> None:
        self.flywheel_motor.set_control(
            self.velocity_voltage.with_velocity(self.set_rps)
        )
