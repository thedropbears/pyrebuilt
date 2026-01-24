from magicbot import feedback
from rev import ClosedLoopConfig, ClosedLoopSlot, SparkMax, SparkMaxConfig

from ids import SparkId
from utilities.rev import configure_spark_reset_and_persist


class TurretComponent:
    setpoint = 0
    rotation_speed = 0

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.TURRET, SparkMax.MotorType.kBrushless)
        self.motor.ControlType(SparkMax.ControlType.kPosition)

        self.closed_loop_controller = self.motor.getClosedLoopController()

        motor_config = SparkMaxConfig()
        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

        configure_spark_reset_and_persist(self.motor, motor_config)

        pid_config = ClosedLoopConfig()
        pid_config.P(0.1, ClosedLoopSlot.kSlot0)  # TODO Tune this value
        pid_config.I(0, ClosedLoopSlot.kSlot0)  # TODO Tune this value
        pid_config.D(0, ClosedLoopSlot.kSlot0)  # TODO Tune this value
        pid_config.allowedClosedLoopError(
            10, ClosedLoopSlot.kSlot0
        )  # TODO Tune this value
        pid_config.positionWrappingEnabled(True)
        pid_config.positionWrappingInputRange(-1, 1)  # TODO Tune this value

    @feedback
    def raw_encoder_val(self):
        pass

    @feedback
    def current_turret_angle(self):
        pass

    def rotate_to(self, angle):
        pass

    def rotate_by(self, angle):
        pass

    def execute(self):
        pass
