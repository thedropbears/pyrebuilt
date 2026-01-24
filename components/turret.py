from magicbot import feedback
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig

from ids import SparkId
from utilities.rev import configure_spark_reset_and_persist


class TurretComponent:
    setpoint = 0
    rotation_speed = 0

    def __init__(self) -> None:
        self.motor = SparkMax(SparkId.TURRET, SparkMax.MotorType.kBrushless)
        self.closed_loop_controller = self.motor.getClosedLoopController()

        motor_config = SparkMaxConfig()

        motor_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        motor_config.closedLoop.pid(
            0.01, 0, 0, ClosedLoopSlot.kSlot0
        )  # TODO Tune these values

        motor_config.closedLoop.positionWrappingEnabled(
            True
        ).positionWrappingInputRange(-1, 1)  # TODO Tune these valuse

        configure_spark_reset_and_persist(self.motor, motor_config)

    @feedback
    def raw_encoder_val(self):
        pass

    @feedback
    def current_turret_angle(self):
        pass

    def rotate_to(self, angle):
        self.setpoint = angle

    def rotate_by(self, angle):
        self.setpoint += angle

    def execute(self):
        self.closed_loop_controller.setSetpoint(
            self.setpoint, SparkMax.ControlType.kPosition
        )
