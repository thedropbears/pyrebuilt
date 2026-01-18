from magicbot import tunable, will_reset_to
from rev import SparkMax, SparkMaxConfig

from ids import SparkId
from utilities.rev import configure_spark_reset_and_persist


class ClimberComponent:
    setpoint = will_reset_to(0.0)
    climb_speed = tunable(0.2)

    def __init__(self):
        # create motor with correct forward direction sparkmax controller
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)
        config = SparkMaxConfig()
        config.inverted(True)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        configure_spark_reset_and_persist(self.motor, config)

    def deploy(self):
        self.setpoint = -self.climb_speed

    def climb(self):
        self.setpoint = self.climb_speed

    def execute(self):
        self.motor.set(self.setpoint)
