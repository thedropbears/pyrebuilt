from magicbot import tunable, will_reset_to
from rev import SparkMax, SparkMaxConfig

from ids import SparkId
from utilities.rev import configure_spark_reset_and_persist


class ClimberComponent:
    setpoint = will_reset_to(0.0)
    climb_speed = tunable(0.2)

    kP, kI, kD = 0, 0, 0
    kMinOutput, kMaxOutput = 0, 0

    def __init__(self):
        # create motor with correct forward direction sparkmax controller
        self.motor = SparkMax(SparkId.CLIMBER, SparkMax.MotorType.kBrushless)
        self.controller = self.motor.getClosedLoopController()

        config = SparkMaxConfig()
        config.inverted(True)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        configure_spark_reset_and_persist(self.motor, config)

        config.closedLoop.pid(self.kP, self.kI, self.kD)
        config.closedLoop.outputRange(self.kMinOutput, self.kMaxOutput)

    def deploy(self):
        self.setpoint = -self.climb_speed

    def climb(self):
        self.setpoint = self.climb_speed

    def execute(self):
        self.controller.setSetpoint(self.setpoint, SparkMax.ControlType.kPosition)