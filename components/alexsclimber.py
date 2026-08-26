import rev
import wpilib


class ClimberComponent:
    def __init__(self):
        self.motor = rev.SparkMax(1, rev.SparkMax.MotorType.kBrushless)
        self.beam_breaker = wpilib.DigitalInput(0)
        self.

    def deploy_arm(self):
        self.motor.set(1.0)

    def retract_arm(self):
        self.motor.set(-1.0)
    
    def sense_scaffold(self):
        

    def execute(self):
        pass

