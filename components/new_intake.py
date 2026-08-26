from magicbot import feedback, will_reset_to
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from wpilib import AnalogEncoder
from wpimath import units
from wpimath.controller import PIDController

from ids import DioChannel, TalonId


class IntakeComponent:
    target_roller_rps = will_reset_to(units.turns_per_second(0))
    target_intake_angle = will_reset_to(units.degrees(0))

    def __init__(self) -> None:
        self.motor = TalonFX(TalonId.INTAKE_DEPLOYER)
        self.retracted_intake_Angle = units.degrees(0)
        self.deployed_intake_Angle = units.degrees(75)
        self.pid = PIDController(1, 0, 1)
        self.encoder = AnalogEncoder(DioChannel.INTAKE_DEPLOYER_ENCODER)

    def retract(self):
        pass

    def extend(self):
        pass

    def intake(self):
        pass

    @feedback
    def get_intake_angle(self):
        return self.encoder.get()

    def execute(self):
        effort = self.pid.calculate(self.target_intake_angle)
        voltage = VoltageOut(effort)
        self.motor.set_control(voltage)
