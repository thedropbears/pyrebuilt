from magicbot import feedback, will_reset_to
from phoenix6.configs import Slot0Configs, TalonFXConfiguration
from phoenix6.controls import DutyCycleOut, PositionVoltage
from phoenix6.hardware import TalonFX
from wpilib import AnalogEncoder, Mechanism2d, SmartDashboard
from wpimath import units

from ids import DioChannel, TalonId


class NewIntakeComponent:
    target_roller_rps = will_reset_to(units.turns_per_second(0))
    target_intake_angle = will_reset_to(units.degrees(0))
    RETRACTED_INTAKE_ANGLE = units.degrees(0)
    DEPLOYED_INTAKE_ANGLE = units.degrees(75)
    DESIRED_ROLLER_VOLTAGE = DutyCycleOut(1)

    def __init__(self) -> None:
        self.intake_deployer = TalonFX(TalonId.INTAKE_DEPLOYER)
        self.intake_roller = TalonFX(TalonId.INTAKE_ROLLER)
        self.encoder = AnalogEncoder(DioChannel.INTAKE_DEPLOYER_ENCODER)
        slot0_configs = Slot0Configs()
        slot0_configs.with_k_p(1)
        slot0_configs.with_k_i(0)
        slot0_configs.with_k_d(1)
        self.intake_deployer.configurator.apply(
            TalonFXConfiguration().with_slot0(slot0_configs)
        )
        self.request = PositionVoltage(0).with_slot(0)
        self.mech = Mechanism2d(3, 3)
        self.root = self.mech.getRoot("intake", 0, 2)
        self.rotating_arm = self.root.appendLigament("rotating arm", 6, 90)
        SmartDashboard.putData("Practice Mech 2D", self.mech)

    def retract(self):
        self.target_intake_angle = self.RETRACTED_INTAKE_ANGLE

    def deploy(self):
        self.target_intake_angle = self.DEPLOYED_INTAKE_ANGLE

    def intake(self):
        pass

    @feedback
    def get_intake_angle(self):
        return self.encoder.get()

    def execute(self):
        self.rotating_arm.setAngle(self.target_intake_angle)
        self.intake_deployer.set_control(
            self.request.with_position(self.target_intake_angle)
        )
        self.intake_roller.set_control(self.DESIRED_ROLLER_VOLTAGE)
