from math import isclose, radians

from magicbot import feedback, will_reset_to
from phoenix6.configs import Slot0Configs, TalonFXConfiguration
from phoenix6.controls import DutyCycleOut, PositionVoltage
from phoenix6.hardware import TalonFX, cancoder
from wpilib import Mechanism2d, SmartDashboard
from wpimath import units

from ids import DioChannel, TalonId


class IntakeComponent:
    target_roller_rps = will_reset_to(units.turns_per_second(0))
    target_intake_angle = will_reset_to(units.degrees(0))
    RETRACTED_INTAKE_ANGLE = units.degrees(0)
    DEPLOYED_INTAKE_ANGLE = units.degrees(75)
    DESIRED_ROLLER_VOLTAGE = DutyCycleOut(1)
    DEPLOYER_TO_CANCODER_GEARING = (1 / 5) * (26 / 50)
    ENCODER_ZERO_OFFSET = -0.486328125

    ARM_LENGTH = 0.34
    ARM_MOI = 0.21313981

    def __init__(self) -> None:
        self.intake_deployer = TalonFX(TalonId.INTAKE_DEPLOYER)
        self.intake_roller = TalonFX(TalonId.INTAKE_ROLLER)
        self.deployer_encoder = cancoder.CANcoder(DioChannel.INTAKE_DEPLOYER_ENCODER)
        slot0_configs = Slot0Configs()
        slot0_configs.with_k_p(60.0)
        slot0_configs.with_k_i(0)
        slot0_configs.with_k_d(3.0)
        self.intake_deployer.configurator.apply(
            TalonFXConfiguration().with_slot0(slot0_configs)
        )
        self.request = PositionVoltage(0).with_slot(0)
        self.mech = Mechanism2d(3, 3)
        self.root = self.mech.getRoot("intake", 0, 2)
        self.rotating_arm = self.root.appendLigament("rotating arm", 6, 90)
        SmartDashboard.putData("Mech 2D", self.mech)
        roller_configs = Slot0Configs()
        roller_configs.with_k_p(1)
        roller_configs.with_k_i(0)
        roller_configs.with_k_d(1)
        self.intake_roller.configurator.apply(
            TalonFXConfiguration().with_slot0(roller_configs)
        )

    def retract(self):
        self.target_intake_angle = self.RETRACTED_INTAKE_ANGLE

    def deploy(self):
        self.target_intake_angle = self.DEPLOYED_INTAKE_ANGLE

    def intake(self):
        pass

    @feedback
    def get_intake_angle(self):
        return self.deployer_encoder.get_absolute_position().value

    @feedback
    def get_target_intake_angle(self):
        return self.target_intake_angle

    def periodic(self):
        self.rotating_arm.setAngle(self.get_intake_angle())

    def is_retracted(self):
        return isclose(
            self.target_intake_angle, self.RETRACTED_INTAKE_ANGLE, abs_tol=0.1
        ) and isclose(
            self.get_intake_angle(),
            self.RETRACTED_INTAKE_ANGLE,
            abs_tol=radians(10),
        )

    def drive(self):
        # fill in later
        return

    def outtake(self):
        # fill in later
        return

    def execute(self):
        self.rotating_arm.setAngle(self.target_intake_angle)
        self.intake_deployer.set_control(
            self.request.with_position(self.target_intake_angle)
        )
        self.intake_roller.set_control(
            self.request.with_position(self.target_intake_angle)
        )
