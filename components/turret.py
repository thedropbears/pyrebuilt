from magicbot import tunable
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    ExternalFeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
    TalonFXSConfiguration,
)
from phoenix6.controls import PositionVoltage
from phoenix6.hardware import TalonFXS
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpilib import DutyCycleEncoder
from wpimath import units

from utilities.rev import configure_through_bore_encoder


class TurretComponent:
    MOTOR_TO_TURRET_GEARING = 1 / (40 / 200)
    TURRET_TO_ENCODER_GEARING = 1 / ((200 / 50) * (20 / 80))
    MOTOR_TO_ENCODER_GEARING = MOTOR_TO_TURRET_GEARING * TURRET_TO_ENCODER_GEARING

    desired_turret_angle = tunable(0.0)

    def __init__(self) -> None:
        # Initialise Motor
        self.motor = TalonFXS(1)
        motor_config = self.motor.configurator
        motor_config.apply(
            TalonFXSConfiguration()
            .with_motor_output(
                MotorOutputConfigs()
                .with_neutral_mode(NeutralModeValue.BRAKE)
                .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            )
            .with_closed_loop_general(
                ClosedLoopGeneralConfigs().with_continuous_wrap(True)
            )
            .with_external_feedback(
                ExternalFeedbackConfigs().with_sensor_to_mechanism_ratio(
                    TurretComponent.MOTOR_TO_TURRET_GEARING
                )
            )
            .with_slot0(Slot0Configs().with_k_p(0.1))
        )

        # Initialise Encoder
        self.absolute_encoder = DutyCycleEncoder(1)
        configure_through_bore_encoder(self.absolute_encoder)
        self.absolute_encoder.setInverted(False)

    def slew_relative(self, angle: units.radians) -> None:
        # TODO Implement this
        # TODO update setpoint
        pass

    def slew_to_local(self, angle: units.radians) -> None:
        # TODO Implement this
        # update setpoint
        pass

    def execute(self) -> None:
        # TODO Implement this
        # wrap angle

        # run control cycle
        self.motor.set_control(PositionVoltage(self.desired_turret_angle))
