import typing

import rev
import wpilib
from wpimath import units
from wpimath.system.plant import DCMotor

from utilities.simulation import MotorSim


def configure_spark_ephemeral(motor: rev.SparkBase, config: rev.SparkBaseConfig):
    motor.configure(
        config,
        rev.ResetMode.kNoResetSafeParameters,
        rev.PersistMode.kNoPersistParameters,
    )


def configure_spark_reset_and_persist(
    motor: rev.SparkBase, config: rev.SparkBaseConfig
):
    motor.configure(
        config,
        rev.ResetMode.kResetSafeParameters,
        rev.PersistMode.kPersistParameters,
    )


def configure_through_bore_encoder(
    enc: wpilib.DutyCycleEncoder, freq: float = 975.6
) -> None:
    """Configure a REV Through Bore Encoder absolute pulse output.

    This avoids the roboRIO duty cycle frequency computation, which will
    be inaccurate at startup.

    Args:
        enc: The DutyCycleEncoder to configure.
        freq: The assumed frequency of the encoder in Hz. Defaults to
            the encoder's nominal frequency per the datasheet.
    """
    enc.setAssumedFrequency(freq)
    enc.setDutyCycleRange(1 / 1025, 1024 / 1025)


class SparkMotorSim(MotorSim):
    def __init__(
        self,
        gearbox_motor: typing.Callable[[int], DCMotor],
        *motors: rev.SparkMax,
        # Reduction between motor and mechanism rotations, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
    ):
        self.gearbox = gearbox_motor(len(motors))
        self.gearing = gearing
        self.sim_states = [rev.SparkSim(motor, self.gearbox) for motor in motors]

    @typing.override
    def get_motor_voltage(self) -> units.volts:
        sim_state = self.sim_states[0]
        return sim_state.getBusVoltage() * sim_state.getAppliedOutput()

    @typing.override
    def update_from_mechanism(
        self,
        position: units.radians,
        velocity: units.radians_per_second,
        dt: units.seconds,
    ) -> None:
        for sim_state in self.sim_states:
            sim_state.iterate(velocity, self.get_bus_voltage(), dt)

    def get_bus_voltage(self) -> units.volts:
        return self.sim_states[0].getBusVoltage()
