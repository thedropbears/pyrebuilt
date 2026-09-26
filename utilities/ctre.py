import math
import typing

from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from wpimath import units
from wpimath.system.plant import DCMotor

from utilities.simulation import EncoderSim, MotorSim

# Freespeed in rev/s
FALCON_FREE_RPS = 100


class TalonFXMotorSim(MotorSim):
    def __init__(
        self,
        # DCMotor gearbox factory, e.g. DCMotor.falcon500
        gearbox_motor: typing.Callable[[int], DCMotor],
        *motors: TalonFX | TalonFXS,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
    ):
        self.gearbox = gearbox_motor(len(motors))
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)

    @typing.override
    def get_motor_voltage(self) -> units.volts:
        return self.sim_states[0].motor_voltage

    @typing.override
    def update_from_mechanism(
        self,
        position: units.radians,
        velocity: units.radians_per_second,
        dt: units.seconds,
    ) -> None:
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(position * motor_rev_per_mechanism_rad)
            sim_state.set_rotor_velocity(velocity * motor_rev_per_mechanism_rad)


class CANcoderSim(EncoderSim):
    def __init__(
        self,
        encoder: CANcoder,
        offset: float,
        # Encoder rotations per mechanism rotation.
        # One when the CANcoder is mounted directly on the mechanism's axis.
        gearing: float,
    ) -> None:
        self.sim_state = encoder.sim_state
        self.sim_state.sensor_offset = offset
        self.gearing = gearing

    @typing.override
    def update_from_mechanism(
        self,
        position: units.radians,
        velocity: units.radians_per_second,
        dt: units.seconds,
    ) -> None:
        encoder_rev_per_mechanism_rad = self.gearing / math.tau
        self.sim_state.set_raw_position(position * encoder_rev_per_mechanism_rad)
        self.sim_state.set_velocity(velocity * encoder_rev_per_mechanism_rad)
