import dataclasses
import math
import typing

from phoenix6.configs import (
    ExternalFeedbackConfigs,
    FeedbackConfigs,
    MagnetSensorConfigs,
    MotorOutputConfigs,
)
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from phoenix6.signals import (
    ExternalFeedbackSensorSourceValue,
    FeedbackSensorSourceValue,
    InvertedValue,
    SensorDirectionValue,
)
from phoenix6.sim import (
    CANcoderSimState,
    ChassisReference,
    TalonFXSimState,
    TalonFXSSimState,
)
from wpimath import units
from wpimath.system.plant import DCMotor

from utilities.simulation import EncoderSim, MotorSim

# Freespeed in rev/s
FALCON_FREE_RPS = 100


def _read_config[ConfigT](configurator: typing.Any, config: ConfigT) -> ConfigT:
    """Read a config group back from a Phoenix device."""
    status = configurator.refresh(config)
    if not status.is_ok():
        raise RuntimeError(
            f"Failed to read {type(config).__name__} from device: {status.name}"
        )
    return config


def _chassis_reference(clockwise_positive: bool) -> ChassisReference:
    if clockwise_positive:
        return ChassisReference.CLOCKWISE_POSITIVE
    return ChassisReference.COUNTER_CLOCKWISE_POSITIVE


INVERTED_VALUE_TO_CHASSIS_REFERENCE = {
    InvertedValue.CLOCKWISE_POSITIVE: ChassisReference.CLOCKWISE_POSITIVE,
    InvertedValue.COUNTER_CLOCKWISE_POSITIVE: ChassisReference.COUNTER_CLOCKWISE_POSITIVE,
}


def _talon_sim_state(
    motor: TalonFX | TalonFXS,
) -> TalonFXSimState | TalonFXSSimState:

    motor_output = _read_config(motor.configurator, MotorOutputConfigs())

    orientation = INVERTED_VALUE_TO_CHASSIS_REFERENCE[motor_output.inverted]

    if isinstance(motor, TalonFXS):
        fxs_state = motor.sim_state
        fxs_state.motor_orientation = orientation
        fxs_state.set_supply_voltage(12.0)
        return fxs_state

    fx_state = motor.sim_state
    fx_state.orientation = orientation
    fx_state.set_supply_voltage(12.0)
    return fx_state


_FX_CANCODER_SOURCES = {
    FeedbackSensorSourceValue.REMOTE_CANCODER,
    FeedbackSensorSourceValue.FUSED_CANCODER,
    FeedbackSensorSourceValue.SYNC_CANCODER,
}
_FXS_CANCODER_SOURCES = {
    ExternalFeedbackSensorSourceValue.REMOTE_CANCODER,
    ExternalFeedbackSensorSourceValue.FUSED_CANCODER,
    ExternalFeedbackSensorSourceValue.SYNC_CANCODER,
}


@dataclasses.dataclass(frozen=True)
class _TalonFeedback:
    """The gearing a Talon's feedback config describes."""

    rotor_to_sensor: float
    sensor_to_mechanism: float
    # Device ID of the CANcoder used for feedback, if any.
    cancoder_id: int | None

    @property
    def motor_to_mechanism(self) -> float:
        """Motor rotations per mechanism rotation."""
        # RotorToSensorRatio should be 1.0 when feedback comes from the rotor.
        return self.rotor_to_sensor * self.sensor_to_mechanism


def _talon_feedback(motor: TalonFX | TalonFXS) -> _TalonFeedback:
    if isinstance(motor, TalonFXS):
        ext = _read_config(motor.configurator, ExternalFeedbackConfigs())
        ext_source = ext.external_feedback_sensor_source
        return _TalonFeedback(
            rotor_to_sensor=ext.rotor_to_sensor_ratio
            if ext_source == ExternalFeedbackSensorSourceValue.COMMUTATION
            else 1.0,
            sensor_to_mechanism=ext.sensor_to_mechanism_ratio,
            cancoder_id=(
                ext.feedback_remote_sensor_id
                if ext_source in _FXS_CANCODER_SOURCES
                else None
            ),
        )

    fb = _read_config(motor.configurator, FeedbackConfigs())
    fx_source = fb.feedback_sensor_source
    return _TalonFeedback(
        rotor_to_sensor=fb.rotor_to_sensor_ratio
        if fx_source == FeedbackSensorSourceValue.ROTOR_SENSOR
        else 1.0,
        sensor_to_mechanism=fb.sensor_to_mechanism_ratio,
        cancoder_id=(
            fb.feedback_remote_sensor_id if fx_source in _FX_CANCODER_SOURCES else None
        ),
    )


class TalonMotorSim(MotorSim):
    def __init__(
        self,
        # DCMotor gearbox factory, e.g. DCMotor.falcon500
        gearbox_motor: typing.Callable[[int], DCMotor],
        *motors: TalonFX | TalonFXS,
    ):
        self.gearbox = gearbox_motor(len(motors))
        self.gearing = _talon_feedback(motors[0]).motor_to_mechanism
        self.sim_states = [_talon_sim_state(motor) for motor in motors]

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


SENSOR_DIRECTION_TO_CHASSIS_REFERENCE = {
    SensorDirectionValue.CLOCKWISE_POSITIVE: ChassisReference.CLOCKWISE_POSITIVE,
    SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE: ChassisReference.COUNTER_CLOCKWISE_POSITIVE,
}


def _cancoder_sim_state(encoder: CANcoder) -> CANcoderSimState:

    magnet_sensor = _read_config(encoder.configurator, MagnetSensorConfigs())
    sim_state = encoder.sim_state
    sim_state.sensor_offset = magnet_sensor.magnet_offset
    sim_state.orientation = SENSOR_DIRECTION_TO_CHASSIS_REFERENCE[
        magnet_sensor.sensor_direction
    ]
    return sim_state


class CANcoderSim(EncoderSim):
    @staticmethod
    def from_gearing(encoder: CANcoder, gearing: float) -> CANcoderSim:
        return CANcoderSim(encoder, gearing)

    @staticmethod
    def from_dependant_device(
        encoder: CANcoder, dependant_device: TalonFX | TalonFXS
    ) -> CANcoderSim:
        feedback = _talon_feedback(dependant_device)
        if feedback.cancoder_id != encoder.device_id:
            raise ValueError(
                f"Talon {dependant_device.device_id} does not use "
                f"CANcoder {encoder.device_id} for feedback"
            )
        return CANcoderSim(encoder, feedback.sensor_to_mechanism)

    def __init__(self, encoder: CANcoder, gearing: float) -> None:
        self.sim_state = _cancoder_sim_state(encoder)
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
