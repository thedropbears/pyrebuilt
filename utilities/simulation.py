import typing

from wpilib.simulation import (
    DCMotorSim,
    SingleJointedArmSim,
)
from wpimath import units
from wpimath.system.plant import DCMotor, LinearSystemId


class MotorSim(typing.Protocol):
    gearbox: DCMotor
    gearing: float

    def get_motor_voltage(self) -> units.volts: ...
    def update_from_mechanism(
        self,
        position: units.radians,
        velocity: units.radians_per_second,
        dt: units.seconds,
    ) -> None: ...


class EncoderSim(typing.Protocol):
    def update_from_mechanism(
        self,
        position: units.radians,
        velocity: units.radians_per_second,
        dt: units.seconds,
    ) -> None: ...


class MechanismSim(typing.Protocol):
    def update(self, motor_voltage: units.volts, dt: units.seconds) -> None: ...

    def get_angular_position(self) -> units.radians: ...
    def get_angular_velocity(self) -> units.radians_per_second: ...


class SimpleMechanism(MechanismSim):
    def __init__(self, motor_sim: MotorSim, moi: units.kilogram_square_meters) -> None:
        self.plant = LinearSystemId.DCMotorSystem(
            motor_sim.gearbox, moi, motor_sim.gearing
        )
        self.mech_sim = DCMotorSim(self.plant, motor_sim.gearbox)

    @typing.override
    def update(self, motor_voltage: float, dt: float) -> None:
        self.mech_sim.setInputVoltage(motor_voltage)
        self.mech_sim.update(dt)

    @typing.override
    def get_angular_position(self) -> units.radians:
        return self.mech_sim.getAngularPosition()

    @typing.override
    def get_angular_velocity(self) -> units.radians_per_second:
        return self.mech_sim.getAngularVelocity()


class ArmMechanism(MechanismSim):
    def __init__(
        self,
        motor_sim: MotorSim,
        moi: units.kilogram_square_meters,
        arm_length: units.meters,
        min_angle: units.radians,
        max_angle: units.radians,
        starting_angle: units.radians,
    ) -> None:
        self.plant_arm = LinearSystemId.singleJointedArmSystem(
            motor_sim.gearbox, moi, motor_sim.gearing
        )
        self.mech_sim = SingleJointedArmSim(
            self.plant_arm,
            motor_sim.gearbox,
            motor_sim.gearing,
            arm_length,
            min_angle,
            max_angle,
            True,
            starting_angle,
        )

    @typing.override
    def update(self, motor_voltage: float, dt: float) -> None:
        self.mech_sim.setInputVoltage(motor_voltage)
        self.mech_sim.update(dt)

    @typing.override
    def get_angular_position(self) -> units.radians:
        return self.mech_sim.getAngle()

    @typing.override
    def get_angular_velocity(self) -> units.radians_per_second:
        return self.mech_sim.getVelocity()


class MotorMechanismSim:
    """A mechanism driven by a motor sim, with any number of external encoders."""

    def __init__(
        self,
        motor_sim: MotorSim,
        mech_sim: MechanismSim,
        encoder_sim: EncoderSim | None = None,
    ) -> None:
        self.motor_sim = motor_sim
        self.mech_sim = mech_sim
        self.encoder_sim = encoder_sim
        # Publish the starting state so robot code reads it before the first tick.
        self._publish(0.0)

    def update(self, dt: units.seconds) -> None:
        self.mech_sim.update(self.motor_sim.get_motor_voltage(), dt)
        self._publish(dt)

    def _publish(self, dt: units.seconds) -> None:
        position = self.mech_sim.get_angular_position()
        velocity = self.mech_sim.get_angular_velocity()
        self.motor_sim.update_from_mechanism(position, velocity, dt)
        if self.encoder_sim:
            self.encoder_sim.update_from_mechanism(position, velocity, dt)
