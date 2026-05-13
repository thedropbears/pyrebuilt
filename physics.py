from __future__ import annotations

import math
import statistics
import typing
from collections.abc import Callable
from typing import override

import phoenix6
import rev
import wpilib
from phoenix6.swerve.sim_swerve_drivetrain import SimSwerveDrivetrain
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import (
    DCMotorSim,
    DutyCycleEncoderSim,
    PWMSim,
    RoboRioSim,
    SingleJointedArmSim,
)
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.system.plant import DCMotor, LinearSystemId

from swerves.comp import TunerConstants
from utilities import game
from utilities.functions import constrain_angle

if typing.TYPE_CHECKING:
    from robot import MyRobot


class RollingBuffer:
    def __init__(self, max_length: int):
        self.max_length = max_length

        self.buffer_: list[float] = []

    def average(self) -> float:
        return statistics.fmean(self.buffer_) if self.buffer_ else 0.0

    def add_sample(self, sample: float):
        self.buffer_.append(sample)

        if len(self.buffer_) > self.max_length:
            self.buffer_.pop(0)


class SimpleTalonFXMotorSim:
    def __init__(
        self, motor: phoenix6.hardware.TalonFX, units_per_rev: float, kV: float
    ) -> None:
        self.sim_state = motor.sim_state
        self.sim_state.set_supply_voltage(12.0)
        self.kV = kV  # volt seconds per unit
        self.units_per_rev = units_per_rev
        self.voltage_buffer = RollingBuffer(10)

    def update(self, dt: units.seconds) -> None:
        voltage = self.sim_state.motor_voltage

        self.voltage_buffer.add_sample(voltage)

        if math.isclose(self.voltage_buffer.average(), 0.0, abs_tol=0.1):
            voltage = 0.0

        velocity = voltage / self.kV  # units per second
        velocity_rps = velocity * self.units_per_rev
        self.sim_state.set_rotor_velocity(velocity_rps)
        self.sim_state.add_rotor_position(velocity_rps * dt)


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


class TalonFXMotorSim(MotorSim):
    def __init__(
        self,
        # DCMotor gearbox factory, e.g. DCMotor.falcon500
        gearbox_motor: Callable[[int], DCMotor],
        *motors: phoenix6.hardware.TalonFX | phoenix6.hardware.TalonFXS,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
    ):
        self.gearbox = gearbox_motor(len(motors))
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)

    @override
    def get_motor_voltage(self) -> units.volts:
        return self.sim_states[0].motor_voltage

    @override
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


class SparkMotorSim(MotorSim):
    def __init__(
        self,
        gearbox_motor: Callable[[int], DCMotor],
        *motors: rev.SparkMax,
        # Reduction between motor and mechanism rotations, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
    ):
        self.gearbox = gearbox_motor(len(motors))
        self.gearing = gearing
        self.sim_states = [rev.SparkSim(motor, self.gearbox) for motor in motors]

    @override
    def get_motor_voltage(self) -> units.volts:
        sim_state = self.sim_states[0]
        return sim_state.getBusVoltage() * sim_state.getAppliedOutput()

    @override
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


class MechanismSim(typing.Protocol):
    def update(self, motor_voltage: units.volts, dt: units.seconds) -> None: ...

    def get_angular_position(self) -> units.radians: ...
    def get_angular_velocity(self) -> units.radians_per_second: ...


class SimpleMechanism(MechanismSim):
    def __init__(
        self, gearbox: DCMotor, moi: units.kilogram_square_meters, gearing: float
    ) -> None:
        self.plant = LinearSystemId.DCMotorSystem(gearbox, moi, gearing)
        self.mech_sim = DCMotorSim(self.plant, gearbox)

    @override
    def update(self, motor_voltage: float, dt: float) -> None:
        self.mech_sim.setInputVoltage(motor_voltage)
        self.mech_sim.update(dt)

    @override
    def get_angular_position(self) -> units.radians:
        return self.mech_sim.getAngularPosition()

    def get_angular_velocity(self) -> units.radians_per_second:
        return self.mech_sim.getAngularVelocity()


class ArmMechanism(MechanismSim):
    def __init__(
        self,
        gearbox: DCMotor,
        moi: units.kilogram_square_meters,
        gearing: float,
        arm_length: units.meters,
        min_angle: units.radians,
        max_angle: units.radians,
        starting_angle: units.radians,
    ) -> None:
        self.plant_arm = LinearSystemId.singleJointedArmSystem(gearbox, moi, gearing)
        self.mech_sim = SingleJointedArmSim(
            self.plant_arm,
            gearbox,
            gearing,
            arm_length,
            min_angle,
            max_angle,
            True,
            starting_angle,
        )

    @override
    def update(self, motor_voltage: float, dt: float) -> None:
        self.mech_sim.setInputVoltage(motor_voltage)
        self.mech_sim.update(dt)

    @override
    def get_angular_position(self) -> units.radians:
        return self.mech_sim.getAngle()

    @override
    def get_angular_velocity(self) -> units.radians_per_second:
        return self.mech_sim.getVelocity()


class SteerModuleSim:
    def __init__(
        self,
        motor_sim: MotorSim,
        moi: units.kilogram_square_meters,
    ):
        self.motor_sim = motor_sim
        self.mech_sim = SimpleMechanism(
            self.motor_sim.gearbox, moi, self.motor_sim.gearing
        )

    def update(self, dt: units.seconds):
        self.mech_sim.update(self.motor_sim.get_motor_voltage(), dt)
        self.motor_sim.update_from_mechanism(
            self.mech_sim.get_angular_position(),
            self.mech_sim.get_angular_velocity(),
            dt,
        )


class TurretSim:
    def __init__(
        self,
        motor_sim: MotorSim,
        moi: units.kilogram_square_meters,
        encoder: phoenix6.hardware.CANcoder,
        encoder_offset: float,
    ):
        self.motor_sim = motor_sim
        self.mech_sim = SimpleMechanism(
            self.motor_sim.gearbox, moi, self.motor_sim.gearing
        )
        self.encoder_sim = encoder.sim_state
        self.encoder_sim.sensor_offset = encoder_offset

    def update(self, dt: units.seconds) -> None:
        self.mech_sim.update(self.motor_sim.get_motor_voltage(), dt)
        self.motor_sim.update_from_mechanism(
            self.mech_sim.get_angular_position(),
            self.mech_sim.get_angular_velocity(),
            dt,
        )
        self.encoder_sim.set_raw_position(
            self.mech_sim.get_angular_position() / math.tau
        )
        self.encoder_sim.set_velocity(self.mech_sim.get_angular_velocity() / math.tau)


class ArmSim:
    def __init__(
        self,
        motor_sim: MotorSim,
        moi: units.kilogram_square_meters,
        arm_length: units.meters,
        encoder: phoenix6.hardware.CANcoder,
        encoder_offset: units.turns,
        min_angle: units.radians,
        max_angle: units.radians,
        starting_angle: units.radians,
    ) -> None:
        self.motor_sim = motor_sim
        self.encoder_sim = encoder.sim_state
        self.encoder_sim.sensor_offset = encoder_offset
        self.encoder_sim.set_raw_position(starting_angle / math.tau)
        self.mech_sim = ArmMechanism(
            self.motor_sim.gearbox,
            moi,
            self.motor_sim.gearing,
            arm_length,
            min_angle,
            max_angle,
            starting_angle,
        )

    def update(self, dt: units.seconds) -> None:
        self.mech_sim.update(self.motor_sim.get_motor_voltage(), dt)

        self.motor_sim.update_from_mechanism(
            self.mech_sim.get_angular_position(),
            self.mech_sim.get_angular_velocity(),
            dt,
        )
        self.encoder_sim.set_raw_position(
            self.mech_sim.get_angular_position() / math.tau
        )
        self.encoder_sim.set_velocity(self.mech_sim.get_angular_velocity() / math.tau)


# class ServoEncoderSim:
#     def __init__(self, pwm, encoder):
#         self.pwm_sim = PWMSim(pwm)
#         self.encoder_sim = DutyCycleEncoderSim(encoder)

#     def update(self):
#         command = self.pwm_sim.getPosition()


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.rio = RoboRioSim()
        self.robot = robot
        self.imu = robot.chassis.imu.sim_state
        swerve_constants = TunerConstants()
        module_constants = [
            swerve_constants.front_left,
            swerve_constants.front_right,
            swerve_constants.back_left,
            swerve_constants.back_right,
        ]

        self.kinematics = self.robot.chassis.kinematics
        swerve_positions = [
            Translation2d(module.location_x, module.location_y)
            for module in module_constants
        ]
        self.swerve = SimSwerveDrivetrain(swerve_positions, self.imu, module_constants)

        self.flywheel_sim = SteerModuleSim(
            TalonFXMotorSim(
                DCMotor.krakenX60,
                robot.shooter.flywheel_motor,
                gearing=robot.shooter.FLYWHEEL_GEAR_RATIO,
            ),
            796.0 * 1e-6,
        )

        self.turret_sim = TurretSim(
            TalonFXMotorSim(
                DCMotor.minion,
                robot.turret.motor,
                gearing=(1 / robot.turret.MOTOR_TO_TURRET_GEARING),
            ),
            0.02890532995,
            robot.turret.absolute_encoder,
            robot.turret.ENCODER_OFFSET,
        )

        self.vision_sim = VisionSystemSim("main")
        self.vision_sim.addAprilTags(game.apriltag_layout)
        properties = SimCameraProperties.OV9281_1280_720()
        self.port_camera = PhotonCameraSim(robot.port_vision.camera, properties)
        self.port_camera.setMaxSightRange(5.0)
        self.port_visual_localiser = robot.port_vision
        self.vision_sim.addCamera(
            self.port_camera,
            self.port_visual_localiser.robot_to_camera(wpilib.Timer.getFPGATimestamp()),
        )
        self.vision_sim_counter = 0

        self.port_vision_servo_sim = PWMSim(self.port_visual_localiser.servo)
        self.port_vision_encoder_sim = DutyCycleEncoderSim(
            self.port_visual_localiser.encoder
        )

        self.intake_arm_sim = ArmSim(
            TalonFXMotorSim(
                DCMotor.falcon500,
                robot.intake.deployer_motor_left,
                robot.intake.deployer_motor_right,
                gearing=1 / robot.intake.DEPLOYER_TO_CANCODER_GEARING,
            ),
            robot.intake.ARM_MOI,
            robot.intake.ARM_LENGTH,
            robot.intake.deployer_encoder,
            robot.intake.ENCODER_ZERO_OFFSET,
            robot.intake.DEPLOYED_INTAKE_ANGLE,
            robot.intake.RETRACTED_INTAKE_ANGLE,
            robot.intake.DEPLOYED_INTAKE_ANGLE,
        )

    def update_sim(self, now: float, tm_diff: units.seconds) -> None:
        self.swerve.update(
            tm_diff, self.rio.getVInVoltage(), self.robot.chassis.modules
        )
        states = tuple(
            [module.get_current_state() for module in self.robot.chassis.modules]
        )
        assert len(states) == 4
        speeds = self.kinematics.toChassisSpeeds(states)
        self.flywheel_sim.update(tm_diff)

        self.imu.add_yaw(math.degrees(speeds.omega * tm_diff))

        self.intake_arm_sim.update(tm_diff)
        self.physics_controller.drive(speeds, tm_diff)
        self.turret_sim.update(tm_diff)
        self.port_vision_encoder_sim.set(
            constrain_angle(
                (
                    (
                        self.port_visual_localiser.servo_offsets.full_range
                        - self.port_visual_localiser.servo_offsets.neutral
                    )
                    * (2.0 * self.port_visual_localiser.servo.getPosition() - 1.0)
                    + self.port_visual_localiser.servo_offsets.neutral
                ).radians()
            )
        )

        # Simulate slow vision updates.
        self.vision_sim_counter += 1
        if self.vision_sim_counter == 10:
            self.vision_sim.adjustCamera(
                self.port_camera,
                self.port_visual_localiser.robot_to_camera(
                    wpilib.Timer.getFPGATimestamp()
                ),
            )
            self.vision_sim.update(self.physics_controller.get_pose())
            self.vision_sim_counter = 0
