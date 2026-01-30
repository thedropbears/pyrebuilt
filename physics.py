from __future__ import annotations

import math
import statistics
import typing
from collections.abc import Callable
from typing import override

import phoenix6
import rev
import wpilib
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import (
    DCMotorSim,
    DutyCycleEncoderSim,
    PWMSim,
    SingleJointedArmSim,
)
from wpimath import units
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.system.plant import DCMotor, LinearSystemId

from components.chassis import SwerveModule
from components.intake import IntakeComponent
from utilities import game
from utilities.functions import constrain_angle

if typing.TYPE_CHECKING:
    from robot import MyRobot


class MotorSim(typing.Protocol):
    def update(self, dt: units.seconds) -> None: ...
    def get_angular_position(self) -> units.radians: ...


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


class TalonFXMotorSim(MotorSim):
    def __init__(
        self,
        # DCMotor gearbox factory, e.g. DCMotor.falcon500
        gearbox_motor: Callable[[int], DCMotor],
        *motors: phoenix6.hardware.TalonFX | phoenix6.hardware.TalonFXS,
        # Reduction between motor and encoder readings, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
        moi: units.kilogram_square_meters,
    ):
        gearbox = gearbox_motor(len(motors))
        self.plant = LinearSystemId.DCMotorSystem(gearbox, moi, gearing)
        self.gearing = gearing
        self.sim_states = [motor.sim_state for motor in motors]
        for sim_state in self.sim_states:
            sim_state.set_supply_voltage(12.0)
        self.motor_sim = DCMotorSim(self.plant, gearbox)

    @override
    def update(self, dt: units.seconds) -> None:
        voltage = self.sim_states[0].motor_voltage
        self.motor_sim.setInputVoltage(voltage)
        self.motor_sim.update(dt)
        motor_rev_per_mechanism_rad = self.gearing / math.tau
        for sim_state in self.sim_states:
            sim_state.set_raw_rotor_position(
                self.motor_sim.getAngularPosition() * motor_rev_per_mechanism_rad
            )
            sim_state.set_rotor_velocity(
                self.motor_sim.getAngularVelocity() * motor_rev_per_mechanism_rad
            )

    @override
    def get_angular_position(self) -> units.radians:
        return self.motor_sim.getAngularPosition()


class TurretSim:
    def __init__(
        self,
        motor_sim: MotorSim,
        encoder: wpilib.DutyCycleEncoder,
        encoder_offset: float,
    ):
        self.motor_sim = motor_sim
        self.encoder_sim = DutyCycleEncoderSim(encoder)
        self.encoder_offset = encoder_offset

    def update(self, dt: units.seconds):
        self.motor_sim.update(dt)
        self.encoder_sim.set(
            self.motor_sim.get_angular_position() + self.encoder_offset
        )


class SparkMotorSim(MotorSim):
    def __init__(
        self,
        gearbox_motor: Callable[[int], DCMotor],
        motor: rev.SparkMax,
        # Reduction between motor and mechanism rotations, as output over input.
        # If the mechanism spins slower than the motor, this number should be greater than one.
        gearing: float,
        moi: units.kilogram_square_meters,
    ):
        gearbox = gearbox_motor(1)
        self.plant = LinearSystemId.DCMotorSystem(gearbox, moi, gearing)
        self.mech_sim = DCMotorSim(self.plant, gearbox)
        self.motor_sim = rev.SparkSim(motor, gearbox)

    @override
    def update(self, dt: units.seconds):
        vbus = self.motor_sim.getBusVoltage()
        self.mech_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * vbus)
        self.mech_sim.update(dt)
        self.motor_sim.iterate(self.mech_sim.getAngularVelocity(), vbus, dt)

    @override
    def get_angular_position(self) -> units.radians:
        return self.mech_sim.getAngularPosition()


class SparkArmSim:
    def __init__(self, mech_sim: SingleJointedArmSim, motor_sim: rev.SparkSim) -> None:
        self.mech_sim = mech_sim
        self.motor_sim = motor_sim
        self.motor_encoder_sim = self.motor_sim.getRelativeEncoderSim()

    def update(self, dt: units.seconds) -> None:
        vbus = self.motor_sim.getBusVoltage()
        self.mech_sim.setInputVoltage(self.motor_sim.getAppliedOutput() * vbus)
        self.mech_sim.update(dt)
        self.motor_sim.iterate(self.mech_sim.getVelocity(), vbus, dt)
        self.motor_encoder_sim.iterate(self.mech_sim.getVelocity(), dt)


# class ServoEncoderSim:
#     def __init__(self, pwm, encoder):
#         self.pwm_sim = PWMSim(pwm)
#         self.encoder_sim = DutyCycleEncoderSim(encoder)

#     def update(self):
#         command = self.pwm_sim.getPosition()


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        self.physics_controller = physics_controller

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: tuple[
            SwerveModule, SwerveModule, SwerveModule, SwerveModule
        ] = robot.chassis.modules

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive,
                units_per_rev=1 / robot.chassis.drive_motor_rev_to_meters,
                kV=2.7,
            )
            for module in robot.chassis.modules
        ]
        self.steer = [
            TalonFXMotorSim(
                DCMotor.krakenX60,
                module.steer,
                gearing=1 / robot.chassis.swerve_config.steer_ratio,
                # measured from MKCad CAD
                moi=0.0009972,
            )
            for module in robot.chassis.modules
        ]

        self.turret_sim = TurretSim(
            SparkMotorSim(
                DCMotor.NEO,
                robot.turret.motor,
                robot.turret.MOTOR_TO_TURRET_GEARING,
                moi=0.02890532995,
            ),
            robot.turret.absolute_encoder,
            robot.turret.ENCODER_OFFSET,
        )

        self.imu = robot.chassis.imu.sim_state

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

        # Intake arm simulation
        intake_arm_gearbox = DCMotor.NEO(1)
        self.intake_arm_motor = rev.SparkMaxSim(
            robot.intake.arm_motor, intake_arm_gearbox
        )
        self.intake_arm_encoder_sim = DutyCycleEncoderSim(robot.intake.encoder)
        self.intake_arm = SparkArmSim(
            SingleJointedArmSim(
                intake_arm_gearbox,
                IntakeComponent.gear_ratio,
                moi=IntakeComponent.ARM_MOI,
                armLength=IntakeComponent.ARM_LENGTH,
                minAngle=IntakeComponent.DEPLOYED_ANGLE_LOWER,
                maxAngle=IntakeComponent.RETRACTED_ANGLE,
                simulateGravity=True,
                startingAngle=IntakeComponent.RETRACTED_ANGLE,
            ),
            self.intake_arm_motor,
        )

        self.intake = robot.intake

    def update_sim(self, now: float, tm_diff: units.seconds) -> None:
        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        speeds = self.kinematics.toChassisSpeeds(
            (
                self.swerve_modules[0].get(),
                self.swerve_modules[1].get(),
                self.swerve_modules[2].get(),
                self.swerve_modules[3].get(),
            )
        )

        self.imu.add_yaw(math.degrees(speeds.omega * tm_diff))

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

        # Update intake arm simulation
        self.intake_arm.update(tm_diff)
        self.intake_arm_encoder_sim.set(
            self.intake_arm.mech_sim.getAngle() + IntakeComponent.ARM_ENCODER_OFFSET
        )
