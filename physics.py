from __future__ import annotations

import typing

import wpilib
from phoenix6.swerve.sim_swerve_drivetrain import SimSwerveDrivetrain
from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import (
    DutyCycleEncoderSim,
    PWMSim,
    RoboRioSim,
)
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.system.plant import DCMotor

from swerves.comp import TunerConstants
from utilities import game
from utilities.ctre import CANcoderSim, TalonFXMotorSim
from utilities.functions import constrain_angle
from utilities.simulation import ArmMechanism, MotorMechanismSim, SimpleMechanism

if typing.TYPE_CHECKING:
    from robot import MyRobot

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

        flywheel_motor = TalonFXMotorSim(
            DCMotor.krakenX60, robot.shooter.flywheel_motor
        )
        self.flywheel_sim = MotorMechanismSim(
            flywheel_motor,
            SimpleMechanism(flywheel_motor, 796.0 * 1e-6),
        )

        turret_motor = TalonFXMotorSim(DCMotor.minion, robot.turret.motor)
        self.turret_sim = MotorMechanismSim(
            turret_motor,
            SimpleMechanism(turret_motor, 0.02890532995),
            CANcoderSim.from_dependant_device(
                robot.turret.absolute_encoder,
                robot.turret.motor,
            ),
        )

        intake_motor = TalonFXMotorSim(DCMotor.falcon500, robot.intake.deployer_motor)
        self.intake_arm_sim = MotorMechanismSim(
            intake_motor,
            ArmMechanism(
                intake_motor,
                robot.intake.ARM_MOI,
                robot.intake.ARM_LENGTH,
                min_angle=robot.intake.DEPLOYED_INTAKE_ANGLE,
                max_angle=robot.intake.RETRACTED_INTAKE_ANGLE,
                starting_angle=robot.intake.DEPLOYED_INTAKE_ANGLE,
            ),
            CANcoderSim.from_dependant_device(
                robot.intake.deployer_encoder, robot.intake.deployer_motor
            ),
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

    def update_sim(self, _now: float, tm_diff: units.seconds) -> None:
        self.swerve.update(
            tm_diff, self.rio.getVInVoltage(), self.robot.chassis.modules
        )
        states = tuple(
            [module.get_current_state() for module in self.robot.chassis.modules]
        )
        assert len(states) == 4
        speeds = self.kinematics.toChassisSpeeds(states)
        self.flywheel_sim.update(tm_diff)

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
