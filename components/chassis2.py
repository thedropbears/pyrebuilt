import math

import wpilib
from magicbot import feedback, tunable, will_reset_to
from pathplannerlib.config import ModuleConfig, RobotConfig
from phoenix6.swerve import requests
from phoenix6.swerve.swerve_module import ChassisSpeeds, SwerveModule
from utilities.positions import TeamPoses
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.system.plant import DCMotor
from wpimath.units import rotationsToRadians

from ids import RioSerialNumber
from utilities import game


class Drivetrain:
    field: wpilib.Field2d
    _should_track_hub = will_reset_to(False)
    max_speed = tunable(0.0)
    max_angular_rate = tunable(rotationsToRadians(0.75))

    def __init__(self) -> None:
        if wpilib.RobotController.getSerialNumber() == RioSerialNumber.STUMPY_BOT:
            from generated.stumpy import TunerConstants, TunerSwerveDrivetrain
        elif wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
            from generated.test import (  # type: ignore
                TunerConstants,
                TunerSwerveDrivetrain,
            )
        else:
            from generated.comp import (  # type: ignore
                TunerConstants,
                TunerSwerveDrivetrain,
            )
        self._heading_controller = PIDController(Kp=10, Ki=0, Kd=0)
        self._heading_controller.enableContinuousInput(-math.pi, math.pi)
        self._heading_controller.setTolerance(math.radians(3.0))
        self._aligned = False

        tuner_constants = TunerConstants()
        modules = [
            tuner_constants.front_left,
            tuner_constants.front_right,
            tuner_constants.back_left,
            tuner_constants.back_right,
        ]
        self._phoenix_swerve = TunerSwerveDrivetrain(
            tuner_constants.drivetrain_constants,
            modules,
        )
        self._phoenix_swerve.set_state_std_devs((0.05, 0.05, math.radians(0.5)))
        self.tuner_constants = tuner_constants

        self._field_drive_request = requests.FieldCentric()
        self._robot_drive_request = requests.RobotCentric()

        # Use open-loop control for drive motors
        self._field_drive_request.drive_request_type = (
            SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self._robot_drive_request.drive_request_type = (
            SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )

        self._request: requests.SwerveRequest = requests.Idle()

        self.on_blue_alliance = game.is_blue()

        robot_mass = 60.0  # kg
        robot_moment_of_inertia = (
            robot_mass * (0.7**2) / 6.0
        )  # approximated -> square plate with 0.7m sides
        self.pp_robot_config = RobotConfig(
            massKG=robot_mass,
            MOI=robot_moment_of_inertia,
            moduleConfig=ModuleConfig(
                wheelRadiusMeters=self.tuner_constants._wheel_radius,
                maxDriveVelocityMPS=self.tuner_constants.speed_at_12_volts,
                wheelCOF=1.0,
                driveMotor=DCMotor.krakenX60FOC().withReduction(
                    self.tuner_constants._drive_gear_ratio
                ),
                driveCurrentLimit=self.tuner_constants._slip_current,
                numMotors=1,
            ),
            moduleOffsets=[
                Translation2d(m.location_x, m.location_y)
                for m in [
                    self.tuner_constants.front_left,
                    self.tuner_constants.front_right,
                    self.tuner_constants.back_left,
                    self.tuner_constants.back_right,
                ]
            ],
        )

    def setup(self) -> None:
        self.max_speed = self.tuner_constants.speed_at_12_volts
        # speed_at_12_volts desired top speed
        self.field_obj = self.field.getObject("odometry")
        # pass through required methods from phoenix swerve object
        self.kinematics = self._phoenix_swerve.kinematics
        self.modules = self._phoenix_swerve.modules
        self.pigeon2 = self._phoenix_swerve.pigeon2
        self.get_state = self._phoenix_swerve.get_state
        self.add_vision_measurement = self._phoenix_swerve.add_vision_measurement
        self.set_state_std_devs = self._phoenix_swerve.set_state_std_devs
        self.set_vision_measurement_std_devs = (
            self._phoenix_swerve.set_vision_measurement_std_devs
        )

        self.set_pose(
            TeamPoses.BLUE_TEST_POSE if game.is_blue() else TeamPoses.RED_TEST_POSE
        )

    def on_enable(self) -> None:
        self._phoenix_swerve.set_operator_perspective_forward(
            Rotation2d.fromDegrees(0) if game.is_blue() else Rotation2d.fromDegrees(180)
        )
        self.update_alliance()

    def roborio_serial(self) -> str:
        return wpilib.RobotController.getSerialNumber()

    @feedback
    def pose(self) -> Pose2d:
        return self._phoenix_swerve.get_state().pose

    @feedback
    def velocity_robot(self) -> ChassisSpeeds:
        return self._phoenix_swerve.get_state().speeds

    def velocity_field(self) -> ChassisSpeeds:
        pose = self.pose()
        robot_vel = self.velocity_robot()
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            robot_vel.vx, robot_vel.vy, robot_vel.omega, pose.rotation()
        )

    def update_alliance(self) -> None:
        # Check whether our alliance has "changed"
        # If so, it means we have an update from the FMS and need to re-init the odom
        if game.is_blue() != self.on_blue_alliance:
            self.on_blue_alliance = game.is_blue()
            # TODO update with new game info
            if self.on_blue_alliance:
                self.set_pose(TeamPoses.BLUE_TEST_POSE)
            else:
                self.set_pose(TeamPoses.RED_TEST_POSE)

    def set_pose(self, pose: Pose2d) -> None:
        self._phoenix_swerve.reset_pose(pose)
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def update_odometry(self) -> None:
        self.field_obj.setPose(self._phoenix_swerve.get_state().pose)

    def drive_field(self, vx: float, vy: float, vz: float) -> None:
        self._set_request_velocities(self._field_drive_request, vx, vy, vz)

    def drive_robot(self, vx: float, vy: float, vz: float) -> None:
        self._set_request_velocities(self._robot_drive_request, vx, vy, vz)

    def stop(self) -> None:
        self._set_request_velocities(self._robot_drive_request, 0.0, 0.0, 0.0)

    def _set_request_velocities(
        self,
        request: requests.FieldCentric | requests.RobotCentric,
        vx: float,
        vy: float,
        vz: float,
    ) -> None:
        request.velocity_x = vx
        request.velocity_y = vy
        request.rotational_rate = vz
        # 10% deadband
        request.deadband = self.max_speed * 0.02
        request.rotational_deadband = self.max_angular_rate * 0.02

        self.set_control(request)

    def set_control(self, request: requests.SwerveRequest) -> None:
        self._request = request

    def track_heading(self, heading: float) -> None:
        self._heading_controller.setSetpoint(heading)
        self._should_track_hub = True

    def is_aligned(self) -> bool:
        return self._aligned

    def execute(self) -> None:
        if self._should_track_hub:
            if not isinstance(self._request, requests.FieldCentric) and not isinstance(
                self._request, requests.RobotCentric
            ):
                self._request = requests.RobotCentric()
            current_heading = self.get_state().pose.rotation().radians()
            vz = self._heading_controller.calculate(current_heading)
            self._aligned = self._heading_controller.atSetpoint()
            self._request.rotational_rate = vz
        else:
            self._aligned = False

        self._phoenix_swerve.set_control(self._request)
        self._request = (
            requests.Idle()
        )  # Safety so that robot stops if not commanded next cycle

        self.update_odometry()
