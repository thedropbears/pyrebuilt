import math
from logging import Logger

import ntcore
import wpilib
from magicbot import feedback, tunable
from phoenix6.swerve import requests
from phoenix6.swerve.swerve_module import ChassisSpeeds, SwerveModule
from phoenix6.utils import fpga_to_current_time
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)
from wpimath.units import rotationsToRadians, seconds

from swerves.comp import TunerConstants, TunerSwerveDrivetrain
from utilities.game import is_red
from utilities.position import TeamPoses


class ChassisComponent:
    field: wpilib.Field2d
    logger: Logger
    max_angular_rate = tunable(rotationsToRadians(0.75))

    def __init__(self) -> None:
        self.on_red_alliance = is_red()

        self.tuner_constants = TunerConstants()
        modules = [
            self.tuner_constants.front_left,
            self.tuner_constants.front_right,
            self.tuner_constants.back_left,
            self.tuner_constants.back_right,
        ]

        self.phoenix_swerve = TunerSwerveDrivetrain(
            self.tuner_constants.drivetrain_constants,
            modules,
        )

        self.imu = self.phoenix_swerve.pigeon2

        kinematics = self.phoenix_swerve.kinematics
        assert isinstance(kinematics, SwerveDrive4Kinematics)
        self.kinematics = kinematics

        self.max_speed = self.tuner_constants.speed_at_12_volts  # TODO update this

        self.request: requests.SwerveRequest = requests.Idle()

        nt = ntcore.NetworkTableInstance.getDefault().getTable("/components/chassis")
        self.drive_state_table = nt.getSubTable("module_states")
        self.drive_pose = self.drive_state_table.getStructTopic(
            "Pose", Pose2d
        ).publish()
        self.drive_speeds = self.drive_state_table.getStructTopic(
            "Speeds", ChassisSpeeds
        ).publish()
        self.drive_module_states = self.drive_state_table.getStructArrayTopic(
            "ModuleStates", SwerveModuleState
        ).publish()
        self.drive_module_targets = self.drive_state_table.getStructArrayTopic(
            "ModuleTargets", SwerveModuleState
        ).publish()
        self.drive_module_positions = self.drive_state_table.getStructArrayTopic(
            "ModulePositions", SwerveModulePosition
        ).publish()
        self.drive_timestamp = self.drive_state_table.getDoubleTopic(
            "Timestamp"
        ).publish()
        self.drive_odometry_frequency = self.drive_state_table.getDoubleTopic(
            "OdometryFrequency"
        ).publish()

    def setup(self) -> None:
        self.modules = self.phoenix_swerve.modules

        self.phoenix_swerve.set_state_std_devs((0.05, 0.05, 0.01))
        self.phoenix_swerve.set_vision_measurement_std_devs((0.4, 0.4, 0.03))

        self.field_obj = self.field.getObject("fused_pose")
        self.set_pose(TeamPoses.RED_TEST_POSE if is_red() else TeamPoses.BLUE_TEST_POSE)

    def on_enable(self) -> None:
        self.update_alliance()
        self.update_odometry()

    @feedback
    def get_pose(self) -> Pose2d:
        return self.phoenix_swerve.get_state().pose

    @feedback
    def get_imu_rotation(self) -> Rotation2d:
        return self.imu.getRotation2d()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    @feedback
    def get_velocity(self) -> ChassisSpeeds:
        return self.phoenix_swerve.get_state().speeds

    @feedback
    def swerve_voltages(self) -> tuple[float, float, float, float]:
        return (
            self.modules[0].drive_motor.get_motor_voltage().value,
            self.modules[1].drive_motor.get_motor_voltage().value,
            self.modules[2].drive_motor.get_motor_voltage().value,
            self.modules[3].drive_motor.get_motor_voltage().value,
        )

    @feedback
    def get_drive_currents(self):
        return (
            self.modules[0].drive_motor.get_stator_current().value,
            self.modules[1].drive_motor.get_stator_current().value,
            self.modules[2].drive_motor.get_stator_current().value,
            self.modules[3].drive_motor.get_stator_current().value,
        )

    @feedback
    def is_stationary(self) -> bool:
        velocity = self.get_velocity()
        return (
            math.isclose(velocity.vx, 0.0, abs_tol=0.1)
            and math.isclose(velocity.vy, 0.0, abs_tol=0.1)
            and math.isclose(velocity.omega, 0.0, abs_tol=math.radians(3))
        )

    def reset_yaw(self) -> None:
        """Sets pose to current pose but with a heading of forwards"""
        cur_pose = self.get_pose()
        self.set_pose(
            Pose2d(cur_pose.translation(), Rotation2d(math.pi if is_red() else 0))
        )

    def update_alliance(self) -> None:
        # Check whether our alliance has "changed"
        # If so, it means we have an update from the FMS and need to re-init the odom
        if is_red() != self.on_red_alliance:
            self.on_red_alliance = is_red()
            # TODO update with new game info
            self.set_pose(
                TeamPoses.RED_TEST_POSE
                if self.on_red_alliance
                else TeamPoses.BLUE_TEST_POSE
            )

    def set_pose(self, pose: Pose2d) -> None:
        self.phoenix_swerve.reset_pose(pose)
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def update_odometry(self) -> None:
        drivetrain_state = self.phoenix_swerve.get_state()
        self.field_obj.setPose(drivetrain_state.pose)
        self.telemeterise(drivetrain_state)

    def reset_odometry(self) -> None:
        """Reset odometry to current team's podium"""
        # TODO update with new game info
        if is_red():
            self.set_pose(TeamPoses.RED_PODIUM)
        else:
            self.set_pose(TeamPoses.BLUE_PODIUM)

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        self.set_request_velocities(requests.FieldCentric(), vx, vy, omega)

    def drive_robot(self, vx: float, vy: float, omega: float) -> None:
        self.set_request_velocities(requests.RobotCentric(), vx, vy, omega)

    def stop(self) -> None:
        self.set_request_velocities(requests.RobotCentric(), 0.0, 0.0, 0.0)

    def add_vision_measurement(
        self,
        vision_robot_pose: Pose2d,
        timestamp: seconds,
        vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ):
        self.phoenix_swerve.add_vision_measurement(
            vision_robot_pose,
            fpga_to_current_time(timestamp),
            vision_measurement_std_devs,
        )

    def set_request_velocities(
        self,
        request: requests.FieldCentric | requests.RobotCentric,
        vx: float,
        vy: float,
        omega: float,
    ) -> None:
        request.velocity_x = vx
        request.velocity_y = vy
        request.rotational_rate = omega
        # 10% deadband
        request.deadband = self.max_speed * 0.02
        request.rotational_deadband = self.max_angular_rate * 0.02
        request.drive_request_type = SwerveModule.DriveRequestType.VELOCITY
        self.set_control(request)

    def set_control(self, request: requests.SwerveRequest) -> None:
        self.request = request

    def execute(self) -> None:
        self.phoenix_swerve.set_control(self.request)
        self.request = (
            requests.Idle()
        )  # Safety so that robot stops if not commanded next cycle

        self.update_odometry()

    def telemeterise(self, state: TunerSwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterise it to NetworkTables
        """
        # Telemeterise the swerve drive state
        self.drive_pose.set(state.pose)
        self.drive_speeds.set(state.speeds)
        self.drive_module_states.set(state.module_states)
        self.drive_module_targets.set(state.module_targets)
        self.drive_module_positions.set(state.module_positions)
        self.drive_timestamp.set(state.timestamp)
