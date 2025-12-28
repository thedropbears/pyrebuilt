import math
from dataclasses import dataclass
from logging import Logger

import magicbot
import ntcore
import wpilib
from magicbot import feedback, tunable
from phoenix6.configs import (
    ClosedLoopGeneralConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    Slot0Configs,
)
from phoenix6.controls import PositionVoltage, VelocityVoltage, VoltageOut
from phoenix6.hardware import CANcoder, Pigeon2, TalonFX
from phoenix6.signals import InvertedValue, NeutralModeValue
from wpimath.controller import (
    PIDController,
    SimpleMotorFeedforwardMeters,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveModulePosition,
    SwerveModuleState,
)

import utilities
import utilities.scalers
from ids import CancoderId, TalonId
from utilities.ctre import FALCON_FREE_RPS
from utilities.functions import rate_limit_module
from utilities.game import is_red
from utilities.position import TeamPoses


@dataclass
class SwerveConfig:
    drive_ratio: float
    drive_gains: Slot0Configs
    steer_ratio: float
    steer_gains: Slot0Configs
    reverse_drive: bool
    wheel_circumference: float = 4 * 2.54 / 100 * math.pi


class SwerveModule:
    # limit the acceleration of the commanded speeds of the robot to what is actually
    # achiveable without the wheels slipping. This is done to improve odometry
    accel_limit = 15  # m/s^2

    def __init__(
        self,
        config: SwerveConfig,
        position: Translation2d,
        drive_id: int,
        steer_id: int,
        encoder_id: int,
    ):
        """
        x, y: where the module is relative to the center of the robot
        *_id: can ids of steer and drive motors and absolute encoder
        """
        self.translation = position
        self.state = SwerveModuleState(0, Rotation2d(0))
        self.do_smooth = True

        # Create Motor and encoder objects
        self.steer = TalonFX(steer_id)
        self.drive = TalonFX(drive_id)
        self.encoder = CANcoder(encoder_id)

        # Reduce CAN status frame rates before configuring
        self.steer.get_fault_field().set_update_frequency(
            frequency_hz=4, timeout_seconds=0.01
        )
        self.drive.get_fault_field().set_update_frequency(
            frequency_hz=4, timeout_seconds=0.01
        )

        # Configure steer motor
        steer_config = self.steer.configurator

        steer_motor_config = MotorOutputConfigs()
        steer_motor_config.neutral_mode = NeutralModeValue.BRAKE
        # The SDS Mk4i rotation has one pair of gears.
        steer_motor_config.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.steer_motor_out_config = steer_motor_config

        steer_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / config.steer_ratio
        )

        # configuration for motor pid
        steer_pid = config.steer_gains
        steer_closed_loop_config = ClosedLoopGeneralConfigs()
        steer_closed_loop_config.continuous_wrap = True

        steer_config.apply(steer_motor_config)
        steer_config.apply(steer_pid, 0.01)
        steer_config.apply(steer_gear_ratio_config)
        steer_config.apply(steer_closed_loop_config)

        # Configure drive motor
        drive_config = self.drive.configurator

        drive_motor_config = MotorOutputConfigs()
        drive_motor_config.neutral_mode = NeutralModeValue.BRAKE
        drive_motor_config.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if config.reverse_drive
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.drive_motor_out_config = drive_motor_config

        drive_gear_ratio_config = FeedbackConfigs().with_sensor_to_mechanism_ratio(
            1 / (config.wheel_circumference * config.drive_ratio)
        )

        # configuration for motor pid and feedforward
        self.drive_pid = (
            Slot0Configs()
            .with_k_p(config.drive_gains.k_p)
            .with_k_i(config.drive_gains.k_i)
            .with_k_d(config.drive_gains.k_d)
        )
        self.drive_ff = SimpleMotorFeedforwardMeters(
            kS=config.drive_gains.k_s,
            kV=config.drive_gains.k_v,
            kA=config.drive_gains.k_a,
        )

        drive_config.apply(drive_motor_config)
        drive_config.apply(self.drive_pid, 0.01)
        drive_config.apply(drive_gear_ratio_config)

        self.central_angle = Rotation2d(position.x, position.y)
        self.module_locked = False

        self.sync_steer_encoder()
        self.central_angle = position.angle()
        self.drive_request = VelocityVoltage(0)
        self.stop_request = VoltageOut(0)

    def get_angle_absolute(self) -> float:
        """Gets steer angle (rot) from absolute encoder"""
        return self.encoder.get_absolute_position().value

    def get_angle_integrated(self) -> float:
        """Gets steer angle from motor's integrated relative encoder"""
        return self.steer.get_position().value * math.tau

    def get_rotation(self) -> Rotation2d:
        """Get the steer angle as a Rotation2d"""
        return Rotation2d(self.get_angle_integrated())

    def get_speed(self) -> float:
        # velocity is in rot/s, return in m/s
        return self.drive.get_velocity().value

    def get_distance_traveled(self) -> float:
        return self.drive.get_position().value

    def set(self, desired_state: SwerveModuleState):
        if self.module_locked:
            desired_state = SwerveModuleState(0, self.central_angle)

        # smooth wheel velocity vector
        if self.do_smooth:
            self.state = rate_limit_module(self.state, desired_state, self.accel_limit)
        else:
            self.state = desired_state
        current_angle = self.get_rotation()
        self.state.optimize(current_angle)

        if abs(self.state.speed) < 0.01 and not self.module_locked:
            self.stop()
            return

        target_displacement = self.state.angle - current_angle
        target_angle = self.state.angle.radians()
        self.steer_request = PositionVoltage(target_angle / math.tau)
        self.steer.set_control(self.steer_request)

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = self.state.speed * target_displacement.cos()
        speed_volt = self.drive_ff.calculate(target_speed)

        # original position change/100ms, new m/s -> rot/s
        self.drive.set_control(
            self.drive_request.with_velocity(target_speed).with_feed_forward(speed_volt)
        )

    def stop(self):
        self.drive.set_control(self.drive_request.with_velocity(0).with_feed_forward(0))
        self.steer.set_control(self.stop_request)

    def sync_steer_encoder(self) -> None:
        self.steer.set_position(self.get_angle_absolute())

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_distance_traveled(), self.get_rotation())

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())

    def set_neutral_mode(self, neutral_mode: NeutralModeValue) -> None:
        if self.steer_motor_out_config.neutral_mode == neutral_mode:
            return

        self.steer_motor_out_config.neutral_mode = neutral_mode
        self.steer.configurator.apply(self.steer_motor_out_config)

        self.drive_motor_out_config.neutral_mode = neutral_mode
        self.drive.configurator.apply(self.drive_motor_out_config)

    def toggle_neutral_mode(self) -> None:
        if self.steer_motor_out_config.neutral_mode == NeutralModeValue.BRAKE:
            self.set_neutral_mode(NeutralModeValue.COAST)
        else:
            self.set_neutral_mode(NeutralModeValue.BRAKE)


class ChassisComponent:
    # size including bumpers
    LENGTH = 0.600 + 2 * 0.09
    WIDTH = LENGTH

    DRIVE_CURRENT_THRESHOLD = 35

    MIN_ALIGN_Y_AXIS_SPEED = tunable(0.3)
    MAX_ALIGN_Y_AXIS_SPEED = tunable(1.5)

    HEADING_TOLERANCE = math.radians(1)

    swerve_config: SwerveConfig
    drive_motor_rev_to_meters: float

    control_loop_wait_time: float

    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))
    field: wpilib.Field2d
    logger: Logger

    send_modules = magicbot.tunable(False)
    fudge_factor = magicbot.tunable(12)
    do_smooth = magicbot.tunable(True)
    swerve_lock = magicbot.tunable(False)

    # TODO: Read from positions.py once autonomous is finished
    def __init__(
        self, track_width: float, wheel_base: float, swerve_config: SwerveConfig
    ) -> None:
        self.imu = Pigeon2(0)
        self.heading_controller = PIDController(3.0, 0.0, 0.00)
        wpilib.SmartDashboard.putData(
            "Chassis heading_controller", self.heading_controller
        )
        self.heading_controller.enableContinuousInput(-math.pi, math.pi)
        self.snapping_to_heading = False
        self.heading_controller.setTolerance(self.HEADING_TOLERANCE)

        self.on_red_alliance = False

        self.swerve_config = swerve_config
        self.track_width = track_width
        self.wheel_base = wheel_base

        self.drive_motor_rev_to_meters = (
            self.swerve_config.wheel_circumference * self.swerve_config.drive_ratio
        )

        # maxiumum speed for any wheel
        self.max_wheel_speed = FALCON_FREE_RPS * self.drive_motor_rev_to_meters

        # Front Left
        self.module_fl = SwerveModule(
            config=self.swerve_config,
            position=Translation2d(self.wheel_base / 2, self.track_width / 2),
            drive_id=TalonId.DRIVE_FL,
            steer_id=TalonId.STEER_FL,
            encoder_id=CancoderId.SWERVE_FL,
        )
        # Rear Left
        self.module_rl = SwerveModule(
            config=self.swerve_config,
            position=Translation2d(-self.wheel_base / 2, self.track_width / 2),
            drive_id=TalonId.DRIVE_RL,
            steer_id=TalonId.STEER_RL,
            encoder_id=CancoderId.SWERVE_RL,
        )
        # Rear Right
        self.module_rr = SwerveModule(
            config=self.swerve_config,
            position=Translation2d(-self.wheel_base / 2, -self.track_width / 2),
            drive_id=TalonId.DRIVE_RR,
            steer_id=TalonId.STEER_RR,
            encoder_id=CancoderId.SWERVE_RR,
        )
        # Front Right
        self.module_fr = SwerveModule(
            config=self.swerve_config,
            position=Translation2d(self.wheel_base / 2, -self.track_width / 2),
            drive_id=TalonId.DRIVE_FR,
            steer_id=TalonId.STEER_FR,
            encoder_id=CancoderId.SWERVE_FR,
        )

        self.modules = (
            self.module_fl,
            self.module_rl,
            self.module_rr,
            self.module_fr,
        )

        self.kinematics = SwerveDrive4Kinematics(
            self.module_fl.translation,
            self.module_rl.translation,
            self.module_rr.translation,
            self.module_fr.translation,
        )
        self.sync_all()
        self.imu.reset()
        # self.imu.resetDisplacement()

        nt = ntcore.NetworkTableInstance.getDefault().getTable("/components/chassis")
        module_states_table = nt.getSubTable("module_states")
        self.setpoints_publisher = module_states_table.getStructArrayTopic(
            "setpoints", SwerveModuleState
        ).publish()
        self.measurements_publisher = module_states_table.getStructArrayTopic(
            "measured", SwerveModuleState
        ).publish()

        wpilib.SmartDashboard.putData("Heading PID", self.heading_controller)

    def get_velocity(self) -> ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.get_module_states())

    @feedback
    def imu_rotation(self) -> Rotation2d:
        return self.imu.getRotation2d()

    def get_module_states(
        self,
    ) -> tuple[
        SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
    ]:
        return (
            self.module_fl.get(),
            self.module_rl.get(),
            self.module_rr.get(),
            self.module_fr.get(),
        )

    def setup(self) -> None:
        # TODO update with new game info
        initial_pose = TeamPoses.RED_TEST_POSE if is_red() else TeamPoses.BLUE_TEST_POSE

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.imu.getRotation2d(),
            self.get_module_positions(),
            initial_pose,
            stateStdDevs=(0.05, 0.05, 0.01),
            visionMeasurementStdDevs=(0.4, 0.4, 0.03),
        )
        self.field_obj = self.field.getObject("fused_pose")
        self.set_pose(initial_pose)

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        """Field oriented drive commands"""
        current_heading = self.get_rotation()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, current_heading
        )

    def to_field_oriented(self, chassis_speed: ChassisSpeeds) -> ChassisSpeeds:
        current_heading = self.get_rotation()
        return ChassisSpeeds.fromRobotRelativeSpeeds(chassis_speed, current_heading)

    def drive_local(self, vx: float, vy: float, omega: float) -> None:
        """Robot oriented drive commands"""
        self.chassis_speeds = ChassisSpeeds(vx, vy, omega)

    def align_on_y(self, offset: float, distance_tol: float, angle_tol: float) -> None:
        if math.isclose(offset, 0.0, abs_tol=distance_tol):
            self.chassis_speeds.vy = 0

        elif math.isclose(self.heading_controller.getError(), 0.0, abs_tol=angle_tol):
            # move in direction opposite to offset, proportional to offset
            self.chassis_speeds.vy = -math.copysign(
                utilities.scalers.scale_value(
                    abs(offset),
                    0,
                    1.5,
                    self.MIN_ALIGN_Y_AXIS_SPEED,
                    self.MAX_ALIGN_Y_AXIS_SPEED,
                ),
                offset,
            )

    def limit_to_positive_longitudinal_velocity(self) -> None:
        self.chassis_speeds.vy = 0.0
        self.chassis_speeds.omega = 0.0
        self.chassis_speeds.vx = (
            0.0 if self.chassis_speeds.vx <= 0.0 else self.chassis_speeds.vx
        )

    def snap_to_heading(self, heading: float) -> None:
        """set a heading target for the heading controller"""
        self.snapping_to_heading = True
        self.heading_controller.setSetpoint(heading)

    def stop_snapping(self) -> None:
        """stops the heading_controller"""
        self.snapping_to_heading = False

    @feedback
    def is_stationary(self) -> bool:
        velocity = self.get_velocity()
        return (
            math.isclose(velocity.vx, 0.0, abs_tol=0.1)
            and math.isclose(velocity.vy, 0.0, abs_tol=0.1)
            and math.isclose(velocity.omega, 0.0, abs_tol=math.radians(3))
        )

    def stop(self) -> None:
        for module in self.modules:
            module.stop()
            # Also reset the state to account for the internal smoothing
            module.state = SwerveModuleState(0, module.get_rotation())
        self.stop_snapping()

    def execute(self) -> None:
        # rotate desired velocity to compensate for skew caused by discretization
        # see https://www.chiefdelphi.com/t/field-relative-swervedrive-drift-even-with-simulated-perfect-modules/413892/

        if self.snapping_to_heading:
            self.chassis_speeds.omega = self.heading_controller.calculate(
                self.get_rotation().radians()
            )
        else:
            self.heading_controller.reset()

        desired_speed_translation = Translation2d(
            self.chassis_speeds.vx, self.chassis_speeds.vy
        ).rotateBy(
            Rotation2d(
                -self.chassis_speeds.omega
                * self.fudge_factor
                * self.control_loop_wait_time
            )
        )
        desired_speeds = ChassisSpeeds(
            desired_speed_translation.x,
            desired_speed_translation.y,
            self.chassis_speeds.omega,
        )

        desired_speeds = ChassisSpeeds.discretize(
            desired_speeds, self.control_loop_wait_time
        )
        if self.swerve_lock:
            self.do_smooth = False

        desired_states = self.kinematics.toSwerveModuleStates(desired_speeds)
        desired_states = self.kinematics.desaturateWheelSpeeds(
            desired_states, attainableMaxSpeed=self.max_wheel_speed
        )

        for state, module in zip(desired_states, self.modules):
            module.module_locked = self.swerve_lock
            module.do_smooth = self.do_smooth
            module.set(state)

        self.update_odometry()

    def on_enable(self) -> None:
        """update the odometry so the pose estimator doesn't have an empty buffer

        While we should be building the pose buffer while disabled,
        this accounts for the edge case of crashing mid match and immediately enabling with an empty buffer
        """
        self.update_alliance()
        self.update_odometry()
        self.set_coast_in_neutral(False)

    def on_disable(self) -> None:
        self.stop()
        self.set_coast_in_neutral(coast_mode=False)
        self.do_smooth = True
        self.heading_controller.setPID(Kp=3.0, Ki=0.0, Kd=0.0)

    def get_rotational_velocity(self) -> float:
        return math.radians(
            self.imu.get_angular_velocity_z_world().value
        )  # TODO Check direction of positive rotation

    def lock_swerve(self) -> None:
        self.swerve_lock = True

    def unlock_swerve(self) -> None:
        self.swerve_lock = False

    def update_alliance(self) -> None:
        # Check whether our alliance has "changed"
        # If so, it means we have an update from the FMS and need to re-init the odom
        if is_red() != self.on_red_alliance:
            self.on_red_alliance = is_red()
            # TODO update with new game info
            if self.on_red_alliance:
                self.set_pose(TeamPoses.RED_TEST_POSE)
            else:
                self.set_pose(TeamPoses.BLUE_TEST_POSE)

    def update_odometry(self) -> None:
        self.estimator.update(self.imu.getRotation2d(), self.get_module_positions())
        self.field_obj.setPose(self.get_pose())
        if self.send_modules:
            self.setpoints_publisher.set([module.state for module in self.modules])
            self.measurements_publisher.set([module.get() for module in self.modules])

    def sync_all(self) -> None:
        for m in self.modules:
            m.sync_steer_encoder()

    def set_pose(self, pose: Pose2d) -> None:
        self.estimator.resetPosition(
            self.imu.getRotation2d(), self.get_module_positions(), pose
        )
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)

    def reset_yaw(self) -> None:
        """Sets pose to current pose but with a heading of forwards"""
        cur_pose = self.estimator.getEstimatedPosition()
        self.set_pose(
            Pose2d(cur_pose.translation(), Rotation2d(math.pi if is_red() else 0))
        )

    def reset_odometry(self) -> None:
        """Reset odometry to current team's podium"""
        # TODO update with new game info
        if is_red():
            self.set_pose(TeamPoses.RED_PODIUM)
        else:
            self.set_pose(TeamPoses.BLUE_PODIUM)

    def get_module_positions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        return (
            self.module_fl.get_position(),
            self.module_rl.get_position(),
            self.module_rr.get_position(),
            self.module_fr.get_position(),
        )

    def get_pose(self) -> Pose2d:
        """Get the current location of the robot relative to ???"""
        return self.estimator.getEstimatedPosition()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    @feedback
    def at_desired_heading(self) -> bool:
        return abs(self.heading_controller.getError()) <= self.HEADING_TOLERANCE

    def set_coast_in_neutral(self, coast_mode: bool = True) -> None:
        mode = NeutralModeValue.COAST if coast_mode else NeutralModeValue.BRAKE
        self.module_fl.set_neutral_mode(mode)
        self.module_rl.set_neutral_mode(mode)
        self.module_rr.set_neutral_mode(mode)
        self.module_fr.set_neutral_mode(mode)

    def toggle_coast_in_neutral(self) -> None:
        self.module_fl.toggle_neutral_mode()
        self.module_rl.toggle_neutral_mode()
        self.module_rr.toggle_neutral_mode()
        self.module_fr.toggle_neutral_mode()
