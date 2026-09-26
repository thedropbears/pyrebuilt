import math
from dataclasses import dataclass
from typing import ClassVar

import wpilib
import wpiutil.log
import wpiutil.wpistruct
from magicbot import feedback, tunable, will_reset_to
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from photonlibpy.targeting import MultiTargetPNPResult, PhotonPipelineResult
from wpimath import units
from wpimath.geometry import (
    Rotation2d,
    Rotation3d,
    Transform2d,
    Transform3d,
    Translation3d,
)
from wpimath.interpolation import TimeInterpolatableRotation2dBuffer

from components.chassis import ChassisComponent
from utilities.caching import HasPerLoopCache, cache_per_loop
from utilities.game import APRILTAGS_2D, apriltag_layout
from utilities.servo import ServoTurret


@wpiutil.wpistruct.make_wpistruct  # pyright: ignore[reportUnknownMemberType]
@dataclass
class VisibleTag:
    WPIStruct: ClassVar

    tag_id: int
    relative_bearing: float
    range: float


class VisualLocalizer(HasPerLoopCache):
    """
    This localizes the robot from AprilTags on the field,
    using information from a single PhotonVision camera.
    """

    # Time since the last target sighting we allow before informing drivers
    TIMEOUT = 1.0  # s

    CAMERA_FOV = math.radians(
        68
    )  # photon vision says 69.8, but we are being conservative

    CAMERA_MAX_RANGE = 4.0  # m

    # More than 90 degrees means the tag faces the turret; 100 keeps us
    # away from viewing it close to edge-on.
    FACING_ANGLE_THRESHOLD: units.degrees = 100

    # currently just any tag on either hub. It will still localise if it sees others but wont try to aim at them.
    TAG_AIM_WHITELIST = [3, 4, 5, 8, 9, 10, 11, 2, 25, 26, 18, 27, 19, 20, 21, 24]

    add_to_estimator = tunable(True)
    only_use_multitag = tunable(True)

    linear_uncertainty_single_tag = tunable(0.30)
    rotation_uncertainty_single_tag = tunable(0.6)

    linear_uncertainty_multi_tag = tunable(0.05)
    rotation_uncertainty_multi_tag = tunable(0.05)

    reproj_error_threshold = tunable(2.0)

    should_override = will_reset_to(False)

    chassis: ChassisComponent

    TURRET_DEADBAND = math.radians(5)
    LINEAR_MEASUREMENT_STD_DEV = 0.05
    ROTATION_MEASUREMENT_STD_DEV = 0.1

    def __init__(
        self,
        # The name of the camera in PhotonVision.
        name: str,
        turret: ServoTurret,
        # Position of the camera relative to the center of the robot
        turret_pos: Translation3d,
        # The turret rotation at its neutral position (ie centred).
        turret_rot: Rotation2d,
        # The camera relative to the turret (ie without servo rotation)
        camera_offset: Translation3d,
        # The camera pitch on the mount, relative to horizontal
        camera_pitch: float,
        field: wpilib.Field2d,
        data_log: wpiutil.log.DataLog,
    ) -> None:
        super().__init__()
        self.camera = PhotonCamera(name)
        self.turret = turret

        self.last_innovation = Transform2d()
        self.last_mahalanobis = 0.0

        self.robot_to_turret = Transform3d(turret_pos, Rotation3d(turret_rot))
        self.robot_to_turret_2d = Transform2d(turret_pos.toTranslation2d(), turret_rot)
        self.turret_to_camera = Transform3d(
            camera_offset, Rotation3d(roll=0.0, pitch=camera_pitch, yaw=0.0)
        )
        self.heading_buffer = TimeInterpolatableRotation2dBuffer(2.0)

        self.estimator = PhotonPoseEstimator(apriltag_layout, Transform3d())
        self.last_timestamp = -1.0
        self.best_log = field.getObject(name + "_best_log")
        self.field_pos_obj = field.getObject(name + "_vision_pose")

        self.turret_pose = field.getObject(name + "_turret_pose")
        self.current_reproj = 0.0
        self.has_multitag = False
        self.has_seen_multitag = False

        self.override_setpoint = 0.5
        self.allowed_tags = [
            tag for tag in APRILTAGS_2D if tag.id in VisualLocalizer.TAG_AIM_WHITELIST
        ]

    @feedback
    @cache_per_loop
    def relative_bearing_to_best_cluster(self) -> float:
        tags = self.get_visible_tags()
        if len(tags) == 0:
            return 0.0
        relative_bearings = sorted(tag.relative_bearing for tag in tags)
        for offset in range(len(relative_bearings) - 1, 0, -1):
            bearing_pairs = zip(relative_bearings, relative_bearings[offset:])
            for pair in bearing_pairs:
                if abs(pair[0] - pair[1]) < self.CAMERA_FOV:
                    return (pair[1] + pair[0]) * 0.5

        return min(tags, key=lambda v: v.range).relative_bearing

    @feedback
    @cache_per_loop
    def get_visible_tags(self) -> list[VisibleTag]:
        tags_in_view: list[VisibleTag] = []

        robot_pose = self.chassis.get_pose()
        turret_pose = robot_pose.transformBy(self.robot_to_turret_2d)
        turret_translation = turret_pose.translation()
        turret_rotation = turret_pose.rotation()

        for tag in self.allowed_tags:
            tag_pose = tag.pose
            turret_to_tag = tag_pose.translation() - turret_translation
            turret_angle_to_tag = turret_to_tag.angle()
            distance = turret_to_tag.norm()
            relative_facing = tag_pose.rotation() - turret_angle_to_tag

            relative_bearing = self.turret.wrap_into_range(
                (turret_angle_to_tag - turret_rotation).radians()
            )

            if (
                relative_bearing is not None
                and abs(relative_facing.degrees()) > self.FACING_ANGLE_THRESHOLD
                and distance < self.CAMERA_MAX_RANGE
            ):
                # Test for relative facing is more than 90 degrees because we don't want to be too
                # close to parallel to the tag
                tags_in_view.append(VisibleTag(tag.id, relative_bearing, distance))

        return tags_in_view

    @feedback
    def get_desired_turret_angle(self) -> float:
        # Read encoder angle and account for offset
        return self.relative_bearing_to_best_cluster()

    @feedback
    def reproj(self) -> float:
        return self.current_reproj

    @feedback
    def using_multitag(self) -> bool:
        return self.has_multitag

    @feedback
    def get_raw_encoder_rotation(self) -> Rotation2d:
        # The encoder has been set up to return values in the interval [0, 2pi]
        return self.turret.raw_encoder_reading_()

    def camera_connected(self) -> bool:
        return self.camera.isConnected()

    @feedback
    def sees_multi_tag_target(self) -> bool:
        return self.has_multitag and self.sees_target()

    @feedback
    def get_last_mahalanobis(self):
        return self.last_mahalanobis

    @feedback
    def get_last_innovation(self) -> Transform2d:
        return self.last_innovation

    @feedback
    def sees_target(self) -> bool:
        return wpilib.Timer.getFPGATimestamp() - self.last_timestamp < self.TIMEOUT

    def robot_to_camera(self, timestamp: float) -> Transform3d:
        turret_rotation = self.turret.rotation_at(timestamp)
        if turret_rotation is None:
            return self.robot_to_turret

        return (
            self.robot_to_turret
            + Transform3d(Translation3d(), Rotation3d(turret_rotation))
            + self.turret_to_camera
        )

    def zero_servo_(self) -> None:
        self.turret.hold_neutral_()

    def full_range_servo_(self) -> None:
        self.turret.hold_full_range_()

    def execute(self) -> None:
        self.aim_turret()
        self.turret.update()

        self.heading_buffer.addSample(
            wpilib.Timer.getFPGATimestamp(), self.chassis.get_rotation()
        )


    def process_camera_results(self) -> None:
        all_results = self.camera.getAllUnreadResults()

        # Prefer the most recent multitag result; otherwise the most recent result.
        last_results: PhotonPipelineResult | None = None
        multitag_result: MultiTargetPNPResult | None = None
        for results in all_results:
            # if results didn't see any targets
            if not results.getTargets():
                continue
            # We trust multitag results more.
            # Don't replace multitag results with single tag results.
            if multitag_result is not None and results.multitagResult is None:
                continue
            last_results = results
            multitag_result = results.multitagResult

        if last_results is None:
            return

        # We have a new frame to judge, so clear the flag and only set it
        # again if a multitag measurement is actually accepted below.
        self.has_multitag = False

        timestamp = last_results.getTimestampSeconds()

        self.estimator.robotToCamera = self.robot_to_camera(timestamp)

        heading = self.heading_buffer.sample(timestamp)
        if heading is not None:
            self.estimator.addHeadingData(timestamp, heading)

        if multitag_result is not None:
            pipeline_result = self.estimator.estimateCoprocMultiTagPose(last_results)
            if pipeline_result is None:
                return
            linear_vision_uncertainty = self.linear_uncertainty_multi_tag
            rotation_vision_uncertainty = self.rotation_uncertainty_multi_tag

            self.current_reproj = multitag_result.estimatedPose.bestReprojErr
            if self.current_reproj > self.reproj_error_threshold:
                return
            self.has_seen_multitag = True
            is_multitag = True
        else:
            if self.only_use_multitag:
                return
            if self.has_seen_multitag:
                pipeline_result = self.estimator.estimatePnpDistanceTrigSolvePose(
                    last_results
                )
            else:
                pipeline_result = self.estimator.estimateLowestAmbiguityPose(
                    last_results
                )
            if pipeline_result is None:
                return
            linear_vision_uncertainty = self.linear_uncertainty_single_tag
            rotation_vision_uncertainty = self.rotation_uncertainty_single_tag
            if pipeline_result.targetsUsed[0].getPoseAmbiguity() > 0.1:
                return
            is_multitag = False

        # Measurement accepted from here on
        self.has_multitag = is_multitag
        self.last_timestamp = timestamp

        pose = pipeline_result.estimatedPose.toPose2d()
        self.last_innovation = pose - self.chassis.get_pose()

        linear_odometry_std_devs, rotation_odometry_std_devs = (
            VisualLocalizer.LINEAR_MEASUREMENT_STD_DEV,
            VisualLocalizer.ROTATION_MEASUREMENT_STD_DEV,
        )
        sxx = linear_vision_uncertainty**2 + linear_odometry_std_devs**2
        syy = linear_vision_uncertainty**2 + linear_odometry_std_devs**2
        stt = rotation_vision_uncertainty**2 + rotation_odometry_std_devs**2

        self.last_mahalanobis = math.sqrt(
            self.last_innovation.X() ** 2 / sxx
            + self.last_innovation.Y() ** 2 / syy
            + self.last_innovation.rotation().radians() ** 2 / stt
        )

        self.chassis.add_vision_measurement(
            pose,
            timestamp,
            (
                linear_vision_uncertainty,
                linear_vision_uncertainty,
                rotation_vision_uncertainty,
            ),
        )
        self.field_pos_obj.setPose(pose)
        self.best_log.setPose(pose)
        self.turret_pose.setPose(
                    self.chassis.get_pose()
                    + Transform2d(
                        current_robot_to_cam.translation().toTranslation2d(),
                        current_robot_to_cam.rotation().toRotation2d(),
                    )
                )

    def aim_turret(self) -> None:
        desired = self.turret.clamp_angle(self.get_desired_turret_angle())
        # Always allow moves to the limits, so the turret can reach its full range.
        near_a_limit = (
            desired - self.turret.min_angle < self.TURRET_DEADBAND
            or self.turret.max_angle - desired < self.TURRET_DEADBAND
        )
        if (
            abs(desired - self.turret.target_angle) > self.TURRET_DEADBAND
            or near_a_limit
        ):
            self.turret.set_target(desired)
