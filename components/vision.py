import math
from dataclasses import dataclass
from typing import ClassVar

import rev
import wpilib
import wpilog
import wpiutil
import wpiutil.wpistruct
from magicbot import feedback, tunable, will_reset_to
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from photonlibpy.targeting import MultiTargetPNPResult, PhotonPipelineResult
from wpimath import (
    Rotation2d,
    Rotation3d,
    TimeInterpolatableRotation2dBuffer,
    Transform2d,
    Transform3d,
    Translation3d,
)

from components.chassis import ChassisComponent
from utilities.caching import HasPerLoopCache, cache_per_loop
from utilities.functions import clamp
from utilities.game import APRILTAGS_2D, apriltag_layout
from utilities.rev_encoder import configure_through_bore_encoder

# Parallax Feedback 360° HS Servo (900-00360): continuous rotation.
# 1500 us = stop, 1280/1720 us = full speed CW/CCW.
SERVO_STOP_PULSE_US = 1500
SERVO_FULL_SPEED_RANGE_US = 220

# Hard rotation limit (from encoder zero) to protect the through-bore encoder
# wrap point and avoid tangling the camera cable.
ROTATION_LIMIT_RAD = math.radians(10.0)


@wpiutil.wpistruct.make_wpistruct
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

    # Give bias to the best pose by multiplying this const to the alt dist
    BEST_POSE_BIAS = 1.2

    # Time since the last target sighting we allow before informing drivers
    TIMEOUT = 1.0  # s

    CAMERA_FOV = math.radians(
        68
    )  # photon vision says 69.8, but we are being conservative
    CAMERA_MAX_RANGE = 4.0  # m

    add_to_estimator = tunable(True)
    only_use_multitag = tunable(True)
    should_log = tunable(True)

    last_pose_z = tunable(0.0, writeDefault=False)
    linear_uncertainty_single_tag = tunable(0.30)
    rotation_uncertainty_single_tag = tunable(0.6)

    linear_uncertainty_multi_tag = tunable(0.05)
    rotation_uncertainty_multi_tag = tunable(0.05)

    reproj_error_threshold = tunable(2.0)

    should_override = will_reset_to(False)
    has_multitag = will_reset_to(False)

    # Proportional gain on rotation error (rad → speed in [-1, 1]).
    # Full speed when the error reaches ~5°.
    servo_kp = tunable(1.0 / math.radians(5.0))
    # Deadband below which the servo is commanded to stop.
    servo_deadband = tunable(math.radians(0.5))

    chassis: ChassisComponent

    def __init__(
        self,
        # The name of the camera in PhotonVision.
        name: str,
        # Position of the camera relative to the center of the robot
        turret_pos: Translation3d,
        # The turret rotation at its neutral position (ie centred).
        turret_rot: Rotation2d,
        # The camera relative to the turret (ie without servo rotation)
        camera_offset: Translation3d,
        # The camera pitch on the mount, relative to horizontal
        camera_pitch: float,
        servo: rev.ServoChannel,
        encoder_id: int,
        encoder_offset: Rotation2d,
        field: wpilib.Field2d,
        data_log: wpilog.DataLog,
    ) -> None:
        super().__init__()
        self.camera = PhotonCamera(name)
        self.encoder = wpilib.DutyCycleEncoder(encoder_id, math.tau, 0.0)
        configure_through_bore_encoder(self.encoder)
        # Offset of encoder in radians when facing forwards (the desired zero).
        # To find this value, manually point the camera forwards and record the encoder value.
        self.encoder_offset = encoder_offset

        self.min_rotation = -ROTATION_LIMIT_RAD
        self.max_rotation = ROTATION_LIMIT_RAD

        self.servo = servo
        self.servo.setEnabled(True)
        self.servo.setPowered(True)
        self.pos = turret_pos
        self.robot_to_turret = Transform3d(turret_pos, Rotation3d(turret_rot))
        self.robot_to_turret_2d = Transform2d(turret_pos.toTranslation2d(), turret_rot)
        self.turret_to_camera = Transform3d(
            camera_offset, Rotation3d(roll=0.0, pitch=camera_pitch, yaw=0.0)
        )
        self.turret_rotation_buffer = TimeInterpolatableRotation2dBuffer(2.0)
        self.heading_buffer = TimeInterpolatableRotation2dBuffer(2.0)
        self.estimator = PhotonPoseEstimator(apriltag_layout, Transform3d())
        self.last_timestamp = -1.0
        self.best_log = field.getObject(name + "_best_log")
        self.field_pos_obj = field.getObject(name + "_vision_pose")
        self.pose_log_entry = wpilog.FloatArrayLogEntry(
            data_log, name + "_vision_pose"
        )

        self.current_reproj = 0.0
        self.has_multitag = False
        self.has_seen_multitag = False

        self._has_pairs = False

        # Target rotation (encoder-relative, radians) when overriding in test mode.
        self.override_target = 0.0

    @feedback
    def get_rotation_limits(self) -> list[float]:
        return [self.min_rotation, self.max_rotation]

    @feedback
    def reproj(self) -> float:
        return self.current_reproj

    @feedback
    def using_multitag(self) -> bool:
        return self.has_multitag

    @feedback
    def get_raw_encoder_rotation(self) -> Rotation2d:
        # The encoder has been set up to return values in the interval [0, 2pi]
        return Rotation2d(self.encoder.get())

    @feedback
    def has_pairs(self) -> bool:
        return self._has_pairs

    @feedback
    @cache_per_loop
    def relative_bearing_to_best_cluster(self) -> float:
        tags = self.get_visible_tags()
        if len(tags) == 0:
            return 0.0
        relative_bearings = [tag.relative_bearing for tag in tags]
        relative_bearings.sort()
        for offset in range(len(relative_bearings) - 1, 0, -1):
            bearing_pairs = zip(relative_bearings, relative_bearings[offset:])
            for pair in bearing_pairs:
                if abs(pair[0] - pair[1]) < self.CAMERA_FOV:
                    self._has_pairs = True
                    return (pair[1] + pair[0]) * 0.5
        # If we get here there are no pairs, so choose the closest
        self._has_pairs = False
        tags.sort(key=lambda v: v.range)
        return tags[0].relative_bearing

    @feedback
    @cache_per_loop
    def get_visible_tags(self) -> list[VisibleTag]:
        tags_in_view = []

        robot_pose = self.chassis.get_pose()
        turret_pose = robot_pose.transformBy(self.robot_to_turret_2d)
        turret_translation = turret_pose.translation()
        turret_rotation = turret_pose.rotation()

        for tag in APRILTAGS_2D:
            tag_pose = tag.pose
            turret_to_tag = tag_pose.translation() - turret_translation
            turret_angle_to_tag = turret_to_tag.angle()
            relative_bearing = turret_angle_to_tag - turret_rotation
            distance = turret_to_tag.norm()
            relative_facing = tag_pose.rotation() - turret_angle_to_tag
            relative_bearing_rad = relative_bearing.radians()
            # Make the angle less than the max rotation, then see if we are above the min too
            in_rotation_range = False
            while relative_bearing_rad > self.max_rotation:
                relative_bearing_rad -= math.tau
            if relative_bearing_rad > self.min_rotation:
                # We are good
                in_rotation_range = True
            # Try in the other direction in case we started below the min
            while relative_bearing_rad < self.min_rotation:
                relative_bearing_rad += math.tau
            if relative_bearing_rad < self.max_rotation:
                # We are good
                in_rotation_range = True

            if (
                in_rotation_range
                and abs(relative_facing.degrees()) > 100
                and distance < self.CAMERA_MAX_RANGE
            ):
                # Test for relative facing is more than 90 degrees because we don't want to be too
                # close to parallel to the tag
                tags_in_view.append(VisibleTag(tag.id, relative_bearing_rad, distance))

        return tags_in_view

    @feedback
    def get_desired_turret_rotation(self) -> float:
        return self.relative_bearing_to_best_cluster()

    @property
    def turret_rotation(self) -> Rotation2d:
        return self.get_raw_encoder_rotation() - self.encoder_offset

    def robot_to_camera(self, timestamp: float) -> Transform3d:
        turret_rotation = self.turret_rotation_buffer.sample(timestamp)
        if turret_rotation is None:
            return self.robot_to_turret

        return (
            self.robot_to_turret
            + self.turret_to_camera
            + Transform3d(Translation3d(), Rotation3d(turret_rotation))
        )

    def zero_servo_(self) -> None:
        # ONLY CALL THIS IN TEST MODE!
        # This is used to put the servo in a neutral position to record the encoder value at that point
        self.should_override = True
        self.override_target = 0.0

    def full_range_servo_(self) -> None:
        # ONLY CALL THIS IN TEST MODE!
        # This is used to put the servo to the full range position to record the encoder value at that point
        self.should_override = True
        self.override_target = self.max_rotation

    def execute(self) -> None:
        target = (
            self.override_target
            if self.should_override
            else self.get_desired_turret_rotation()
        )
        target = clamp(target, self.min_rotation, self.max_rotation)

        current = self.turret_rotation.radians()
        error = target - current

        if abs(error) < self.servo_deadband:
            speed = 0.0
        else:
            speed = clamp(error * self.servo_kp, -1.0, 1.0)

        # 
        if (current >= self.max_rotation and speed > 0.0) or (
            current <= self.min_rotation and speed < 0.0
        ):
            speed = 0.0

        pulse_us = SERVO_STOP_PULSE_US + int(speed * SERVO_FULL_SPEED_RANGE_US)
        self.servo.setPulseWidth(pulse_us)

        now = wpilib.Timer.getTimestamp()
        self.turret_rotation_buffer.addSample(now, self.turret_rotation)
        self.heading_buffer.addSample(now, self.chassis.get_rotation())

        if not self.add_to_estimator:
            return

        all_results = self.camera.getAllUnreadResults()
        # Skip processing results other than the most recent.
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
            self.has_multitag = True

            self.current_reproj = multitag_result.estimatedPose.bestReprojErr
            if self.current_reproj > self.reproj_error_threshold:
                return
            self.has_seen_multitag = True
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

        self.last_timestamp = timestamp

        pose = pipeline_result.estimatedPose.toPose2d()
        self.chassis.phoenix_swerve.add_vision_measurement(
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

    @feedback
    def sees_target(self) -> bool:
        return wpilib.Timer.getTimestamp() - self.last_timestamp < self.TIMEOUT

    @feedback
    def sees_multi_tag_target(self) -> bool:
        return self.has_multitag and self.sees_target()
