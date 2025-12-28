"""Descriptions of the field and match state."""

import dataclasses
import math
import typing

import robotpy_apriltag
import wpilib
from wpimath.geometry import (
    Pose2d,
    Pose3d,
    Rotation2d,
    Translation2d,
)

apriltag_layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
    robotpy_apriltag.AprilTagField.k2025ReefscapeWelded
)

TagId = typing.Literal[
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22
]

get_fiducial_pose = typing.cast(
    typing.Callable[[TagId], Pose3d],
    apriltag_layout.getTagPose,
)

APRILTAGS = apriltag_layout.getTags()


@dataclasses.dataclass(slots=True)
class Tag2d:
    id: TagId
    pose: Pose2d


APRILTAGS_2D = [
    Tag2d(typing.cast(TagId, tag.ID), tag.pose.toPose2d()) for tag in APRILTAGS
]


L3_TAGS = [7, 9, 11, 18, 20, 22]
L2_TAGS = [6, 8, 10, 17, 19, 21]

FIELD_WIDTH = apriltag_layout.getFieldWidth()
FIELD_LENGTH = apriltag_layout.getFieldLength()

RED_REEF_POS = (
    get_fiducial_pose(7).translation().toTranslation2d()
    + get_fiducial_pose(10).translation().toTranslation2d()
) / 2
BLUE_REEF_POS = (
    get_fiducial_pose(18).translation().toTranslation2d()
    + get_fiducial_pose(21).translation().toTranslation2d()
) / 2

ALGAE_MIN_DIAMETER = 16.0  # inches
ALGAE_MAX_DIAMETER = 16.5  # inches


def cage_pos(is_red: bool) -> list[Translation2d]:
    if is_red:
        return [
            Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2 - 1.054),
            Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2 - 2.144),
            Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2 - 3.234),
        ]

    else:
        return [
            Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2 + 1.054),
            Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2 + 2.144),
            Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2 + 3.234),
        ]


# TODO: write functions for rotational symmetry


def field_flip_pose2d(p: Pose2d):
    return Pose2d(
        field_flip_translation2d(p.translation()),
        field_flip_rotation2d(p.rotation()),
    )


def field_flip_rotation2d(r: Rotation2d):
    return Rotation2d(-r.cos(), r.sin())


def field_flip_translation2d(t: Translation2d):
    return Translation2d(FIELD_LENGTH - t.x, t.y)


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def nearest_reef_tag(pose: Pose2d) -> tuple[int, Pose2d]:
    position = pose.translation()
    distance = math.inf
    closest_tag_id = 0
    tag_pose_2d = Pose2d()

    for tag_id in L2_TAGS + L3_TAGS:
        tag_pose = apriltag_layout.getTagPose(tag_id)

        assert tag_pose

        tag_distance = tag_pose.toPose2d().translation().distance(position)
        if tag_distance < distance:
            distance = tag_distance
            closest_tag_id = tag_id
            tag_pose_2d = tag_pose.toPose2d()

    return (closest_tag_id, tag_pose_2d)


def is_L3(tag_id: int) -> bool:
    return tag_id in L3_TAGS
