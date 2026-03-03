"""Descriptions of the field and match state."""

import dataclasses
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
    robotpy_apriltag.AprilTagField.k2026RebuiltWelded
)

TagId = typing.Literal[
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31,
    32,
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

FIELD_WIDTH = apriltag_layout.getFieldWidth()
FIELD_LENGTH = apriltag_layout.getFieldLength()

INTERSECT_OF_TRANSITION_ALLIANCE_ZONE_FROM_ALLIANCE_WALL = 3.9
INTERSECT_OF_NEUTRAL_TRANSITION_ZONE_FROM_ALLIANCE_WALL = 6

SHOOT_POINT_OFFSET = 0.75


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


BLUE_HUB_POS = (
    get_fiducial_pose(20).translation().toTranslation2d()
    + get_fiducial_pose(26).translation().toTranslation2d()
) / 2

BLUE_SHOOT_ANCHOR_1 = Translation2d(
    get_fiducial_pose(18).x,
    get_fiducial_pose(18).y - SHOOT_POINT_OFFSET,
)
BLUE_SHOOT_ANCHOR_2 = Translation2d(
    get_fiducial_pose(21).x,
    get_fiducial_pose(21).y + SHOOT_POINT_OFFSET,
)

BLUE_TRENCH_SHOOT_POS_1 = Translation2d(
    get_fiducial_pose(17).x + 0.5, get_fiducial_pose(17).y
)
BLUE_TRENCH_SHOOT_POS_2 = Translation2d(
    get_fiducial_pose(22).x + 0.5, get_fiducial_pose(22).y
)

BLUE_SHOOT_LINE = get_fiducial_pose(26).x - SHOOT_POINT_OFFSET

BLUE_TOWER_CLIMB_POLE = Translation2d(
    get_fiducial_pose(31).x + 1.055624, get_fiducial_pose(31).y
)

RED_HUB_POS = field_flip_translation2d(BLUE_HUB_POS)

RED_SHOOT_ANCHOR_1 = field_flip_translation2d(BLUE_SHOOT_ANCHOR_1)
RED_SHOOT_ANCHOR_2 = field_flip_translation2d(BLUE_SHOOT_ANCHOR_2)

RED_TRENCH_SHOOT_POS_1 = field_flip_translation2d(BLUE_TRENCH_SHOOT_POS_1)
RED_TRENCH_SHOOT_POS_2 = field_flip_translation2d(BLUE_TRENCH_SHOOT_POS_2)

RED_SHOOT_LINE = FIELD_LENGTH - BLUE_SHOOT_LINE

RED_TOWER_CLIMB_POLE = field_flip_translation2d(BLUE_TOWER_CLIMB_POLE)

FIELD_CENTRE_POS = Translation2d(FIELD_WIDTH / 2, FIELD_LENGTH / 2)

ALLOWABLE_DISTANCE_FROM_TOWER = 1


def is_in_alliance_zone(robot_pos: Translation2d) -> bool:
    if is_red():
        return robot_pos.x > (
            FIELD_LENGTH - INTERSECT_OF_TRANSITION_ALLIANCE_ZONE_FROM_ALLIANCE_WALL
        )
    else:
        return robot_pos.x < INTERSECT_OF_TRANSITION_ALLIANCE_ZONE_FROM_ALLIANCE_WALL


def is_in_transition_zone(robot_pos: Translation2d) -> bool:
    if is_red():
        return (
            (FIELD_LENGTH - INTERSECT_OF_NEUTRAL_TRANSITION_ZONE_FROM_ALLIANCE_WALL)
            < robot_pos.x
            < (FIELD_LENGTH - INTERSECT_OF_TRANSITION_ALLIANCE_ZONE_FROM_ALLIANCE_WALL)
        )
    else:
        return (
            INTERSECT_OF_NEUTRAL_TRANSITION_ZONE_FROM_ALLIANCE_WALL
            < robot_pos.x
            < INTERSECT_OF_TRANSITION_ALLIANCE_ZONE_FROM_ALLIANCE_WALL
        )


def is_in_neutral_zone(robot_pos: Translation2d) -> bool:
    return (
        INTERSECT_OF_NEUTRAL_TRANSITION_ZONE_FROM_ALLIANCE_WALL
        < robot_pos.x
        < (FIELD_LENGTH - INTERSECT_OF_NEUTRAL_TRANSITION_ZONE_FROM_ALLIANCE_WALL)
    )


def is_close_to_tower(robot_pos: Translation2d) -> bool:
    return (
        robot_pos.distance(BLUE_TOWER_CLIMB_POLE) <= ALLOWABLE_DISTANCE_FROM_TOWER
        or robot_pos.distance(RED_TOWER_CLIMB_POLE) <= ALLOWABLE_DISTANCE_FROM_TOWER
    )


def get_movement_to_tower(robot_pos: Translation2d) -> Rotation2d:
    y_direction = -1 if is_in_upper_field_half(robot_pos) else 1
    x_direction = -1 if is_red() else 1

    return Rotation2d(
        -x_direction, -y_direction
    )  # Negative because coordinate system on field is inversed i.e. top right is (0, 0)


def alliance_tower_pos(is_red: bool) -> Translation2d:
    if is_red:
        return get_fiducial_pose(15).translation().toTranslation2d()
    else:
        return get_fiducial_pose(31).translation().toTranslation2d()


def alliance_hub_pos(is_red: bool) -> Translation2d:
    if is_red:
        return RED_HUB_POS
    else:
        return BLUE_HUB_POS


def behind_alliance_hub_pos(is_red: bool) -> Translation2d:
    if is_red:
        return Translation2d(RED_HUB_POS.x + 1.5, RED_HUB_POS.y)
    else:
        return Translation2d(BLUE_HUB_POS.x - 1.5, BLUE_HUB_POS.y)


def alliance_shoot_anchor_pos(is_red: bool) -> tuple:
    if is_red:
        return (RED_SHOOT_ANCHOR_1, RED_SHOOT_ANCHOR_2)
    else:
        return (BLUE_SHOOT_ANCHOR_1, BLUE_SHOOT_ANCHOR_2)


def alliance_trench_shoot_pos(is_red: bool) -> tuple:
    if is_red:
        return (RED_TRENCH_SHOOT_POS_1, RED_TRENCH_SHOOT_POS_2)
    else:
        return (BLUE_TRENCH_SHOOT_POS_1, BLUE_TRENCH_SHOOT_POS_2)


def alliance_shoot_line(is_red: bool) -> float:
    if is_red:
        return RED_SHOOT_LINE
    else:
        return BLUE_SHOOT_LINE


def is_alliance_hub_active() -> bool:
    alliance = wpilib.DriverStation.getAlliance()

    # If we have no alliance, we cannot be enabled, therefore no hub.
    if alliance is None:
        return False

    # Hub is always enabled in autonomous.
    if wpilib.DriverStation.isAutonomousEnabled():
        return True

    # If we're not teleop enabled, there is no hub.
    if not wpilib.DriverStation.isTeleopEnabled():
        return False

    # We're teleop enabled, compute.
    match_time = wpilib.DriverStation.getMatchTime()
    game_data = wpilib.DriverStation.getGameSpecificMessage()

    match game_data:
        case "R":
            red_inactive_first = True
        case "B":
            red_inactive_first = False
        case _:
            # No or invalid game data, assume hub is active.
            return True

    # Shift 1 is active for blue if red won auto, or red if blue won auto.
    shift1_active = (
        not red_inactive_first
        if alliance == wpilib.DriverStation.Alliance.kRed
        else red_inactive_first
    )

    if match_time > 130:
        return True  # Transition shift, hub is active
    elif match_time > 105:
        return shift1_active
    elif match_time > 80:
        return not shift1_active
    elif match_time > 55:
        return shift1_active
    elif match_time > 30:
        return not shift1_active
    else:
        return True  # End game, hub always active


# This will default to the blue alliance if a proper link to the driver station has not yet been established
def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def is_in_upper_field_half(robot_pos: Translation2d) -> bool:
    return robot_pos.y < FIELD_CENTRE_POS.y
