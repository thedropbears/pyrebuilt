import math

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from utilities.game import field_flip_pose2d


# TODO update with new game info
class TeamPoses:
    BLUE_TEST_POSE = Pose2d(2.5, 5.5, 0)
    RED_TEST_POSE = field_flip_pose2d(BLUE_TEST_POSE)
    BLUE_PODIUM = Pose2d(Translation2d(2.992, 4.08455), Rotation2d(math.pi))
    RED_PODIUM = field_flip_pose2d(BLUE_PODIUM)
