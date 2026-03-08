import wpilib

from autonomous.auto_base import AutoBase
from ids import RioSerialNumber


class Spin3mandreturn(AutoBase):
    MODE_NAME = "spin forward 3m then come back"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "spin3m_andreturn",
            ],
            [
                "trav",
            ],
        )


class dp_pickup_sm_side_climb(AutoBase):
    MODE_NAME = "Start at depot corner, shoot preloaded, shoot depot, depot side climb"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "dp_cage_leg_1",
                "dp_shoot_leg_2",
                "dp_trav_leg_3_sm",
            ],
            [
                "cage",
                "shoot",
                "trav",
            ],
        )


class dp_pickup_opp_side_climb(AutoBase):
    MODE_NAME = (
        "Start at depot corner, shoot preloaded, shoot depot, non-depot side climb"
    )
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "dp_cage_leg_1",
                "dp_shoot_leg_2",
                "dp_trav_leg_3_opp",
            ],
            [
                "cage",
                "shoot",
                "trav",
            ],
        )


class mid_dp_side_climb(AutoBase):
    MODE_NAME = "start at mid, shoot preloaded, depot side climb"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "mid_cage_leg_1",
                "mid_trav_leg_2_sm",
            ],
            [
                "cage",
                "trav",
            ],
        )


class mid_ndp_side_climb(AutoBase):
    MODE_NAME = "start at mid, shoot preloaded, non-depot side climb"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "mid_cage_leg_1",
                "mid_trav_leg_2",
                "mid_trav_leg_3_opp",
            ],
            [
                "cage",
                "trav",
                "trav",
            ],
        )


class mid_dp_pickup_same_side_climb(AutoBase):
    MODE_NAME = "start at mid, shoot preloaded, shoot depot, depot side climb"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "mid_cage_leg_1",
                "mid_trav_leg_2",
                "mid_shoot_leg_3",
                "mid_trav_leg_4_sm",
            ],
            [
                "cage",
                "trav",
                "shoot",
                "trav",
            ],
        )


class mid_dp_pickup_opp_side_climb(AutoBase):
    MODE_NAME = "start at mid, shoot preloaded, shoot depot, then non-depot side climb"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "mid_cage_leg_1",
                "mid_trav_leg_2",
                "mid_shoot_leg_3",
                "mid_trav_leg_3_opp",
            ],
            [
                "cage",
                "trav",
                "shoot",
                "trav",
            ],
        )
