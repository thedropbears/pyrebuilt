import wpilib

from autonomous.auto_base import AutoBase
from ids import RioSerialNumber


class Spin3m(AutoBase):
    MODE_NAME = "Spin forward for 3m"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "spin3m",
            ]
        )


class Spin3mandreturn(AutoBase):
    MODE_NAME = "spin forward 3m then come back"
    if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
        DISABLED = True

    def __init__(self):
        super().__init__(
            [
                "spin3m_andreturn",
            ]
        )


class Move3m(AutoBase):
    MODE_NAME = "move forward 3m"

    def __init__(self):
        super().__init__(
            [
                "move3m",
            ]
        )


class Move3mandreturn(AutoBase):
    MODE_NAME = "move forward 3m and then come back"

    def __init__(self):
        super().__init__(
            [
                "move3m_andreturn",
            ]
        )
