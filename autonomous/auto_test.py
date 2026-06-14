import wpilib

from autonomous.auto_base import AutoBase
from ids import RioSerialNumber


class Drive3m(AutoBase):
    MODE_NAME = "Drive forward for 3m"

    def __init__(self):
        super().__init__(
            [
                "testbot/drive3m"
                if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT
                else "compbot/drive3m"
            ]
        )


class Spin3m(AutoBase):
    MODE_NAME = "Spin while forward for 3m"

    def __init__(self):
        super().__init__(
            [
                "testbot/spin3m"
                if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT
                else "compbot/spin3m"
            ]
        )


class Spin3mandreturn(AutoBase):
    MODE_NAME = "Spin forward 3m then come back"

    def __init__(self):
        super().__init__(
            [
                "testbot/spin3m_andreturn"
                if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT
                else "compbot/spin3m_andreturn"
            ]
        )
