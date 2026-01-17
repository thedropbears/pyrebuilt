from autonomous.auto_base import AutoBase


class Spin3m(AutoBase):
    MODE_NAME = "Spin while forward for 3m"

    def __init__(self):
        super().__init__(
            [
                "spin3m",
            ]
        )


class Spin3mandreturn(AutoBase):
    MODE_NAME = "spin forward 3m then come back"

    def __init__(self):
        super().__init__(
            [
                "spin3m_andreturn",
            ]
        )
