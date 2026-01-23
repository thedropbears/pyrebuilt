from magicbot import feedback


class TurretComponent:
    setpoint = 0
    rotation_speed = 0

    def __init__(self) -> None:
        pass

    @feedback
    def raw_encoder_val(self):
        pass

    @feedback
    def current_turret_angle(self):
        pass

    def rotate_to(self, angle):
        pass

    def rotate_by(self, angle):
        pass

    def execute(self):
        pass
