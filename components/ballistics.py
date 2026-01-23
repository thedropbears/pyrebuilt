from math import atan2

from components.chassis import ChassisComponent
from utilities.game import get_hub_pos, is_red


class BallisticsComponent:
    chassis: ChassisComponent

    def __init__(self):
        self.hub_pos = get_hub_pos(is_red())
        self.chassis_pos = self.chassis.get_pose().translation()
        self.chassis_rot = self.chassis.get_rotation()

    def distance_to_hub(self):  # Distance from robot to hub
        return self.chassis_pos.distance(get_hub_pos(is_red()))

    def angle_to_hub(self):  # Angle from robot to hub
        return atan2(
            self.hub_pos.y - self.chassis_pos.y, self.hub_pos.x - self.chassis_pos.x
        )

    def required_shooter_angle_to_hub(
        self,
    ):  # Angle of shooter (relative to robot) to hub
        return self.angle_to_hub() - self.chassis_rot.radians()

    def execute(self):
        pass
