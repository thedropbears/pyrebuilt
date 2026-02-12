from magicbot import StateMachine, feedback, state
from wpimath.geometry import Translation2d

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from utilities.game import (
    alliance_hub_pos,
    behind_alliance_hub_pos,
    is_alliance_hub_active,
    is_in_alliance_zone,
    is_red,
)


class Shooter(StateMachine):
    ballistics: BallisticsComponent
    intake: IntakeComponent
    chassis: ChassisComponent

    def __init__(self):
        # TODO Implement this
        pass

    @feedback
    def is_hub_active(self) -> bool:
        return is_alliance_hub_active()

    def get_target_shoot_pos(self) -> Translation2d:
        if is_in_alliance_zone(self.chassis.get_pose().translation()):
            return alliance_hub_pos(is_red())
        else:
            return behind_alliance_hub_pos(is_red())

    @state(first=True)
    def shooting(self) -> None:
        self.intake.intake()
        self.ballistics.energise_flywheels()
        self.ballistics.calculate_for(
            self.get_target_shoot_pos(),
            self.chassis.get_pose(),
            self.chassis.get_velocity().toTwist2d(0.02),
        )
