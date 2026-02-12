from magicbot import StateMachine, feedback, state

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from utilities import game


class Shooter(StateMachine):
    ballistics: BallisticsComponent
    intake: IntakeComponent
    chassis: ChassisComponent

    MAX_SHOOT_RANGE = 10  # TODO make this something accurate

    def __init__(self):
        # TODO Implement this
        pass

    @feedback
    def is_hub_active(self) -> bool:
        return game.is_hub_active()

    @feedback
    def is_in_shooting_position(self) -> bool:
        chassis_pos = self.chassis.get_pose().translation()
        shooting_pos = game.RED_HUB_POS if game.is_red() else game.BLUE_HUB_POS
        
        return shooting_pos.distance(chassis_pos) <= self.MAX_SHOOT_RANGE

    @state(first=True)
    def shooting(self) -> None:
        # TODO Implement this
        # deploy intake
        # energise flywheels
        # run ballistics calculation

        pass
