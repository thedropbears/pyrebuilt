from magicbot import StateMachine, feedback, state

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from utilities import game


class Shooter(StateMachine):
    ballistics: BallisticsComponent
    intake: IntakeComponent
    chassis: ChassisComponent

    def __init__(self):
        # TODO Implement this
        pass

    @feedback
    def is_hub_active(self) -> bool:
        return game.is_hub_active()

    @state(first=True)
    def shooting(self) -> None:
        # TODO Implement this
        # deploy intake
        # energise flywheels
        # run ballistics calculation
        pass
