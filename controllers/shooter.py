from magicbot import StateMachine, state

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.intake import IntakeComponent


class Shooter(StateMachine):
    ballistics: BallisticsComponent
    intake: IntakeComponent
    chassis: ChassisComponent

    def __init__(self):
        # TODO Implement this
        pass

    @state(first=True)
    def shooting(self) -> None:
        # TODO Implement this
        # deploy intake
        # energise flywheels
        # run ballistics calculation
        pass
