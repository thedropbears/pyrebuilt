from magicbot import StateMachine, feedback, state

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from components.targeter import Targeter
from utilities.game import is_alliance_hub_active


class Conductor(StateMachine):
    ballistics: BallisticsComponent
    intake: IntakeComponent
    chassis: ChassisComponent
    targeter: Targeter

    def __init__(self):
        # TODO Implement this
        pass

    @feedback
    def is_hub_active(self) -> bool:
        return is_alliance_hub_active()

    @state(first=True)
    def shooting(self) -> None:
        self.intake.intake()
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.energise_flywheels()
