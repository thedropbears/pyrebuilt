"""This is the top-level state machine that controls the robot.

Main behaviours we are looking for:

    The turret should always be tracking
    The hood should aways be tracking
    The optimal target for shots should always be updated
    The optimal target is dependant on the match time
    The optimal target is dependant on where we are on the field
    The robot should default to tracking where we dispatch the current optimal target for shot
    Commanding a transition to firing should:
    Deploy and run the intake
    Run the hopper index and feed
    Energise the flywheels
    Finishing the firing state should return back to tracking only including denergising any active components
    This will replace controllers/shooter.py

states

tracking:
    ballistics.solve_for(target)

shooting:
    intake.intake
    hopper.feed
    ballistics.solve_for(target)
    ballistics.energise_flywheel"""

from magicbot import StateMachine, default_state, feedback, state

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.hopper import HopperComponent
from components.intake import IntakeComponent
from components.targeter import Targeter
from utilities.game import is_alliance_hub_active


class Conductor(StateMachine):
    ballistics: BallisticsComponent
    intake: IntakeComponent
    chassis: ChassisComponent
    targeter: Targeter
    hopper: HopperComponent

    def __init__(self) -> None:
        # TODO Implement this
        pass

    def shoot(self) -> None:
        self.engage()

    def stop_shooting(self) -> None:
        self.done()

    @feedback
    def is_hub_active(self) -> bool:
        return is_alliance_hub_active()

    @default_state
    def tracking(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())

    @state(first=True, must_finish=True)
    def shooting(self) -> None:
        self.hopper.feed()
        self.intake.intake()
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.energise_flywheels()

    def done(self) -> None:
        super().done()
        # TODO Retract intake
