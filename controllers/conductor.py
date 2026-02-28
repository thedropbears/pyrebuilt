from magicbot import StateMachine, default_state, state

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.hopper import HopperComponent
from components.intake import IntakeComponent
from components.targeter import Targeter


class Conductor(StateMachine):
    ballistics: BallisticsComponent
    intake: IntakeComponent
    chassis: ChassisComponent
    targeter: Targeter
    hopper: HopperComponent

    def shoot(self) -> None:
        self.engage()

    def stop_shooting(self) -> None:
        self.done()

    @default_state
    def tracking(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())

    @state(first=True, must_finish=True)
    def shooting(self) -> None:
        self.hopper.feed()
        self.intake.intake()
        #self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.force_solution( 
            desired_flywheel_speed=20,
            desired_turret_angle=0,
            desired_hood_angle=0,
        )
        self.ballistics.energise_flywheels()

    def done(self) -> None:
        super().done()
