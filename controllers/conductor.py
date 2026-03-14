from magicbot import StateMachine, default_state, state, will_reset_to

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.intake import IntakeComponent
from components.leds import LEDComponent
from components.targeter import Targeter
from controllers.gobbler import Gobbler


class Conductor(StateMachine):
    intake: IntakeComponent
    ballistics: BallisticsComponent
    gobbler: Gobbler
    chassis: ChassisComponent
    targeter: Targeter
    leds: LEDComponent

    keep_shooting = will_reset_to(False)

    def log_shot(self) -> None:
        return self.ballistics.log_shot()

    def shoot(self) -> None:
        if self.ballistics.shooter_is_ready():
            self.engage(self.shooting)
        self.keep_shooting = True

    def activate_full_ballistics(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.feed_shooter()
        self.ballistics.energise_flywheels()

    @default_state
    def priming(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.energise_flywheels()
        self.leds.conductor_state_machine_tracking()

    @state(must_finish=True)
    def shooting(self) -> None:
        self.gobbler.gobble()
        self.activate_full_ballistics()
        self.leds.conducter_state_machine_active()

        if not self.keep_shooting:
            self.next_state(self.purging)

    @state(must_finish=True)
    def purging(self) -> None:
        self.activate_full_ballistics()

        if self.intake.is_retracted():
            self.done()
