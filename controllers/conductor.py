from magicbot import StateMachine, default_state, state

from components.ballistics import BallisticsComponent
from components.intake import IntakeComponent
from components.leds import LEDComponent
from components.shooter import ShooterComponent
from components.targeter import Targeter
from controllers.gobbler import Gobbler


class Conductor(StateMachine):
    intake: IntakeComponent
    ballistics: BallisticsComponent
    gobbler: Gobbler
    shooter: ShooterComponent
    targeter: Targeter
    leds: LEDComponent

    purged = True

    def is_purged(self) -> bool:
        return self.purged

    def log_shot(self) -> None:
        return self.ballistics.log_shot()

    def shoot(self) -> None:
        self.engage()

    def purge(self) -> None:
        self.engage(self.purging)

    @default_state
    def tracking(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())
        self.leds.conductor_state_machine_tracking()

    @state(first=True)
    def energising(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.energise_flywheels()
        if self.shooter.is_at_speed():
            self.next_state(self.shooting)
            return

    @state(must_finish=True)
    def shooting(self) -> None:
        self.purged = False
        self.gobbler.gobble()
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.energise_flywheels()
        self.ballistics.energise_hopper()
        self.leds.conducter_state_machine_active()

    @state(must_finish=True)
    def purging(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.energise_flywheels()
        self.ballistics.energise_hopper()
        if self.intake.is_retracted():
            self.purged = True
            self.done()

    def done(self) -> None:
        super().done()
