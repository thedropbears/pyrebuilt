from magicbot import StateMachine, default_state, state

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.hopper import HopperComponent
from components.leds import LEDComponent
from components.targeter import Targeter
from controllers.gobbler import Gobbler


class Conductor(StateMachine):
    ballistics: BallisticsComponent
    gobbler: Gobbler
    chassis: ChassisComponent
    targeter: Targeter
    hopper: HopperComponent
    leds: LEDComponent

    def log_shot(self) -> None:
        return self.ballistics.log_shot()

    def shoot(self) -> None:
        self.engage()

    def stop_shooting(self) -> None:
        self.done()

    @default_state
    def tracking(self) -> None:
        self.ballistics.solve_for(self.targeter.get_target())
        self.leds.conductor_state_machine_tracking()

    @state(first=True)
    def shooting(self) -> None:
        self.hopper.feed()
        self.gobbler.gobble()
        self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.energise_flywheels()
        self.leds.conducter_state_machine_active()

    def done(self) -> None:
        super().done()
