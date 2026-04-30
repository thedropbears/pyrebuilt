from magicbot import StateMachine, default_state, state, will_reset_to

from components.ballistics import BallisticsComponent
from components.chassis import ChassisComponent
from components.hopper import HopperComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from components.targeter import Targeter
from controllers.gobbler import Gobbler


class Conductor(StateMachine):
    intake: IntakeComponent
    ballistics: BallisticsComponent
    gobbler: Gobbler
    chassis: ChassisComponent
    targeter: Targeter
    hopper: HopperComponent
    shooter: ShooterComponent
    keep_shooting = will_reset_to(False)

    def get_solution(self):
        self.solution = self.ballistics.solve_for(
            self.targeter.get_target(),
            self.ballistics.get_robot_velocity(),
            self.chassis.get_pose().translation(),
        )

    def get_target(self):
        self.target = self.targeter.get_target()

    def energise_flywheels(self):
        self.shooter.set_flywheel(self.solution.target_flywheel_speed)

    def feed_shooter(self):
        self.hopper.feed_rate = 6

    def log_shot(self) -> None:
        return self.ballistics.log_shot()

    def shoot(self) -> None:
        if self.ballistics.shooter_is_ready():
            self.engage(self.shooting)
        self.keep_shooting = True

    def caged_shoot(self) -> None:
        if self.ballistics.shooter_is_ready():
            self.engage(self.caged_shooting)
        self.keep_shooting = True

    def activate_full_ballistics(self) -> None:
        self.get_target()
        self.feed_shooter()
        self.energise_flywheels()

    @default_state
    def priming(self) -> None:
        self.get_target()
        self.energise_flywheels()

    @state(first=True, must_finish=True)
    def shooting(self) -> None:
        self.gobbler.gobble()
        self.activate_full_ballistics()

        if not self.keep_shooting:
            self.next_state(self.purging)

    @state(must_finish=True)
    def caged_shooting(self) -> None:
        self.gobbler.cage()
        self.activate_full_ballistics()

        if not self.keep_shooting:
            self.next_state(self.purging)

    @state(must_finish=True)
    def purging(self) -> None:
        self.activate_full_ballistics()

        if self.intake.is_retracted():
            self.done()
