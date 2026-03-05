from magicbot import StateMachine, default_state, state, tunable
from wpimath.geometry import Rotation2d

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

    flywheel_shoot_speed = tunable(20)
    hood_angle = tunable(30)

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
        # self.ballistics.solve_for(self.targeter.get_target())
        self.ballistics.force_solution(
            desired_flywheel_speed=self.flywheel_shoot_speed,
            desired_turret_bearing=Rotation2d(0),
            desired_hood_angle=self.hood_angle,
        )
        self.ballistics.energise_flywheels()

    def done(self) -> None:
        super().done()
