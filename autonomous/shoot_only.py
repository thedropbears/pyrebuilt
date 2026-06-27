from magicbot import AutonomousStateMachine, timed_state

from controllers.conductor import Conductor


class ShootOnly(AutonomousStateMachine):
    MODE_NAME = "Shoot only"

    conductor: Conductor

    @timed_state(duration=1, first=True, next_state="caged_shooting")
    def prepping(self) -> None:
        self.conductor.deploy_only()

    @timed_state(duration=10)
    def caged_shooting(self) -> None:
        self.conductor.shoot()
