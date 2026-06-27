from magicbot import AutonomousStateMachine, timed_state

from autonomous.auto_purge_base import AutoPurgeBase
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


class ShootAndRetreatTower(AutoPurgeBase):
    MODE_NAME = "Shoot and retreat from tower bump"

    conductor: Conductor

    def __init__(self):
        super().__init__(["comp/retreat_from_tower_bump"])


class ShootAndRetreatDepot(AutoPurgeBase):
    MODE_NAME = "Shoot and retreat from depot bump"

    conductor: Conductor

    def __init__(self):
        super().__init__(["comp/retreat_from_depot_bump"])
