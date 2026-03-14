from magicbot import AutonomousStateMachine, state, timed_state

from controllers.conductor import Conductor


class AutoStationary(AutonomousStateMachine):
    conductor: Conductor

    def __init__(self) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()

    def on_enable(self) -> None:
        super().on_enable()

    def on_disable(self) -> None:
        super().on_disable()

    @state(first=True)
    def initialising(self) -> None:
        self.next_state("shooting")

    # Shoots for specified duration
    @timed_state(duration=10.0, next_state="finish")
    def shooting(self) -> None:
        self.conductor.caged_shoot()

    @state
    def finish(self) -> None:
        super().done()
