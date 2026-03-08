from magicbot import StateMachine, state

from components.intake import IntakeComponent


class Gobbler(StateMachine):
    intake: IntakeComponent

    def gobble(self) -> None:
        self.engage(self.intaking)

    def cage(self) -> None:
        self.engage(self.caging)

    @state(first=True, must_finish=True)
    def intaking(self) -> None:
        self.intake.intake()
        self.next_state(self.retracting)

    @state(must_finish=True)
    def retracting(self) -> None:
        self.intake.backdrive()
        if not self.intake.is_retracting():
            self.done()

    @state
    def caging(self) -> None:
        self.intake.drive()
