from magicbot import StateMachine, state

from components.intake import IntakeComponent


class Gobbler(StateMachine):
    intake: IntakeComponent

    def gobble(self) -> None:
        self.engage("intaking")

    def cage(self) -> None:
        self.engage("caging")

    @state(first=True, must_finish=True)
    def intaking(self) -> None:
        self.next_state("retracting")
        self.intake.intake()

    @state(must_finish=True)
    def retracting(self) -> None:
        self.intake.backdrive()
        if not self.intake.is_retracting():
            self.done()

    @state
    def caging(self) -> None:
        self.intake.drive()
