from magicbot import StateMachine, state, will_reset_to

from components.intake import IntakeComponent


class Gobbler(StateMachine):
    intake: IntakeComponent

    should_retract = will_reset_to(True)

    def gobble(self) -> None:
        self.should_retract = False
        self.engage(self.intaking, force=True)

    def cage(self) -> None:
        self.engage(self.caging, force=True)

    @state(first=True, must_finish=True)
    def intaking(self) -> None:
        self.intake.intake()

        if self.should_retract:
            self.next_state(self.retracting)

    @state(must_finish=True)
    def retracting(self) -> None:
        self.intake.drive()
        if self.intake.is_retracted():
            self.done()

    @state
    def caging(self) -> None:
        self.intake.drive()
