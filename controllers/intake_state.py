from magicbot import StateMachine, state

from components.intake import IntakeComponent


class Intake(StateMachine):
    intake_component: IntakeComponent
    is_intaking: bool = False

    def intake(self):
        self.is_intaking = True
        self.engage("intaking")

    @state(first=True, must_finish=True)
    def intaking(self):
        if not self.is_intaking:
            self.next_state("retracting")
        self.intake_component.intake()
        self.is_intaking = False

    @state(must_finish=True)
    def retracting(self):
        self.intake_component.backdrive()
        if not self.intake_component.is_retracting():
            self.done()
