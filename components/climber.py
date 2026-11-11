class ClimberComponent:
    def __init__(self):
        pass

    def execute(self) -> None:
        if self.is_extended() or self.is_retracted():
            self.desired_voltage = 0

    def extend(self):
        pass

    def retract(self):
        pass

    def is_extended(self) -> bool:
        return True

    def is_retracted(self) -> bool:
        return True

    def try_index(self):
        pass
