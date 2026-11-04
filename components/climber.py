class ClimberComponent:
    def __init__(self):
        self.motor = None
        self.encoder = None
        self.desired_voltage = 0

    def is_extended(self) -> bool:
        # read limit switches
        return True  # change when code written

    def is_retracted(self) -> bool:
        # read limit switches
        return True  # change when code written

    def extend(self) -> None:
        pass

    def retract(self) -> None:
        pass

    def try_index(self) -> None:
        pass

    def execute(self) -> None:
        if self.is_extended() or self.is_retracted():
            self.desired_voltage = 0

        else:
            pass

        self.motor.set(self.desired_voltage)
        pass
