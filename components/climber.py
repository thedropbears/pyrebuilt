class ClimberComponent:
    def __init__(self):
        pass

        # Look up Phoenix 6 documentation

    def try_index(self) -> None:
        # returns nothing. Mutates something in class
        pass

    def extend(self) -> None:
        # Kraken X60
        pass

    def retract(self) -> None:
        # Kraken X60
        pass

    def is_extended(self) -> bool:
        # read extended limit switch and return true if triggered
        # switch is connected to CANdi S2IN
        return True

    def is_retracted(self) -> bool:
        # read retracted limit switch and return true if triggered
        # switch is connected to CANdi S1IN
        return True

    def execute(self) -> None:
        pass
