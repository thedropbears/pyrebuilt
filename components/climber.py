class ClimberComponent:  # recipe for a thing. Primatives are a number, word etc. a class is a more complext thing.
    # class [motor_encoder, Candi, switch]:
    def __init__(self):  # equivalent to a constructor, make this fucking thing please
        pass

    def execute(self) -> None:
        if self.is_extended() or self.is_retracted():
            self.desired_voltage = 0

    def extend(self):  # move up until you hit the limit switch
        pass

    def retract(self):  # move down until you hit the limit switch
        pass

    def is_extended(self) -> bool:  # read limit switch and return its state
        return True

    def is_retracted(self) -> bool:  # read limit switch and return its state
        return True

    def try_index(self):  # move down until you reach the limit switch, then set as home
        pass


# this is an example of what code would look like


# homework - commit it
# look at magic bot documentation
# look at documention of hardware - pheonix 6
