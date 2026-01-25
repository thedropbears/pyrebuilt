from wpimath import units


class TurretComponent:
    def __init__(self) -> None:
        # TODO Implement this
        # Initialise Motor

        # Initialise Encoder
        pass

    def slew_relative(self, angle: units.radians) -> None:
        # TODO Implement this
        # TODO update setpoint
        pass

    def slew_to_local(self, angle: units.radians) -> None:
        # TODO Implement this
        # update setpoint
        pass

    def execute(self) -> None:
        # TODO Implement this
        # wrap angle

        # run control cycle
        pass
