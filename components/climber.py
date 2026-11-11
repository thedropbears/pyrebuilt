from phoenix6.configs import (
    CANdiConfiguration,
    DigitalInputsConfigs,
)
from phoenix6.hardware import CANdi, TalonFX
from phoenix6.signals import (
    S1CloseStateValue,
    S2CloseStateValue,
)


class ClimberComponent:  # recipe for a thing. Primatives are a number, word etc. a class is a more complext thing.
    # class [motor_encoder, Candi, switch:
    DEVICE_ID_MOTOR = 12  # capitalised with underscores is a global constant
    DEVICE_ID_CANDI = 2
    Gear_Ratio = (1.0 / 1.0) * (1.0 / 9.0) * (1.0 / 4.0)  # A 1 to 1, a 1 to 9, a 1 to 4

    def __init__(self):  # equivalent to a constructor, make this fucking thing please
        self.motor = TalonFX(self.DEVICE_ID_MOTOR)
        self.candi = CANdi(self.DEVICE_ID_CANDI)
        self.candi_config = CANdiConfiguration().with_digital_inputs(
            DigitalInputsConfigs()
            .with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING)
            .with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING)
        )

        pass
        # make motor
        # foward direction
        # gear reduction
        # brake mode - BRAKE
        # experimental
        # get from code
        # get encoder #relative encoder
        # make candi
        # get switches
        # normally open
        # normally closed

    def extend(self):  # move up until you hit the limit switch
        pass

    def retract(self):  # move down until you hit the limit switch
        pass

    def is_extended(self) -> bool:  # read limit switch and return its state
        return True

    def is_retracted(self) -> bool:  # read limit switch and return its state
        return True

    def try_index(
        self,
    ) -> None:  # move down until you reach the limit switch, then set as home (relative encoder to an absolute encoder)
        pass

    def execute(self) -> None:
        if self.is_extended() or self.is_retracted():
            self.desired_voltage = 0
        # send motor command
        # zero next command


# this is an example of what code would look like
# homework - commit it
# look at magic bot documentation
# look at documention of hardware - pheonix 6
# git status, add, commit - m "what i am committing", push
