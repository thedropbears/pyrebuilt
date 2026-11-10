class ClimberComponent:
    def __init__(self):

        #Make methods for extending, retecting, check if it is extended, checking if it's reteacted, homing
        #actucator: motor
        #sensors: CANDi (limit switches), motor encoder

        def extend(self) -> None:
            pass
        
        def retract(self) -> None:
            pass 

        def is_extended(self) -> bool:
            return True #temp - setup to prevent robot from hurting itself

        def is_retracted(self) -> bool:
            return True  #temp - setup to prevent robot from hurting itself

        def try_index(self) -> None:
            pass

        pass

    def execute(self) -> None:
        pass
