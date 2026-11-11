from phoenix6.hardware import TalonFX, CANdi
from phoenix6.configs import CANdiConfiguration,DigitalInputsConfigs,FeedbackConfigs,MotorOutputConfigs, HardwareLimitSwitchConfigs
from phoenix6.signals import S1CloseStateValue,S2CloseStateValue,NeutralModeValue,InvertedValue

class ClimberComponent:

    DEVICE_ID_MOTOR = 12 #capitalized to show as constant,
    CANDI_ID = 2
    Gear_Ratio = (1.0/1.0) * (1.0/9.0) * (1.0/4.0)
    def __init__(self):
        self.motor = TalonFX (self.DEVICE_ID_MOTOR)
        self.candi = CANdi(self.CANDI_ID)
        self.candi_config = CANdiConfiguration().with_digital_inputs(
            DigitalInputsConfigs().with_s1_close_state(
                S1CloseStateValue.CLOSE_WHEN_FLOATING).
                with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING))
        
        pass
# def __init__ (): creates a variable
# for climber we must: make motr, get motor encoder, make cancle, get switches.
# Motor defines: foreward direction, gear reduction, brake mode, speed from code
# Switch defines: set as normally closed (decide vif normaly open or closed), 
# relative concoders say rototion while absolute encoders say exactly its position.
def is_retracted(self) -> bool: # read if climber is retracted
    return True

def is_extended(self) -> bool: # read if climber is extended
    return True

def try_index(self):
    pass

def extend(self): #move down until you hit the limit switch
    pass

def retract(self): #move up untu you hit the limit switch
    pass



def execute(self) -> None:
    if self.is_extended() or self.is_retracted():
        #add make do nothing here 
        return
    