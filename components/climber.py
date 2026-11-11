from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANdi
from phoenix6.configs import CANdiConfiguration, DigitalInputsConfigs 
from phoenix6.signals import S1CloseStateValue
from phoenix6.signals import S2CloseStateValue
from phoenix6.configs import MotorOutputConfigs
from phoenix6.configs import FeedbackConfigs
from phoenix6.configs import HardwareLimitSwitchConfigs
from phoenix6.configs import SoftwareLimitSwitchConfigs


from phoenix6.signals import NeutralModeValue



Device_ID_Motor = 12 #Device ID 12 is the climber motor
Device_ID_LimitSwitch = 13 #Device ID 13 is the limit switch

class ClimberComponent:


    def __init__(self):

        #Make methods for extending, retecting, check if it is extended, checking if it's reteacted, homing
        #actucator: motor
        #sensors: CANDi (limit switches), motor encoder

        self.motor = TalonFX(Device_ID_Motor)
        self.limit_switch = CANdi(Device_ID_LimitSwitch)
        self.candi_config = CANdiConfiguration().with_digital_inputs(DigitalInputsConfigs().with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING).with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING))
        self.motor_config = MotorOutputConfigs().with_inverted(CLOCKWISE_POSITIVE).with_neutral_mode(NeutralModeValue.BRAKE)


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

        if self.is_extended():
            #don't extend
            pass

        if self.is_retracted():
            #don't retract
            pass



        #set motor command

        #zero motor command

        pass
