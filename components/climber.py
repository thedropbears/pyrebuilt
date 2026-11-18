import math

from phoenix6.hardware import TalonFX, CANdi
from phoenix6.controls import VoltageOut

from phoenix6.configs import (
CANdiConfiguration,
DigitalInputsConfigs,
FeedbackConfigs,
MotorOutputConfigs,
HardwareLimitSwitchConfigs,
SoftwareLimitSwitchConfigs,
TalonFXConfiguration,
)
from magicbot import feedback
from phoenix6.signals import (
    S1CloseStateValue,
    S2CloseStateValue,
    NeutralModeValue,
    InvertedValue,
    ForwardLimitSourceValue,
    ReverseLimitSourceValue,
)

class ClimberComponent:
    DEVICE_ID_MOTOR = 12 #capitalized to show as constant,
    CANDI_ID = 2

    RETRACT_LIMIT = 0.12
    EXTEND_LIMIT = 0.22
    GEAR_RATIO = (1.0/1.0) * (1.0/9.0) * (1.0/4.0)
    SHAFT_RADIUS = 0.00733 # meters radius of hexshaft
    FUDGE_FACTOR = 0.773

    MOTION_VOLTAGE = 6.0

    
    def __init__(self):
        
        self.switch_candi = CANdi(self.CANDI_ID)
        self.switch_candi.configurator.apply(
             CANdiConfiguration().with_digital_inputs(
            DigitalInputsConfigs()
            .with_s1_close_state(S1CloseStateValue.CLOSE_WHEN_FLOATING)
            .with_s2_close_state(S2CloseStateValue.CLOSE_WHEN_FLOATING)
        )
        )

        self.motor = TalonFX (self.DEVICE_ID_MOTOR)
        self.motor.configurator.apply(
            TalonFXConfiguration()
            .with_motor_output(
             MotorOutputConfigs()
             .with_neutral_mode(NeutralModeValue.BRAKE)
             .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
            )
            .with_feedback(
                 FeedbackConfigs().with_sensor_to_mechanism_ratio(
                      1.0 / (ClimberComponent.GEAR_RATIO 
                             * math.tau 
                             * ClimberComponent.SHAFT_RADIUS 
                      )
                             * ClimberComponent.FUDGE_FACTOR        
                ))

            .with_hardware_limit_switch(
                HardwareLimitSwitchConfigs()
                .with_forward_limit_source(ForwardLimitSourceValue.REMOTE_CANDI_S2)
                .with_forward_limit_remote_sensor_id(ClimberComponent.CANDI_ID)
                .with_reverse_limit_source(ReverseLimitSourceValue.REMOTE_CANDI_S1)
                .with_reverse_limit_remote_sensor_id(ClimberComponent.CANDI_ID)
            )
            .with_software_limit_switch(
                SoftwareLimitSwitchConfigs()
                .with_forward_soft_limit_enable(True)
                .with_forward_soft_limit_threshold(ClimberComponent.EXTEND_LIMIT)
                .with_reverse_soft_limit_enable(True)
                .with_reverse_soft_limit_threshold(ClimberComponent.RETRACT_LIMIT)
            )
        )

        self.target_output = 0.0
        self.has_indexed = False
        pass
# def __init__ (): creates a variable
# for climber we must: make motr, get motor encoder, make cancle, get switches.
# Motor defines: foreward direction, gear reduction, brake mode, speed from code
# Switch defines: set as normally closed (decide vif normaly open or closed), 
# relative concoders say rototion while absolute encoders say exactly its position.
    def is_retracted(self) -> bool: # read if climber is retracted
        return self.switch_candi.get_s1_closed().value
    def is_extended(self) -> bool: # read if climber is extended
        return self.switch_candi.get_s2_closed().value

    def try_index(self):
        if self.is_extended():
            self.motor.set_position(ClimberComponent.EXTEND_LIMIT)
            self.has_indexed = True

        if self.is_retracted():
            self.motor.set_position(ClimberComponent.RETRACT_LIMIT)
            self.has_indexed = True

    def extend(self): #move down until you hit the limit switch
        self.target_output = ClimberComponent.MOTION_VOLTAGE

    def retract(self): #move up until you hit the limit switch
        self.target_output = -ClimberComponent.MOTION_VOLTAGE



    def execute(self) -> None:
        self.try_index()

        if not self.has_indexed:
            self.retract()

        if self.is_extended():
            self.target_output = min(self.target_output, 0.0)
        if self.is_retracted():
            self.target_output = max(self.target_output, 0.0)

        #add make do nothing here 
        
        self.motor.set_control(
            VoltageOut(self.target_output, ignore_software_limits=not self.has_indexed)
        )

        self.target_output = 0.0