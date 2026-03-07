import enum


@enum.unique
class TalonId(enum.IntEnum):
    """CAN ID for CTRE Talon motor controllers (e.g. Talon FX, Talon SRX)."""

    DRIVE_FL = 1
    STEER_FL = 5

    DRIVE_RL = 2
    STEER_RL = 6

    DRIVE_RR = 3
    STEER_RR = 7

    DRIVE_FR = 4
    STEER_FR = 8

    FLYWHEEL = 9

    CLIMBER = 12

    INTAKE = 15
    INDEXER = 14
    INJECTOR = 18

    INTAKE_DEPLOYER_LEFT = 16
    INTAKE_DEPLOYER_RIGHT = 17

    TURRET = 20


@enum.unique
class CancoderId(enum.IntEnum):
    """CAN ID for CTRE CANcoder."""

    SWERVE_FL = 1
    SWERVE_RL = 2
    SWERVE_RR = 3
    SWERVE_FR = 4

    TURRET = 6

    HOOD = 7

    INTAKE = 8


@enum.unique
class SparkId(enum.IntEnum):
    """CAN ID for REV SPARK motor controllers (Spark Max, Spark Flex)."""

    CLIMBER_SENSOR = 4

    TURRET = 2


@enum.unique
class DioChannel(enum.IntEnum):
    """roboRIO Digital I/O channel number."""

    PORT_VISION_ENCODER = 0

    INTAKE_DEPLOYER_ENCODER = 7

    CLIMBER_BREAKBEAM_SENSOR_FRONT = 2
    CLIMBER_BREAKBEAM_SENSOR_BACK = 4
    HOOD_ENCODER = 3


@enum.unique
class PwmChannel(enum.IntEnum):
    """roboRIO PWM output channel number."""

    PORT_VISION_SERVO = 0

    HOOD_SERVO = 1


@enum.unique
class RioSerialNumber(enum.StrEnum):
    """roboRIO serial number"""

    TEST_BOT = "0305cc42"
    COMP_BOT = "03062898"


@enum.unique
class AnalogChannel(enum.IntEnum):
    """roboRIO Analog input channel number"""


@enum.unique
class CandleId(enum.IntEnum):
    """CAN ID for CTRE CANdle devices"""

    LED = 0


@enum.unique
class CandiId(enum.IntEnum):
    """CAN ID for CTRE CANdi devices"""

    CLIMBER_SENSOR = 2
