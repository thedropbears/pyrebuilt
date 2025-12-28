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

    TOP_FLYWHEEL = 10
    BOTTOM_FLYWHEEL = 9

    INTAKE = 15


@enum.unique
class CancoderId(enum.IntEnum):
    """CAN ID for CTRE CANcoder."""

    SWERVE_FL = 1
    SWERVE_RL = 2
    SWERVE_RR = 3
    SWERVE_FR = 4


@enum.unique
class SparkId(enum.IntEnum):
    """CAN ID for REV SPARK motor controllers (Spark Max, Spark Flex)."""

    CLIMBER = 2

    CORAL_PLACER = 20

    INJECTOR_1 = 10
    INJECTOR_2 = 11

    INTAKE_ARM = 4

    WRIST = 5


@enum.unique
class DioChannel(enum.IntEnum):
    """roboRIO Digital I/O channel number."""

    STARBOARD_VISION_ENCODER = 1

    SWERVE_COAST_SWITCH = 2

    INTAKE_ENCODER = 3

    CLIMBER_ENCODER = 4

    ALGAE_INTAKE_SWITCH = 5

    LEFT_CLIMBER_SWITCH = 6

    RIGHT_CLIMBER_SWITCH = 7

    WRIST_ENCODER = 8

    PORT_VISION_ENCODER = 9


@enum.unique
class PwmChannel(enum.IntEnum):
    """roboRIO PWM output channel number."""

    STARBOARD_VISION_SERVO = 0
    PORT_VISION_SERVO = 8

    LIGHT_STRIP = 2


@enum.unique
class RioSerialNumber(enum.StrEnum):
    """roboRIO serial number"""

    TEST_BOT = "0305cc42"
    COMP_BOT = "03062898"


@enum.unique
class AnalogChannel(enum.IntEnum):
    """roboRIO Analog input channel number"""
