import rev
import wpilib


def configure_spark_ephemeral(motor: rev.SparkBase, config: rev.SparkBaseConfig):
    motor.configure(
        config,
        rev.SparkBase.ResetMode.kNoResetSafeParameters,
        rev.SparkBase.PersistMode.kNoPersistParameters,
    )


def configure_spark_reset_and_persist(
    motor: rev.SparkBase, config: rev.SparkBaseConfig
):
    motor.configure(
        config,
        rev.SparkBase.ResetMode.kResetSafeParameters,
        rev.SparkBase.PersistMode.kPersistParameters,
    )


def configure_through_bore_encoder(
    enc: wpilib.DutyCycleEncoder, freq: float = 975.6
) -> None:
    """Configure a REV Through Bore Encoder absolute pulse output.

    This avoids the roboRIO duty cycle frequency computation, which will
    be inaccurate at startup.

    Args:
        enc: The DutyCycleEncoder to configure.
        freq: The assumed frequency of the encoder in Hz. Defaults to
            the encoder's nominal frequency per the datasheet.
    """
    enc.setAssumedFrequency(freq)
    enc.setDutyCycleRange(1 / 1025, 1024 / 1025)
