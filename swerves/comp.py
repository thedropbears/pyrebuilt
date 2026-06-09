from typing import overload

from phoenix6 import CANBus, configs, hardware, signals, swerve, units

from swerves.tuner_constants import TunerConstants

__all__ = ["TunerConstants", "TunerSwerveDrivetrain", "tuner_constants"]


tuner_constants = TunerConstants(
    # Both sets of gains need to be tuned to your individual robot
    # The steer motor uses any SwerveModule.SteerRequestType control request with the
    # output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    _steer_gains=(
        configs.Slot0Configs()
        .with_k_p(34.876)
        .with_k_i(0)
        .with_k_d(2.5167)
        .with_k_s(0.17715)
        .with_k_v(2.1227)
        .with_k_a(0.10327)
        .with_static_feedforward_sign(
            signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
        )
    ),
    # When using closed-loop control, the drive motor uses the control
    # output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    _drive_gains=(
        configs.Slot0Configs()
        .with_k_p(0.023983)
        .with_k_d(0)
        .with_k_i(0)
        .with_k_s(0.14118)
        .with_k_v(0.11548)
        .with_k_a(0.0099061)
    ),
    # The closed-loop output type to use for the steer motors;
    # This affects the PID/FF gains for the steer motors
    _steer_closed_loop_output=swerve.ClosedLoopOutputType.VOLTAGE,
    # The closed-loop output type to use for the drive motors;
    # This affects the PID/FF gains for the drive motors
    _drive_closed_loop_output=swerve.ClosedLoopOutputType.VOLTAGE,
    # The type of motor used for the drive motor
    _drive_motor_type=swerve.DriveMotorArrangement.TALON_FX_INTEGRATED,
    # The type of motor used for the drive motor
    _steer_motor_type=swerve.SteerMotorArrangement.TALON_FX_INTEGRATED,
    # The remote sensor feedback type to use for the steer motors;
    # When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    _steer_feedback_type=swerve.SteerFeedbackType.REMOTE_CANCODER,
    # The stator current at which the wheels start to slip;
    # This needs to be tuned to your individual robot
    _slip_current=120.0,
    _steer_dampening_threshold=0.05,
    # Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    # Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
    _drive_initial_configs=configs.TalonFXConfiguration(),
    # Swerve azimuth does not require much torque output, so we can set a relatively low
    # stator current limit to help avoid brownouts without impacting performance.
    _steer_initial_configs=configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(60.0)
        .with_stator_current_limit_enable(True)
    ),
    _encoder_initial_configs=configs.CANcoderConfiguration(),
    # Configs for the Pigeon 2; leave this None to skip applying Pigeon 2 configs
    _pigeon_configs=None,
    # CAN bus that the devices are located on;
    # All swerve devices must share the same CAN bus
    canbus=CANBus("", "./logs/example.hoot"),
    # Theoretical free speed (m/s) at 12 V applied output;
    # This needs to be tuned to your individual robot
    speed_at_12_volts=5.23,
    # Every 1 rotation of the azimuth results in _couple_ratio drive motor turns;
    # This may need to be tuned to your individual robot
    _couple_ratio=3.125,
    _drive_gear_ratio=5.902777777777778,
    _steer_gear_ratio=18.75,
    _wheel_radius=0.0508,
    _invert_left_side=False,
    _invert_right_side=True,
    _pigeon_id=0,
    # These are only used for simulation
    _steer_inertia=0.01,
    _drive_inertia=0.01,
    # Simulated voltage necessary to overcome friction
    _steer_friction_voltage=0.2,
    _drive_friction_voltage=0.2,
    # Front Left
    _front_left_drive_motor_id=1,
    _front_left_steer_motor_id=5,
    _front_left_encoder_id=1,
    _front_left_encoder_offset=-0.177978515625,
    _front_left_steer_motor_inverted=True,
    _front_left_encoder_inverted=False,
    _front_left_x_pos=0.2585,
    _front_left_y_pos=0.2585,
    # Front Right
    _front_right_drive_motor_id=4,
    _front_right_steer_motor_id=8,
    _front_right_encoder_id=4,
    _front_right_encoder_offset=-0.2197265625,
    _front_right_steer_motor_inverted=True,
    _front_right_encoder_inverted=False,
    _front_right_x_pos=0.2585,
    _front_right_y_pos=-0.2585,
    # Back Left
    _back_left_drive_motor_id=2,
    _back_left_steer_motor_id=6,
    _back_left_encoder_id=2,
    _back_left_encoder_offset=-0.333251953125,
    _back_left_steer_motor_inverted=True,
    _back_left_encoder_inverted=False,
    _back_left_x_pos=-0.2585,
    _back_left_y_pos=0.2585,
    # Back Right
    _back_right_drive_motor_id=3,
    _back_right_steer_motor_id=7,
    _back_right_encoder_id=3,
    _back_right_encoder_offset=0.49462890625,
    _back_right_steer_motor_inverted=True,
    _back_right_encoder_inverted=False,
    _back_right_x_pos=-0.2585,
    _back_right_y_pos=-0.2585,
)


class TunerSwerveDrivetrain(
    swerve.SwerveDrivetrain[hardware.TalonFX, hardware.TalonFX, hardware.CANcoder]
):
    """Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types."""

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]T, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]T, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0: None,
        arg1: None,
        arg2: None,
        arg3: None,
        /,
    ) -> None: ...

    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        swerve.SwerveDrivetrain.__init__(
            self,
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            drivetrain_constants,
            arg0,
            arg1,
            arg2,
            arg3,
        )
