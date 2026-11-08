from typing import overload

from phoenix6 import configs, hardware, swerve, units

type ModuleConstants = swerve.SwerveModuleConstants[
    configs.TalonFXConfiguration,
    configs.TalonFXConfiguration,
    configs.CANcoderConfiguration,
]


class TunerSwerveDrivetrain(
    swerve.SwerveDrivetrain[hardware.TalonFX, hardware.TalonFX, hardware.CANcoder]
):
    """Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types."""

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[ModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :param modules:              Constants for each specific module
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[ModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :param modules:                     Constants for each specific module
        """
        ...

    @overload
    def __init__(
        self,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[ModuleConstants],
        /,
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]T, with units in meters
                                            and radians
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]T, with units in meters
                                            and radians
        :param modules:                     Constants for each specific module
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
