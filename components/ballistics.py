from wpimath.geometry import Pose2d, Translation3d, Twist2d

from components.shooter import ShooterComponent
from components.turret import TurretComponent


class BallisticsComponent:
    shooter: ShooterComponent
    turret: TurretComponent

    # TODO setup sensible reset and tunable vars

    # TODO Define lookup table for use

    def __init__(self) -> None:
        # TODO Implement this
        pass

    def energise_flywheels(self) -> None:
        # TODO Implement this
        # assuming that we dont want to have the flywheel spun up all the time,
        # but the hood and turret should always run
        pass

    def calculate_for(
        self,
        target_position: Translation3d,
        current_pose: Pose2d,
        current_twist: Twist2d,
    ) -> None:
        # TODO implement this
        # like components with hardware attached we dont want to perform the
        # calculation here. Just set the required vars and wait for execute.
        pass

    def execute(self) -> None:
        # TODO Implement this

        # perform turret calculations

        # dispatch turret command

        # perform shooter calculations

        # dispatch shooter commands
        pass
