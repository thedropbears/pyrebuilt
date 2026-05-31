from magicbot import (
    StateMachine,
    default_state,
    feedback,
    state,
    tunable,
    will_reset_to,
)
from wpilib import Field2d
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from components.ballistics import BallisticsSolver
from components.chassis import ChassisComponent
from components.hopper import HopperComponent
from components.intake import IntakeComponent
from components.shooter import ShooterComponent
from components.targeter import Targeter
from components.turret import TurretComponent
from controllers.gobbler import Gobbler


class Conductor(StateMachine):
    ballistics: BallisticsSolver
    gobbler: Gobbler
    hopper: HopperComponent
    intake: IntakeComponent
    shooter: ShooterComponent
    chassis: ChassisComponent
    turret: TurretComponent
    targeter: Targeter
    field: Field2d

    keep_shooting = will_reset_to(False)
    keep_deploying = will_reset_to(False)
    TURRET_OFFSET = Transform2d(Translation2d(0.149, -0.171), Rotation2d())
    MAX_DRIVE_SPEED_FOR_SHOOTING: units.meters_per_second = 2
    shot_succesful = will_reset_to(False)
    backdriving_rps = tunable(50)

    def setup(self) -> None:
        self.turret_pose = self.field.getObject("Turret Pose")
        turret_base_pose, _ = self.get_current_turret_config()

        self.turret_pose.setPose(turret_base_pose)

    def get_current_turret_config(self) -> tuple[Pose2d, Translation2d]:

        chassis_pose = self.chassis.get_pose()
        chassis_rotation = chassis_pose.rotation()
        chassis_speeds = self.chassis.get_velocity()

        turret_base_pose = chassis_pose.transformBy(self.TURRET_OFFSET)

        chassis_velocity = ChassisSpeeds.fromRobotRelativeSpeeds(
            chassis_speeds, chassis_rotation
        )

        turret_offset_field = (
            turret_base_pose.translation() - chassis_pose.translation()
        )

        turret_base_velocity = Translation2d(
            chassis_velocity.vx - chassis_velocity.omega * turret_offset_field.Y(),
            chassis_velocity.vy + chassis_velocity.omega * turret_offset_field.X(),
        )

        return turret_base_pose, turret_base_velocity

    def dispatch_ballistics_setpoints(self, feed_needed: bool = True):

        turret_base_pose, turret_base_velocity = self.get_current_turret_config()

        self.turret_pose.setPose(
            turret_base_pose.rotateAround(
                turret_base_pose.translation(),
                Rotation2d(self.turret.get_current_angle()),
            )
        )

        solution = self.ballistics.solve_for(
            turret_base_pose,
            turret_base_velocity,
            self.targeter.get_target(),
        )

        if feed_needed:
            self.hopper.feed(solution.feed_speed)
        self.turret.slew_to(solution.bearing)
        self.shooter.set_flywheel(solution.flywheel_speed)

    def shoot(self) -> None:
        if self.shooter.flywheel_is_at_speed():
            self.engage(self.shooting, force=True)
        self.keep_shooting = True

    def log_shot(self) -> None:
        self.shot_succesful = True

    def outtake_intake(self) -> None:
        self.engage(self.outtaking_intake, force=True)
        self.keep_deploying = True

    @feedback
    def get_is_shooting(self) -> bool:
        return self.shot_succesful

    def caged_shoot(self) -> None:
        if self.shooter.flywheel_is_at_speed():
            self.engage(self.caged_shooting)
        self.keep_shooting = True

    def deploy_only(self) -> None:
        self.engage(self.deploying_only, force=True)
        self.keep_deploying = True

    def backdrive(self) -> None:
        self.engage(self.backdriving, force=True)

    @default_state
    def priming(self) -> None:
        self.dispatch_ballistics_setpoints(False)

    @state(first=True, must_finish=True)
    def shooting(self) -> None:
        self.gobbler.gobble()
        self.dispatch_ballistics_setpoints()

        if not self.keep_shooting:
            self.next_state(self.purging)

    @state(must_finish=True)
    def outtaking_intake(self) -> None:
        self.gobbler.outtake()
        self.dispatch_ballistics_setpoints()

        if not self.keep_deploying:
            self.next_state(self.purging)

    @state(must_finish=True)
    def caged_shooting(self) -> None:
        self.gobbler.cage()
        self.dispatch_ballistics_setpoints()

        if not self.keep_shooting:
            self.next_state(self.purging)

    @state(must_finish=True)
    def deploying_only(self) -> None:
        self.gobbler.gobble()
        self.dispatch_ballistics_setpoints(False)
        if not self.keep_deploying:
            self.next_state(self.purging)

    @state(must_finish=True)
    def purging(self) -> None:
        self.dispatch_ballistics_setpoints()

        if self.intake.is_retracted():
            self.done()

    @state
    def backdriving(self) -> None:
        self.hopper.backdrive(self.backdriving_rps)
