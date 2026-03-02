from magicbot import StateMachine, state
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from utilities.game import (
    alliance_tower_pos,
    get_movement_to_tower,
    get_tower_heading,
    is_red,
)


class AutoClimber(StateMachine):
    chassis: ChassisComponent
    climber: ClimberComponent

    has_breakbeam_triggered = False

    DRIVE_SPEED = 0.1
    ALLOWABLE_DISTANCE_FROM_TOWER = 0.75
    DRIVE_INTO_TOWER_TIME = 1

    pid_controller = PIDController(2.0, 0.0, 0.0)

    def start(self):
        self.engage()

    def retract(self):
        self.engage(self.retracting, True)

    @state(first=True)
    def aligning(self):
        self.climber.deploy()
        self.chassis.snap_to_heading(
            get_tower_heading(self.chassis.get_pose().translation())
        )

        if self.chassis.at_desired_heading():
            self.next_state(self.moving_to_tower)
            return

    @state
    def moving_to_tower(self):
        pose = self.chassis.get_pose()
        tower_pos = alliance_tower_pos(is_red())
        tower_heading = get_tower_heading(pose.translation())

        speeds = ChassisSpeeds(
            self.pid_controller.calculate(pose.X(), tower_pos.X()),
            self.pid_controller.calculate(pose.Y(), tower_pos.Y()),
            self.chassis.heading_controller.calculate(
                pose.rotation().radians(), tower_heading
            ),
        )

        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)

        if self.climber.at_tower_either_hook():
            self.next_state(self.driving_into_tower)
            return

    @state
    def driving_into_tower(self, initial_call: bool, state_tm):
        if initial_call:
            self.old_time = state_tm

        direction = get_movement_to_tower(self.chassis.get_pose().translation())

        self.chassis.drive_field(
            direction[0] * self.DRIVE_SPEED, direction[1] * self.DRIVE_SPEED, 0
        )

        if state_tm >= self.old_time + self.DRIVE_INTO_TOWER_TIME:
            self.next_state(self.climbing)
            return

    @state
    def climbing(self):
        self.climber.retract()
        self.chassis.stop_snapping()

    @state(must_finish=True)
    def retracting(self):
        self.climber.deploy()
        if self.climber.at_extension_limit():
            self.next_state(self.safing)
            return

    @state(must_finish=True)
    def safing(self):
        if (
            self.chassis.get_pose().translation().distance(alliance_tower_pos(is_red()))
            > self.ALLOWABLE_DISTANCE_FROM_TOWER
        ):
            self.done()

    def done(self):
        super().done()
        self.climber.retract()
