from magicbot import StateMachine, state, timed_state, tunable
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from utilities.game import (
    alliance_tower_pos,
    get_movement_to_tower,
    get_tower_heading,
    is_close_to_tower,
    is_red,
)


class AutoClimber(StateMachine):
    chassis: ChassisComponent
    climber: ClimberComponent

    has_breakbeam_triggered = False
    force_autoclimb = tunable(False)

    DRIVE_SPEED = 0.1
    DRIVE_INTO_TOWER_TIME = 1

    pid_controller = PIDController(2.0, 0.0, 0.0)

    def should_autoclimb(self) -> bool:
        return self.force_autoclimb or is_close_to_tower(
            self.chassis.get_pose().translation()
        )

    def autoclimb(self):
        self.engage()

    def ground(self):
        self.engage(self.grounding, True)

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

    @timed_state(duration=DRIVE_INTO_TOWER_TIME, next_state="climbing")
    def driving_into_tower(self, initial_call: bool, state_tm):
        direction = get_movement_to_tower(self.chassis.get_pose().translation())

        self.chassis.drive_field(
            direction.cos() * self.DRIVE_SPEED, direction.sin() * self.DRIVE_SPEED, 0
        )

    @state
    def climbing(self):
        self.climber.retract()
        self.chassis.stop_snapping()

    @state(must_finish=True)
    def grounding(self):
        self.climber.deploy()
        if self.climber.at_extension_limit():
            self.next_state(self.safing)
            return

    @state(must_finish=True)
    def safing(self):
        if not is_close_to_tower(self.chassis.get_pose().translation()):
            self.done()

    def done(self):
        super().done()
        self.climber.retract()
