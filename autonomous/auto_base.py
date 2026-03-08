import choreo
import wpilib
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import AutonomousStateMachine, state
from wpilib import RobotBase
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

from components.chassis import ChassisComponent
from controllers.conductor import Conductor
from utilities import game

x_controller = PIDController(2.0, 0.0, 0.0)
y_controller = PIDController(2.0, 0.0, 0.0)

wpilib.SmartDashboard.putData("Auto X PID", x_controller)
wpilib.SmartDashboard.putData("Auto Y PID", y_controller)


class AutoBase(AutonomousStateMachine):
    field: wpilib.Field2d
    chassis: ChassisComponent
    shooter_state_machine: Conductor

    def __init__(self, trajectory_names: list[str], actions: list[str]) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()

        self.current_leg = -1
        self.starting_pose = None
        self.trajectories: list[SwerveTrajectory] = []
        self.actions = actions
        for trajectory_name in trajectory_names:
            try:
                self.trajectories.append(choreo.load_swerve_trajectory(trajectory_name))
                if self.starting_pose is None:
                    self.starting_pose = self.get_starting_pose()
            except ValueError:
                # If the trajectory is not found, ChoreoLib already prints to DriverStation
                pass

    def setup(self) -> None:
        #  setup path tracking controllers
        self.auto_sample_field_obj = self.field.getObject("auto_sample")

        # init any other defaults
        pass

    def on_enable(self) -> None:
        # configure defaults for pose in sim

        # Setup starting position in the simulator
        starting_pose = self.get_starting_pose()
        if RobotBase.isSimulation() and starting_pose is not None:
            self.chassis.set_pose(starting_pose)
        # Reset the counter for which leg we are executing
        self.current_leg = -1

        super().on_enable()

    def get_starting_pose(self) -> Pose2d | None:
        return self.trajectories[0].get_initial_pose(game.is_red())

    def _get_full_path_poses(self) -> list[Pose2d]:
        """Get a list of poses for the full path for display."""
        return [
            sample.get_pose()
            for trajectory in self.trajectories
            for sample in trajectory.get_samples()
        ]

    def display_trajectory(self) -> None:
        self.field.getObject("trajectory").setPoses(self._get_full_path_poses())

    def on_disable(self) -> None:
        super().on_disable()
        self.field.getObject("trajectory").setPoses([])

    @state(first=True)
    def initialising(self) -> None:
        # Add any tasks that need doing first
        self.chassis.do_smooth = False
        self.chassis.heading_controller.setPID(Kp=1.0, Ki=0.0, Kd=0.0)
        self.next_state("tracking_trajectory")

    @state
    def tracking_trajectory(self, initial_call, state_tm) -> None:
        if initial_call:
            self.current_leg += 1

            if self.current_leg == len(self.trajectories):
                self.done()
                return

            trajectory = (
                self.trajectories[self.current_leg].flipped()
                if game.is_red()
                else self.trajectories[self.current_leg]
            )

            trajectory_poses = trajectory.get_poses()
            self.field.getObject("trajectory").setPoses(trajectory_poses)

        final_pose = self.trajectories[self.current_leg].get_final_pose(game.is_red())
        if final_pose is None:
            self.done()
            return

        action = self.actions[self.current_leg]
        if action == "shoot":
            self.shooter_state_machine.shoot()
        elif action == "cage":
            # self.conductor.cage() # We need the cage method for this
            pass

        sample = self.trajectories[self.current_leg].sample_at(state_tm, game.is_red())
        if sample is not None:
            self.follow_trajectory(sample)
            self.auto_sample_field_obj.setPose(sample.get_pose())

        if state_tm > self.trajectories[self.current_leg].get_total_time():
            self.next_state("tracking_trajectory")

    def follow_trajectory(self, sample: SwerveSample):
        # track path

        pose = self.chassis.get_pose()

        # Generate the next speeds for the robot
        speeds = ChassisSpeeds(
            sample.vx + x_controller.calculate(pose.X(), sample.x),
            sample.vy + y_controller.calculate(pose.Y(), sample.y),
            sample.omega
            + self.chassis.heading_controller.calculate(
                pose.rotation().radians(), sample.heading
            ),
        )

        # Apply the generated speeds
        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)
