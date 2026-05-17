from math import hypot, pi

import choreo
import wpilib
from choreo.trajectory import SwerveSample, SwerveTrajectory
from magicbot import AutonomousStateMachine, state
from wpilib import RobotBase
from wpimath.controller import (
    HolonomicDriveController,
    PIDController,
    ProfiledPIDControllerRadians,
)
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrapezoidProfileRadians

from components.chassis import ChassisComponent
from utilities import game

controller = HolonomicDriveController(
    PIDController(2.0, 0.0, 0.0),
    PIDController(2.0, 0.0, 0.0),
    ProfiledPIDControllerRadians(
        1.0, 0.0, 0.0, TrapezoidProfileRadians.Constraints(2 * pi, pi)
    ),
)
controller.setTolerance(Pose2d(0.01, 0.01, Rotation2d.fromDegrees(1)))

wpilib.SmartDashboard.putData("Auto X PID", controller.getXController())
wpilib.SmartDashboard.putData("Auto Y PID", controller.getYController())
wpilib.SmartDashboard.putData("Auto Theta PID", controller.getThetaController())


class AutoBase(AutonomousStateMachine):
    field: wpilib.Field2d
    chassis: ChassisComponent

    def __init__(self, trajectory_names: list[str]) -> None:
        # We want to parameterise these by paths and potentially a sequence of events
        super().__init__()

        self.current_leg = -1
        self.starting_pose = None
        self.trajectories: list[SwerveTrajectory] = []
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
        speeds = controller.calculate(
            pose,
            sample.get_pose(),
            hypot(sample.vx, sample.vy),
            Rotation2d(sample.heading),
        )

        # Apply the generated speeds
        self.chassis.drive_field(speeds.vx, speeds.vy, speeds.omega)
