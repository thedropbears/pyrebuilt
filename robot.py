import math

import magicbot
import ntcore
import wpilib
import wpilib.event
from magicbot import tunable
from wpimath.geometry import Rotation2d, Translation3d

from autonomous.auto_base import AutoBase
from components.ballistics import BallisticsSolver
from components.chassis import ChassisComponent
from components.climber import ClimberComponent
from components.hopper import HopperComponent
from components.intake import IntakeComponent
from components.leds import LEDComponent
from components.shooter import ShooterComponent
from components.targeter import Targeter
from components.turret import TurretComponent
from components.vision import ServoOffsets, VisualLocalizer
from controllers.conductor import Conductor
from controllers.gobbler import Gobbler
from ids import DioChannel, PwmChannel
from utilities.game import is_red
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # These components have specific ordering concerns with data flow.
    port_vision: VisualLocalizer
    chassis: ChassisComponent
    targeter: Targeter

    # Controllers
    conductor: Conductor
    gobbler: Gobbler

    # Components
    ballistics: BallisticsSolver
    hopper: HopperComponent
    shooter: ShooterComponent
    climber: ClimberComponent
    intake: IntakeComponent
    turret: TurretComponent
    leds: LEDComponent

    # Driving constraints
    max_speed = tunable(3.0)  # m/s
    max_spin_rate = tunable(2.5)  # rad/s

    slowed_speed = tunable(1)

    test_x = tunable(0.0)
    test_y = tunable(0.0)
    test_omega = tunable(0.0)

    test_max_speed = tunable(1.5)
    test_spin_rate = tunable(1)

    test_flywheel_speed = tunable(0.0)  # rotations/s
    test_turret_angle = tunable(0.0)  # degrees
    test_hopper_surface_speed = tunable(12.0)  # metres/s

    START_POS_TOLERANCE = 0.2

    def createObjects(self) -> None:
        self.event_loop = wpilib.event.EventLoop()
        self.data_log = wpilib.DataLogManager.getLog()

        # Log driver station data
        wpilib.DriverStation.startDataLog(self.data_log)

        # Log deploy info to show in AdvantageScope.
        meta_table = ntcore.NetworkTableInstance.getDefault().getTable("Metadata")
        deploy_info = wpilib.deployinfo.getDeployData()
        if deploy_info is not None:
            for k, v in deploy_info.items():
                meta_table.putString(k, v)

        # Also log roboRIO metadata.
        meta_table.putString("runtime_type", self.getRuntimeType().name[1:])
        meta_table.putString("rio_serial", wpilib.RobotController.getSerialNumber())

        self.gamepad = wpilib.XboxController(0)
        self.codriver_joystick = wpilib.Joystick(1)
        self.left_trigger_reset = True

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.mech = wpilib.Mechanism2d(2, 2)
        wpilib.SmartDashboard.putData("Mech2d", self.mech)
        self.intake_mech_root = self.mech.getRoot("Intake", 1.5, 0.1)
        self.frame_mech_root = self.mech.getRoot("A-Frame", 1, 0)
        self.frame_member = self.frame_mech_root.appendLigament(
            "upright", length=1, angle=90, lineWidth=3
        )
        self.wrist_mech_root = self.mech.getRoot("Wrist", 1, 1)

        self.status_lights_strip_length = 112 * 4

        self.port_vision_encoder_id = DioChannel.PORT_VISION_ENCODER
        self.port_vision_servo_id = PwmChannel.PORT_VISION_SERVO

        self.port_vision_name = "port_turret"
        self.port_vision_turret_pos = Translation3d(0.15424, 0.174645, 0.427393)
        self.port_vision_turret_rot = Rotation2d()
        self.port_vision_camera_offset = Translation3d(0.01078, 0, 0.0013)
        self.port_vision_camera_pitch = math.radians(-10.0)
        self.port_vision_encoder_offset = Rotation2d(2.055)
        self.port_vision_servo_offsets = ServoOffsets(
            neutral=Rotation2d(1.928),
            full_range=Rotation2d(3.960),
        )
        self.port_vision_rotation_range = (
            Rotation2d(0.952),
            Rotation2d(3.482),
        )

    def teleopInit(self) -> None:
        self.field.getObject("Intended start pos").setPoses([])

    def teleopPeriodic(self) -> None:
        drive_speed = self.max_speed
        spin_rate = self.max_spin_rate

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            drive_speed = self.slowed_speed

        drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 1.5) * drive_speed
        drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 1.5) * drive_speed
        drive_z = (
            -rescale_js(self.gamepad.getRightX(), 0.1, exponential=2.0) * spin_rate
        )

        local_driving = self.gamepad.getRightBumperButton()

        if local_driving:
            self.chassis.drive_robot(drive_x, drive_y, drive_z)
        else:
            if is_red():
                drive_x = -drive_x
                drive_y = -drive_y
            self.chassis.drive_field(drive_x, drive_y, drive_z)

        if self.gamepad.getYButton():
            self.climber.deploy()

        if self.gamepad.getXButton():
            self.climber.retract()

        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.conductor.shoot()

        if self.gamepad.getRightBumperButton():
            self.conductor.caged_shoot()

        if self.gamepad.getAButton():
            self.port_vision.zero_servo_()

        if self.codriver_joystick.getTrigger():
            self.conductor.log_shot()

        if self.codriver_joystick.getRawButton(4):
            self.ballistics.LATENCY_FACTOR += 0.01

        if self.codriver_joystick.getRawButton(3):
            self.ballistics.LATENCY_FACTOR -= 0.01

        self.leds.execute()

    def testPeriodic(self) -> None:
        allowed_to_drive = self.gamepad.getRightBumperButton()

        if allowed_to_drive:
            # Set max speed
            max_speed = self.test_max_speed
            max_spin_rate = self.test_spin_rate

            if self.gamepad.getXButton():
                drive_x = 0.75 * max_speed
                drive_y = 0.0
                drive_z = 0.0
            else:
                # Driving
                drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 15) * max_speed
                drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 15) * max_speed
                drive_z = (
                    -rescale_js(self.gamepad.getRightX(), 0.1, exponential=20)
                    * max_spin_rate
                )

            self.chassis.drive_robot(drive_x, drive_y, drive_z)
        else:
            self.chassis.stop()

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.gobbler.gobble()

        # if self.gamepad.getAButton():
        #     self.climber.deploy()

        # if self.gamepad.getBButton():
        #     self.climber.retract()

        if self.gamepad.getLeftBumperButton():
            self.hopper.feed(self.test_hopper_surface_speed)

        # if self.gamepad.getYButton():
        #     self.shooter.pitch_to(math.radians(self.test_hood_angle))

        if self.gamepad.getPOV() == 90:
            self.chassis.flash_gains(True)

        if self.gamepad.getPOV() == 270:
            self.chassis.flash_gains(False)

        if self.gamepad.getPOV() == 0:
            self.test_drive_inverted = -1
        else:
            self.test_drive_inverted = 1

        if self.gamepad.getXButton():
            self.chassis.drive_robot(self.test_x * self.test_drive_inverted, 0, 0)

        if self.gamepad.getYButton():
            self.chassis.drive_robot(0, self.test_y * self.test_drive_inverted, 0)

        if self.gamepad.getAButton():
            self.chassis.drive_robot(0, 0, self.test_omega * self.test_drive_inverted)

        if self.gamepad.getLeftStickButton():
            self.port_vision.zero_servo_()
        elif self.gamepad.getRightStickButton():
            self.port_vision.full_range_servo_()

        self.port_vision.execute()
        self.chassis.execute()
        self.targeter.execute()
        if self.gamepad.getRightTriggerAxis() > 0.5:
            self.hopper.feed(self.test_hopper_surface_speed)
            self.turret.slew_to(math.radians(self.test_turret_angle))
            self.shooter.set_flywheel(self.test_flywheel_speed)

        self.gobbler.execute()
        self.shooter.execute()
        self.climber.execute()
        self.intake.execute()
        self.leds.execute()
        self.turret.execute()
        self.hopper.execute()

    def disabledPeriodic(self) -> None:
        self.event_loop.poll()

        selected_auto = self._automodes.chooser.getSelected()
        if isinstance(selected_auto, AutoBase):
            intended_start_pose = selected_auto.get_starting_pose()
            if intended_start_pose is not None:
                self.field.getObject("Intended start pos").setPose(intended_start_pose)

        self.climber.try_index()

        self.chassis.update_alliance()
        self.port_vision.execute()
        self.chassis.update_odometry()
        self.leds.execute()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.port_vision._per_loop_cache.clear()
        self.turret.periodic()
        self.intake.periodic()
