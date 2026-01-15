import magicbot
import ntcore
import wpilib
import wpilib.event
from magicbot import tunable
from phoenix6.configs import Slot0Configs

from components.chassis import ChassisComponent, SwerveConfig
from components.shooter import ShooterComponent
from ids import DioChannel, PwmChannel, RioSerialNumber
from utilities.scalers import rescale_js


class MyRobot(magicbot.MagicRobot):
    # Components
    chassis: ChassisComponent
    shooter: ShooterComponent
    max_speed = tunable(3.5)  # m/s
    lower_max_speed = tunable(2.0)  # m/s
    max_spin_rate = tunable(2.8)  # m/s
    lower_max_spin_rate = tunable(2.0)  # m/s
    inclination_angle = tunable(0.0)
    dpad_max_speed = tunable(0.4)
    is_robot_oriented = tunable(False)

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

        self.starboard_vision_encoder_id = DioChannel.STARBOARD_VISION_ENCODER
        self.starboard_vision_servo_id = PwmChannel.STARBOARD_VISION_SERVO
        self.port_vision_encoder_id = DioChannel.PORT_VISION_ENCODER
        self.port_vision_servo_id = PwmChannel.PORT_VISION_SERVO
        if wpilib.RobotController.getSerialNumber() == RioSerialNumber.TEST_BOT:
            self.chassis_swerve_config = SwerveConfig(
                drive_ratio=(14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
                drive_gains=Slot0Configs()
                .with_k_p(1.0868)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.15172)
                .with_k_v(2.8305)
                .with_k_a(0.082659),
                steer_ratio=(14 / 50) * (10 / 60),
                steer_gains=Slot0Configs()
                .with_k_p(30.234)
                .with_k_i(0)
                .with_k_d(0.62183)
                .with_k_s(0.1645),
                reverse_drive=False,
            )
            # metres between centre of left and right wheels
            self.chassis_track_width = 0.467
            # metres between centre of front and back wheels
            self.chassis_wheel_base = 0.467

        else:
            self.chassis_swerve_config = SwerveConfig(
                drive_ratio=(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
                drive_gains=Slot0Configs()
                .with_k_p(7.8294)
                .with_k_i(0)
                .with_k_d(0)
                .with_k_s(0.11742)
                .with_k_v(2.3941)
                .with_k_a(0.11426),
                steer_ratio=(14 / 50) * (10 / 60),
                steer_gains=Slot0Configs()
                .with_k_p(92.079)
                .with_k_i(0)
                .with_k_d(1.6683)
                .with_k_s(0.086374),
                reverse_drive=True,
            )
            # metres between centre of left and right wheels
            self.chassis_track_width = 0.517
            # metres between centre of front and back wheels
            self.chassis_wheel_base = 0.517

    def teleopInit(self) -> None:
        self.field.getObject("Intended start pos").setPoses([])
        self.chassis.set_coast_in_neutral(False)

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        self.chassis.set_coast_in_neutral(True)

    def testPeriodic(self) -> None:
        allowed_to_drive = self.gamepad.getRightBumperButton()
        if allowed_to_drive:
            # Set max speed
            max_speed = self.lower_max_speed
            max_spin_rate = self.lower_max_spin_rate

            # Driving
            drive_x = -rescale_js(self.gamepad.getLeftY(), 0.05, 15) * max_speed
            drive_y = -rescale_js(self.gamepad.getLeftX(), 0.05, 15) * max_speed
            drive_z = (
                -rescale_js(self.gamepad.getRightX(), 0.1, exponential=20)
                * max_spin_rate
            )

            self.chassis.drive_local(drive_x, drive_y, drive_z)
            self.chassis.execute()
        else:
            self.chassis.stop()

        if self.gamepad.getLeftTriggerAxis() > 0.5:
            self.shooter.set_speed()

        self.shooter.execute()

    def disabledPeriodic(self) -> None:
        self.event_loop.poll()

        self.chassis.update_alliance()
        self.chassis.update_odometry()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
