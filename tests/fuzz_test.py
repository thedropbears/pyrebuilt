from __future__ import annotations

import os
import random
import typing

import hal
import pytest
import wpilib.simulation
from wpilib.simulation import DriverStationSim

if typing.TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController

pytestmark = pytest.mark.integration_test


def rand_bool() -> bool:
    return random.getrandbits(1) != 0


def rand_axis() -> float:
    """Get a random number between -1 and 1."""
    return random.random() * 2 - 1


def rand_pov() -> wpilib.POVDirection:
    """Pick a random POV hat direction."""
    import wpilib
    return random.choice(list(wpilib.POVDirection.__members__.values()))


class AllTheThings:
    """Fuzzer for robot hardware inputs."""

    def __init__(self) -> None:
        # 2027: getNumDigitalChannels removed; use fixed channel range
        num_dio = getattr(wpilib.SensorUtil, "getNumDigitalChannels", lambda: 26)()
        self.dios = [
            dio
            for dio in map(
                wpilib.simulation.DIOSim,
                range(num_dio),
            )
            if dio.getInitialized()
        ]

    def fuzz(self) -> None:
        for dio in self.dios:
            if dio.getIsInput():  # pragma: no branch
                dio.setValue(rand_bool())


class DSInputs:
    """Fuzzer for HIDs attached to the driver station."""

    def __init__(self) -> None:
        # 2027: XboxControllerSim renamed to NiDsXboxControllerSim
        XboxSim = getattr(wpilib.simulation, "XboxControllerSim", None) or wpilib.simulation.NiDsXboxControllerSim
        self.gamepad = XboxSim(0)
        self.joystick = wpilib.simulation.JoystickSim(1)

    def fuzz(self) -> None:
        fuzz_xbox_gamepad(self.gamepad)
        fuzz_joystick(self.joystick)


def fuzz_joystick(joystick: wpilib.simulation.JoystickSim) -> None:
    """Fuzz a Logitech Extreme 3D Pro flight stick."""
    for axis in range(5):
        joystick.setRawAxis(axis, rand_axis())
    for button in range(12):
        joystick.setRawButton(button, rand_bool())
    joystick.setPOV(rand_pov())


def fuzz_xbox_gamepad(gamepad: wpilib.simulation.XboxControllerSim) -> None:
    """Fuzz an XInput gamepad."""
    gamepad.setLeftX(rand_axis())
    gamepad.setLeftY(rand_axis())
    gamepad.setRightX(rand_axis())
    gamepad.setRightY(rand_axis())
    gamepad.setLeftTriggerAxis(random.random())
    gamepad.setRightTriggerAxis(random.random())
    for button in range(10):
        gamepad.setRawButton(button, rand_bool())
    gamepad.setPOV(rand_pov())


def get_alliance_stations() -> list[str]:
    choices_env_var = "FUZZ_ALLIANCE_STATIONS"
    choices_env = os.environ.get(choices_env_var, None)
    if choices_env is not None:  # pragma: no cover
        return choices_env.split(",")

    stations = (1, 2, 3)
    if "CI" in os.environ:  # pragma: no branch
        choices = [
            f"{alliance}{station}"
            for alliance in ("Blue", "Red")
            for station in stations
        ]
    else:  # pragma: no cover
        choices = [f"Blue{random.choice(stations)}", f"Red{random.choice(stations)}"]

    os.environ[choices_env_var] = ",".join(choices)
    return choices


@pytest.mark.parametrize("station", get_alliance_stations())
def test_fuzz(control: TestController, station: str) -> None:
    # 2027: AllianceStationID enum uses BLUE_1/RED_3 instead of kBlue1/kRed3
    import re
    enum_name = re.sub(r"(\D)(\d)", r"\1_\2", station).upper()
    station_id = getattr(hal.AllianceStationID, enum_name)

    with control.run_robot():
        things = AllTheThings()
        hids = DSInputs()

        # Disabled mode
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        DriverStationSim.setAllianceStationId(station_id)
        things.fuzz()
        hids.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)

        # Autonomous mode
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=True, enabled=False)
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=True, enabled=True)

        # Transition between autonomous and teleop
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=False)
        things.fuzz()
        control.step_timing(seconds=0.2, autonomous=False, enabled=True)

        # Teleop
        for _ in range(20):
            things.fuzz()
            hids.fuzz()
            control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        DriverStationSim.setAllianceStationId(hal.AllianceStationID.UNKNOWN)


def test_fuzz_test(control: TestController) -> None:
    with control.run_robot():
        hids = DSInputs()

        # Start the robot in disabled mode for a short period
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        # ... in disabled test mode too
        DriverStationSim.setTest(True)
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        DriverStationSim.setEnabled(True)

        assert control.robot_is_alive

        for _ in range(20):
            hids.fuzz()
            DriverStationSim.notifyNewData()
            wpilib.simulation.stepTiming(0.2)
            assert control.robot_is_alive
