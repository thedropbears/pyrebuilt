import hal
import pytest
from pyfrc.test_support.controller import TestController as RobotTestController
from robotpy_ext.autonomous.selector_tests import (  # type: ignore[import-untyped]
    test_all_autonomous as _test_all_autonomous,
)
from wpilib.simulation import DriverStationSim


@pytest.mark.slow_integration_test
@pytest.mark.parametrize("alliance", ["Red", "Blue"])
def test_all_autonomous(control: RobotTestController, alliance: str):
    station: hal.AllianceStationID = getattr(hal.AllianceStationID, f"k{alliance}1")
    DriverStationSim.setAllianceStationId(station)

    _test_all_autonomous(control)

    # Clean up the global simulation state we set.
    DriverStationSim.setAllianceStationId(hal.AllianceStationID.kUnknown)
