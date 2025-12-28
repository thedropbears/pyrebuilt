import math

import pytest
from hypothesis import given
from hypothesis.strategies import floats

from utilities.functions import constrain_angle


@given(angle=floats(-math.pi, math.pi))
def test_happy(angle: float):
    """Test the happy path: the angle is in [-pi,pi]."""
    assert constrain_angle(angle) == pytest.approx(angle)


@given(angle=floats(allow_infinity=False, allow_nan=False))
def test_all(angle: float):
    assert -math.pi <= constrain_angle(angle) <= math.pi


def test_zero():
    assert constrain_angle(0) == 0


def test_edge_pos():
    assert constrain_angle(math.pi) == pytest.approx(math.pi)


def test_edge_neg():
    assert constrain_angle(-math.pi) == pytest.approx(-math.pi)


def test_revolution_pos():
    assert constrain_angle(math.tau) == pytest.approx(0)


def test_revolution_neg():
    assert constrain_angle(-math.tau) == pytest.approx(0)


@given(angle=floats(-math.tau, -math.pi, exclude_max=True))
def test_one_wrap_positive_half(angle: float):
    assert constrain_angle(angle) == pytest.approx(angle + math.tau)


@given(angle=floats(math.pi, math.tau, exclude_min=True))
def test_one_wrap_negative_half(angle: float):
    assert constrain_angle(angle) == pytest.approx(angle - math.tau)
