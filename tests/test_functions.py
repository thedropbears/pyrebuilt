from math import cos, hypot, sin

from hypothesis import given
from hypothesis.strategies import floats, tuples
from pytest import approx

from utilities.functions import clamp_2d, rate_limit_2d

sensible_floats = floats(allow_infinity=False, allow_nan=False, width=16)
sensible_positive_floats = floats(
    min_value=0, allow_infinity=False, allow_nan=False, width=16
)


def test_rate_limit2d():
    assert rate_limit_2d((0, 0), (1, 0), rate_limit=0.5, dt=1) == (0.5, 0)
    assert rate_limit_2d((0, 0), (1, 0), rate_limit=2, dt=1) == (1, 0)
    assert rate_limit_2d((0, 0), (1, 1), rate_limit=0, dt=1) == (0, 0)
    assert rate_limit_2d((0, 0), (1, 1), rate_limit=1, dt=0) == (0, 0)


@given(
    cur=tuples(sensible_floats, sensible_floats),
    target=tuples(sensible_floats, sensible_floats),
    rate_limit=sensible_positive_floats,
    dt=sensible_positive_floats,
)
def test_rate_limit_2d_limit(cur, target, rate_limit, dt):
    new = rate_limit_2d(cur, target, rate_limit, dt)
    diff = new[0] - cur[0], new[1] - cur[1]
    assert hypot(*diff) <= rate_limit * dt or hypot(*diff) == approx(rate_limit * dt)


@given(mag=floats(0, 1), angle=sensible_floats)
def test_clamp2d_noconstrain(mag, angle):
    # generate a point in the unit circle
    x = cos(angle) * mag
    y = sin(angle) * mag
    # make sure its not contrained
    assert clamp_2d((x, y), 1) == approx((x, y))


@given(x=sensible_floats, y=sensible_floats)
def test_clamp2d_constrain(x, y):
    result = clamp_2d((x, y), 1)
    magnitude = hypot(*result)
    assert magnitude <= 1 or magnitude == approx(1)
