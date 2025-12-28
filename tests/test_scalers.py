import math

from hypothesis import assume, given
from hypothesis.strategies import floats, tuples
from pytest import approx

from utilities.scalers import apply_deadzone, map_exponential, scale_value


@given(value=floats(0, 1), threshold=floats(0, 1, exclude_min=True, exclude_max=True))
def test_deadzone(value, threshold):
    result = apply_deadzone(value, threshold)
    neg_result = apply_deadzone(-value, threshold)
    if value in (0, 1):
        assert result == value
    elif math.isclose(value, 0, abs_tol=threshold):
        assert result == 0
    else:
        assert 0 <= result <= value
    assert neg_result == -result


@given(value=floats(0, 1))
def test_deadzone_zero_threshold(value):
    result = apply_deadzone(value, 0)
    neg_result = apply_deadzone(-value, 0)
    assert result == value
    assert neg_result == -value


@given(
    value=floats(0, 1), base=floats(1, exclude_min=True, allow_infinity=False, width=16)
)
def test_exponential(value, base):
    result = map_exponential(value, base)
    neg_result = map_exponential(-value, base)
    if value in (0, 1):
        assert result == value
    else:
        assert 0 <= result <= value or result == approx(value)
    assert neg_result == -result


real_halves = floats(allow_nan=False, allow_infinity=False, width=16)


@given(
    value=real_halves,
    input_range=tuples(real_halves, real_halves).filter(lambda x: x[0] != x[1]),
    output_range=tuples(real_halves, real_halves).filter(lambda x: x[0] != x[1]),
)
def test_scale_value(
    value: float, input_range: tuple[float, float], output_range: tuple[float, float]
):
    input_lower, input_upper = input_range
    output_lower, output_upper = output_range
    assume(min(input_lower, input_upper) <= value <= max(input_lower, input_upper))
    result = scale_value(value, input_lower, input_upper, output_lower, output_upper)
    assert min(output_range) <= result <= max(output_range)
    if value == input_lower:
        assert result == output_lower
    elif value == input_upper:
        assert result == output_upper
