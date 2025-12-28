import math


def apply_deadzone(value: float, threshold: float) -> float:
    """Apply a deadzone to a value in [-1,1].

    This ensures that the rest of the input space maps to [-1,1].
    """
    assert 0 <= threshold < 1
    if abs(value) < threshold:
        return 0
    return (value - math.copysign(threshold, value)) / (1 - threshold)


def map_exponential(value: float, base: float) -> float:
    """Takes a value in [-1,1] and maps it to an exponential curve."""
    assert base > 1
    return math.copysign((base ** abs(value) - 1) / (base - 1), value)


def rescale_js(value: float, deadzone: float, exponential: float = 1.5) -> float:
    """Rescale a joystick input, applying a deadzone and exponential.

    Args:
        value: the joystick value, in the interval [-1, 1].
        deadzone: the deadzone to apply.
        exponential: the strength of the exponential to apply
                     (i.e. how non-linear should the response be)
    """
    return map_exponential(apply_deadzone(value, deadzone), exponential)


def scale_value(
    value: float,
    input_lower: float,
    input_upper: float,
    output_lower: float,
    output_upper: float,
) -> float:
    """Scales a value based on the input range and output range.
    For example, to scale a joystick throttle (1 to -1) to 0-1, we would:
        scale_value(joystick.getThrottle(), 1, -1, 0, 1)
    """
    input_distance = input_upper - input_lower
    output_distance = output_upper - output_lower
    ratio = (value - input_lower) / input_distance
    result = ratio * output_distance + output_lower
    return result


def lerp(value: float, lo: float, hi: float) -> float:
    return value * (hi - lo) + lo
