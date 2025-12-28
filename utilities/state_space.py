import numpy as np
from wpimath import units
from wpimath.system import LinearSystem_2_1_1
from wpimath.system.plant import DCMotor


def single_jointed_arm_system(
    motor: DCMotor, J: units.kilogram_square_meters, gearing: float
) -> LinearSystem_2_1_1:
    A = np.array(
        [[0.0, 1.0], [0.0, -(gearing**2) * motor.Kt / (motor.Kv * motor.R * J)]]
    )
    B = np.array([[0.0], [gearing * motor.Kt / (motor.R * J)]])
    C = np.array([1.0, 0.0])
    D = np.array([0.0])

    return LinearSystem_2_1_1(A, B, C, D)
