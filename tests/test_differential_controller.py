import numpy as np
import pytest

from reconciliation.controllers.differential import (
    differential_wheel_speeds,
    map_wheel_speeds_by_side,
)


RADIUS = 0.1
SEPARATION = 0.4


@pytest.mark.parametrize(
    ("linear", "angular", "expected"),
    [
        (0.0, 0.0, [0.0, 0.0]),
        (0.5, 0.0, [5.0, 5.0]),
        (0.0, 0.25, [-0.5, 0.5]),
        (0.0, -0.25, [0.5, -0.5]),
        (0.4, 0.25, [3.5, 4.5]),
    ],
)
def test_differential_formula_cases(linear, angular, expected) -> None:
    np.testing.assert_allclose(
        differential_wheel_speeds(linear, angular, RADIUS, SEPARATION), expected
    )


def test_four_wheel_side_mapping_preserves_runtime_order() -> None:
    mapped = map_wheel_speeds_by_side(
        [-2.0, 3.0], ["right", "left", "right", "left"]
    )
    np.testing.assert_array_equal(mapped, [3.0, -2.0, 3.0, -2.0])


@pytest.mark.parametrize(
    "arguments",
    [(np.nan, 0.0, RADIUS, SEPARATION), (0.0, np.inf, RADIUS, SEPARATION), (0, 0, 0, 1)],
)
def test_differential_formula_rejects_invalid_values(arguments) -> None:
    with pytest.raises(ValueError):
        differential_wheel_speeds(*arguments)
