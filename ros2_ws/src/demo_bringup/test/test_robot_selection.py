"""Tests for the coupled plant/navigation robot selector."""

from demo_bringup.robot_selection import launch_file, ROBOT_LAUNCH_FILES
import pytest


def test_both_public_robot_types_are_supported() -> None:
    """F6 keeps the old robot available after promoting the Go2."""
    assert set(ROBOT_LAUNCH_FILES) == {'diffdrive', 'quadruped'}


@pytest.mark.parametrize(
    ('robot_type', 'plant', 'navigation'),
    [
        ('diffdrive', 'simulation.launch.py', 'nav.launch.py'),
        ('quadruped', 'quadruped.launch.py', 'nav_quadruped.launch.py'),
    ],
)
def test_robot_selects_a_matching_pair(
        robot_type: str, plant: str, navigation: str) -> None:
    """One selector must choose both halves of the same robot stack."""
    assert launch_file(robot_type, 'plant') == plant
    assert launch_file(robot_type, 'navigation') == navigation


def test_unknown_robot_fails_loudly() -> None:
    """A typo must not fall back to a plausible but wrong robot."""
    with pytest.raises(ValueError, match='not a known robot'):
        launch_file('go2', 'plant')


def test_unknown_role_fails_loudly() -> None:
    """The mapping must not silently grow an unpaired runtime role."""
    with pytest.raises(ValueError, match='role'):
        launch_file('quadruped', 'driver')
