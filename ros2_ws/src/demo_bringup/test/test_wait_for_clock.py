"""
Unit tests for the /clock gate that replaced learn.launch.py's Nav2 timer.

The property under test is the one the node exists for: a clock that publishes
but does not advance must NOT release the launch. A paused Gazebo is
indistinguishable from a running one if you only check that messages arrive, and
letting Nav2 start against a frozen clock reproduces exactly the silent stall
that F1 set out to remove.

is_advancing and to_nanoseconds are pure, so the paused, running, reset and
first-sample cases are all testable without a live graph — no simulator, no DDS,
no timing flakiness.
"""

from builtin_interfaces.msg import Time
from demo_bringup.wait_for_clock import ClockWaiter, is_advancing, to_nanoseconds
import pytest
from rosgraph_msgs.msg import Clock


def _make_clock(sec: int, nanosec: int = 0) -> Clock:
    msg = Clock()
    msg.clock = Time(sec=sec, nanosec=nanosec)
    return msg


def test_first_sample_never_releases() -> None:
    """One message proves the topic exists, not that time is moving."""
    assert is_advancing(None, to_nanoseconds(_make_clock(5))) is False


def test_advancing_clock_releases() -> None:
    """The normal case: a running simulator publishes increasing stamps."""
    previous = to_nanoseconds(_make_clock(5))
    current = to_nanoseconds(_make_clock(6))

    assert is_advancing(previous, current) is True


def test_paused_clock_does_not_release() -> None:
    """
    The failure this node exists to catch.

    `gz sim` without -r starts paused and republishes a constant stamp forever.
    Releasing here would hand Nav2 a clock that never advances, and its
    lifecycle nodes would stall on timestampless transforms with nothing in the
    logs naming the cause.
    """
    frozen = to_nanoseconds(_make_clock(5, 250))

    assert is_advancing(frozen, frozen) is False


def test_sub_second_advance_releases() -> None:
    """Sim time advances in millisecond steps; whole seconds are not required."""
    previous = to_nanoseconds(_make_clock(5, 100_000_000))
    current = to_nanoseconds(_make_clock(5, 200_000_000))

    assert is_advancing(previous, current) is True


def test_clock_going_backwards_does_not_release() -> None:
    """A reset simulator jumped backwards; TF holds only stale transforms."""
    previous = to_nanoseconds(_make_clock(10))
    current = to_nanoseconds(_make_clock(2))

    assert is_advancing(previous, current) is False


def test_nanoseconds_conversion_combines_both_fields() -> None:
    """Ignoring the nanosec field would make millisecond ticks look frozen."""
    assert to_nanoseconds(_make_clock(2, 500_000_000)) == 2_500_000_000


def test_rejects_non_positive_timeout() -> None:
    """A zero timeout would expire before the first spin slice."""
    with pytest.raises(ValueError, match=r'timeout_s must be > 0'):
        ClockWaiter._validate_timeout(0.0)
