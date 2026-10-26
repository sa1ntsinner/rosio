from rosio import stamps
from rosio.stamps import clock
from rosio.stamps import read as bag


def test_public_api_is_importable_from_the_package_root():
    for name in stamps.__all__:
        assert hasattr(stamps, name), name


def test_private_aliases_still_point_at_the_public_names():
    assert bag._stamp_cdr is stamps.stamp_from_cdr
    assert bag._starts_with_header is stamps.starts_with_header
    assert bag._probe is stamps.probe_stamp_parser
    assert clock._classify is stamps.classify_offset


def test_largest_backward_step_finds_a_single_jump():
    assert stamps.largest_backward_step([1.0, 1.1, 1.2]) == (0.0, -1)
    drop, index = stamps.largest_backward_step([1.0, 1.1, 0.9, 1.3])
    assert round(drop, 9) == 0.2
    assert index == 2


def test_longest_frozen_run_needs_the_offline_thresholds():
    frozen = [5.0] * 12
    receive = [i * 0.2 for i in range(12)]
    run, span, stamp, index = stamps.longest_frozen_run(frozen, receive)
    assert (run, stamp, index) == (12, 5.0, 0)
    assert span >= stamps.FROZEN_MIN_SPAN_S
    assert stamps.longest_frozen_run(frozen[:9], receive[:9]) is None
    assert stamps.longest_frozen_run(frozen, [i * 0.01 for i in range(12)]) is None
