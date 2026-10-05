import math
import pytest
from ab_robot.core import GeoReference, RouteState, validate_route


def test_origin_and_east_north():
    ref = GeoReference(55.75, 37.61, -4.0, -3.0)
    assert ref.to_map(55.75, 37.61) == pytest.approx((-4, -3))
    lat, lon = ref.to_gps(2.0, 3.0)
    assert lat > 55.75 and lon > 37.61
    assert ref.to_map(lat, lon) == pytest.approx((2, 3), abs=0.001)


def test_rotated_map():
    ref = GeoReference(0.0, 0.0, 0, 0, math.pi / 2)
    lat, lon = ref.to_gps(10, 0)
    assert lat < 0 and abs(lon) < 1e-8
    assert ref.to_map(lat, lon) == pytest.approx((10, 0), abs=0.001)


@pytest.mark.parametrize('lat,lon', [(91,0),(0,181),(math.nan,0),(0,math.inf)])
def test_invalid_gps(lat, lon):
    with pytest.raises(ValueError):
        GeoReference(lat, lon)


def test_geographic_range_is_bounded():
    with pytest.raises(ValueError):
        GeoReference(55, 37).to_map(56, 37)


def test_success_advances_once():
    state = RouteState(2, 10)
    state.start(0)
    state.result(True)
    assert state.index == 1 and not state.finished
    state.start(1)
    state.result(True)
    assert state.finished and state.success


def test_failure_stops_route():
    state = RouteState(2, 10)
    state.start(0)
    state.result(False)
    assert state.finished and not state.success and state.index == 0


def test_timeout_includes_goal_acknowledgement():
    state = RouteState(2, 10)
    state.start(4)
    assert not state.expired(13.9)
    assert state.expired(14)
    state.result(False)
    assert state.finished


def test_rejects_empty_and_nonfinite_route():
    with pytest.raises(ValueError):
        validate_route([])
    with pytest.raises(ValueError):
        validate_route([{'latitude': 55, 'longitude': 37, 'yaw': math.nan}])


def test_state_rejects_duplicate_result():
    state = RouteState(2, 10)
    state.start(0)
    state.result(True)
    with pytest.raises(RuntimeError):
        state.result(True)


def test_watchdog_stops_on_stale_sensor_or_command():
    from ab_robot.core import command_allowed
    assert command_allowed(10, {'cmd':9.9,'scan':9,'depth':9,'gps':9}, False, 3, .5)
    assert not command_allowed(10, {'cmd':9.9,'scan':9,'depth':6,'gps':9}, False, 3, .5)
    assert not command_allowed(10, {'cmd':9,'scan':9,'depth':9,'gps':9}, False, 3, .5)
    assert not command_allowed(10, {'cmd':9.9,'scan':9,'depth':9,'gps':9}, True, 3, .5)
    assert not command_allowed(10, {'cmd':9.9,'scan':9,'gps':9}, False, 3, .5)


def test_contact_distinguishes_floor_from_obstacle():
    from ab_robot.core import obstacle_contact
    assert not obstacle_contact('ab_robot::left_wheel','ground::link::collision')
    assert obstacle_contact('ab_robot::base','low::link::collision')
