"""config/station.yaml: what the setup app writes, and how it combines with .env."""
import os

import yaml

from ros2_vive_controller import launch_config as lc
from ros2_vive_controller import station as st

BOX = {'x_min': -0.5, 'x_max': 0.5, 'y_min': -0.4, 'y_max': 0.4, 'z_min': 0.1, 'z_max': 0.9}
ENV = {'SERIAL_RIGHT': 'LHR-ENV-R # right hand', 'SERIAL_LEFT': 'LHR-ENV-L',
       'REFERENCE_LIGHTHOUSE_SERIAL': 'LHB-ENV'}


def test_no_file_means_nothing_chosen(tmp_path):
    station = st.load(str(tmp_path / 'station.yaml'))
    assert station == st.empty()
    assert st.resolve_device('right', '', station, ENV) == ('LHR-ENV-R', st.SOURCE_ENV)
    assert st.resolve_device('right', '', station, {}) == ('', st.NOT_SET)


def test_round_trip_with_backup(tmp_path):
    path = str(tmp_path / 'station.yaml')
    station = {'robot': 'franka', 'devices': {'right': 'LHR-R', 'left': '', 'origin_lighthouse': 'LHB-O'}}
    station = st.set_fence(station, 'right', BOX, 'lighthouse_origin')
    assert st.save(path, station) is None
    assert st.load(path) == st.normalize(station)
    text = open(path).read()
    assert text.startswith('# This machine')
    assert yaml.safe_load(text)['workspace']['right']['frame_id'] == 'lighthouse_origin'

    station['robot'] = 'tiago'
    assert st.save(path, station) == path + '.bak'
    assert st.load(path + '.bak')['robot'] == 'franka'
    assert st.load(path)['robot'] == 'tiago'
    assert not os.path.exists(path + '.tmp')


def test_station_wins_over_env_even_when_empty():
    """Choosing "no origin base station" in the app must not be undone by an old .env value."""
    station = st.normalize({'devices': {'right': 'LHR-APP', 'origin_lighthouse': ''}})
    assert st.resolve_device('right', '', station, ENV) == ('LHR-APP', st.SOURCE_STATION)
    assert st.resolve_device('origin_lighthouse', '', station, ENV) == ('', st.SOURCE_STATION)
    # Not listed in the file: .env still applies.
    assert st.resolve_device('left', '', station, ENV) == ('LHR-ENV-L', st.SOURCE_ENV)
    # A launch argument wins over both.
    assert st.resolve_device('right', 'LHR-ARG', station, ENV) == ('LHR-ARG', st.SOURCE_ARGUMENT)


def test_unset_devices_are_not_written(tmp_path):
    """A fence saved by `make calibrate` before the app was used must not freeze the .env serials."""
    path = str(tmp_path / 'station.yaml')
    st.save(path, st.set_fence(st.empty(), 'right', BOX, 'vive_world'), writer='calibration_node')
    assert 'devices' not in yaml.safe_load(open(path))
    assert st.resolve_device('right', '', st.load(path), ENV) == ('LHR-ENV-R', st.SOURCE_ENV)


def test_fence_without_all_bounds_is_never_enabled():
    fence = st.normalize_fence({'enabled': True, 'frame_id': 'vive_world', 'x_min': -1})
    assert fence['enabled'] is False


def test_garbage_is_ignored():
    assert st.normalize(None) == st.empty()
    station = st.normalize({'devices': 'LHR', 'workspace': {'middle': BOX, 'left': 'x'}})
    assert station['workspace'] == {}
    assert station['devices'] == st.empty()['devices']


def test_fence_becomes_driver_parameters():
    fence = st.set_fence(st.empty(), 'left', BOX, 'vive_world')['workspace']['left']
    params = lc.driver_parameters('left', 'LHR-L', '', 1.0, fence)
    assert params['workspace.enabled'] is True
    assert params['workspace.frame_id'] == 'vive_world'
    assert params['workspace.z_max'] == 0.9
    assert 'workspace.enabled' not in lc.driver_parameters('left', 'LHR-L', '', 1.0, None)


def test_resolve_devices_respects_the_profile_lighthouse_choice():
    station = st.normalize({'devices': {'right': 'LHR-R', 'origin_lighthouse': 'LHB-APP'}})
    devices, sources = lc.resolve_devices({}, station, ENV, use_origin_lighthouse=True)
    assert devices == {'serial_left': 'LHR-ENV-L', 'serial_right': 'LHR-R', 'reference_lighthouse_serial': 'LHB-APP'}
    assert sources['serial_left'] == st.SOURCE_ENV

    devices, sources = lc.resolve_devices({}, station, ENV, use_origin_lighthouse=False)
    assert devices['reference_lighthouse_serial'] == ''
    assert sources['reference_lighthouse_serial'] == 'not used by this robot'

    devices, _ = lc.resolve_devices({'reference_lighthouse_serial': 'LHB-ARG'}, station, ENV, False)
    assert devices['reference_lighthouse_serial'] == 'LHB-ARG'     # explicit argument still honoured
