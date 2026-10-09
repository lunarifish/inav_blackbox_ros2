# Copyright 2026 lunarifish
# SPDX-License-Identifier: MIT
"""
End-to-end checks on the bags inav_blackbox_to_bag writes.

The assertions here are the ones that would catch a sign, unit or epoch mistake,
so they are deliberately concrete: the demo INAV log starts at a known UTC
instant, its IMU sits still on the ground at the beginning, and a multi-rotor
must show roughly +9.81 m/s^2 of specific force along the body-up axis.
"""

import datetime
import math
import os
import re
import shutil
import subprocess
import tempfile

import pytest

SOURCE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO_DIR = os.path.join(SOURCE_DIR, 'demo_data')
INAV_LOG = os.path.join(DEMO_DIR, 'blackbox_log_2026-10-08_193942.TXT')
BF_LOG = os.path.join(DEMO_DIR, 'blackbox_log_2026-10-08_184828.TXT')
DECODER = os.path.join(SOURCE_DIR, 'bin', 'blackbox_decode')

# "  <index>  <size> bytes  <start datetime>  <firmware>[  [not INAV...]]"
LOG_LINE = re.compile(r'^\s*(\d+)\s+(\d+) bytes\s+(\S+)\s+(.*?)(?:\s+\[not INAV.*\])?$')

_TYPE_CACHE = {}


def message_type(name):
    """'sensor_msgs/msg/Imu' -> the Python message class."""
    if name not in _TYPE_CACHE:
        package, _, kind = name.partition('/msg/')
        module = __import__(package + '.msg', fromlist=[kind])
        _TYPE_CACHE[name] = getattr(module, kind)
    return _TYPE_CACHE[name]


def find_workspace():
    """Walk up from the package until the colcon workspace root turns up."""
    path = SOURCE_DIR
    for _ in range(4):
        path = os.path.dirname(path)
        if any(os.path.isdir(os.path.join(path, name)) for name in ('install', 'build')):
            return path
    return None


def find_executable():
    """Locate the freshly built converter, wherever colcon put it."""
    workspace = find_workspace()
    candidates = [
        os.environ.get('INAV_BLACKBOX_TO_BAG', ''),
        shutil.which('inav_blackbox_to_bag') or '',
    ]
    if workspace is not None:
        candidates += [
            os.path.join(
                workspace, 'install', 'inav_blackbox_ros2', 'lib', 'inav_blackbox_ros2',
                'inav_blackbox_to_bag'),
            os.path.join(workspace, 'build', 'inav_blackbox_ros2', 'inav_blackbox_to_bag'),
        ]
    for candidate in candidates:
        if candidate and os.access(candidate, os.X_OK):
            return candidate
    pytest.fail('inav_blackbox_to_bag was not found; build the package before testing')


EXECUTABLE = find_executable()


def run_converter(args):
    result = subprocess.run(
        [EXECUTABLE, '--decoder', DECODER] + list(args), capture_output=True, text=True)
    return result


def list_logs(path):
    """Parse --list output into [{index, size, start, firmware}]."""
    result = run_converter(['--list', path])
    assert result.returncode == 0, result.stderr
    header = re.search(r':\s*(\d+) logs?', result.stdout)
    assert header is not None, result.stdout
    logs = []
    for line in result.stdout.splitlines():
        match = LOG_LINE.match(line)
        if match is not None:
            logs.append({
                'index': int(match.group(1)),
                'size': int(match.group(2)),
                'start': match.group(3),
                'firmware': match.group(4),
            })
    assert len(logs) == int(header.group(1)), result.stdout
    return logs


def convert(path, output, extra=()):
    result = run_converter(['-o', output] + list(extra) + [path])
    assert result.returncode == 0, result.stderr
    return result.stdout


def read_bag(bag_path, per_topic_limit=4000):
    """Read a bag into (types, {topic: [(stamp_ns, message)]})."""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id='mcap'), ConverterOptions('', ''))
    entries = reader.get_all_topics_and_types()
    types = {entry.name: entry.type for entry in entries}
    messages = {}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        bucket = messages.setdefault(topic, [])
        if len(bucket) < per_topic_limit:
            bucket.append((stamp, deserialize_message(data, message_type(types[topic]))))
    return types, messages


def parse_epoch_ns(text):
    return int(datetime.datetime.fromisoformat(text).timestamp() * 1e9)


def quaternion_body_rate(q_a, q_b, dt):
    """Return the body-frame angular rate that takes q_a to q_b, in rad/s."""
    w1, v1 = q_a.w, (-q_a.x, -q_a.y, -q_a.z)  # conj(q_a)
    w2, v2 = q_b.w, (q_b.x, q_b.y, q_b.z)
    cross = (v1[1] * v2[2] - v1[2] * v2[1],
             v1[2] * v2[0] - v1[0] * v2[2],
             v1[0] * v2[1] - v1[1] * v2[0])
    w = w1 * w2 - sum(a * b for a, b in zip(v1, v2))
    v = tuple(w1 * v2[i] + w2 * v1[i] + cross[i] for i in range(3))
    norm = math.sqrt(sum(component * component for component in v))
    if norm < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(norm, w)
    return tuple(component / norm * angle / dt for component in v)


def correlation(a, b):
    """Pearson correlation, 0 when either side is constant."""
    import numpy

    left = numpy.asarray(a, dtype=float)
    right = numpy.asarray(b, dtype=float)
    if left.std() < 1e-9 or right.std() < 1e-9:
        return 0.0
    return float(numpy.corrcoef(left, right)[0, 1])


def moving_average(values, window):
    """Windowed mean with the ends held, so no edge artefacts leak in."""
    import numpy

    pad = window // 2
    padded = numpy.concatenate([numpy.full(pad, values[0]), values, numpy.full(pad, values[-1])])
    return numpy.convolve(padded, numpy.ones(window) / window, mode='valid')[:len(values)]


@pytest.fixture(scope='module')
def inav():
    """Convert the INAV demo file's largest log once for the whole module."""
    info = list_logs(INAV_LOG)
    assert len(info) == 4, info
    target = max(info, key=lambda entry: entry['size'])
    assert 'INAV' in target['firmware'], target

    directory = tempfile.mkdtemp(prefix='inav_bag_test_')
    bag = os.path.join(directory, 'inv.bag')
    output = convert(
        INAV_LOG, bag,
        ['--index', str(target['index']), '--decimate', '5', '--tf-rate', '10',
         '--plot-rate', '10'])
    types, messages = read_bag(bag)
    return {'info': target, 'bag': bag, 'types': types, 'messages': messages, 'output': output}


def test_list_finds_every_log():
    assert len(list_logs(INAV_LOG)) == 4
    assert any('INAV' in entry['firmware'] for entry in list_logs(INAV_LOG))

    bf_logs = list_logs(BF_LOG)
    assert len(bf_logs) == 34
    assert all('INAV' not in entry['firmware'] for entry in bf_logs)


def test_bag_is_mcap_with_the_expected_topics(inav):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    reader.open(StorageOptions(uri=inav['bag'], storage_id='mcap'), ConverterOptions('', ''))
    assert reader.get_metadata().storage_identifier == 'mcap'

    for topic in ['/tf', '/tf_static', '/inav/imu', '/inav/pose', '/inav/odom', '/inav/status',
                  '/inav/battery', '/inav/gps/fix', '/inav/path/nav', '/inav/markers',
                  '/inav/baro']:
        assert topic in inav['types'], 'missing ' + topic
    assert any(name.startswith('/inav/plot/') for name in inav['types']), sorted(inav['types'])


def test_timestamps_are_the_logs_absolute_utc_time(inav):
    expected = parse_epoch_ns(inav['info']['start'])
    stamps = [stamp for bucket in inav['messages'].values() for stamp, _ in bucket]
    assert stamps, 'the bag holds no messages'
    # Row 0 sits at time 0 of the log, so the earliest message here is the log's
    # own start instant; anything else means the epoch is wrong.
    assert abs(min(stamps) - expected) < 50_000_000, (min(stamps), expected)


def test_imu_is_the_expected_body_frame(inav):
    imu = inav['messages']['/inav/imu']
    assert imu, 'no IMU messages'
    magnitudes = []
    for _, message in imu:
        acc = message.linear_acceleration
        magnitude = math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)
        # Specific force is only ever near 1 g on average, so this range just
        # rules out unit and axis errors, not manoeuvres.
        assert 2.0 < magnitude < 30.0, (acc.x, acc.y, acc.z)
        magnitudes.append(magnitude)
        quaternion_norm = math.sqrt(
            message.orientation.x ** 2 + message.orientation.y ** 2 +
            message.orientation.z ** 2 + message.orientation.w ** 2)
        assert abs(quaternion_norm - 1.0) < 1e-6, quaternion_norm

    # Body up is the gravity axis of a multi-rotor, and the average magnitude
    # over a whole flight is 1 g because the manoeuvres cancel out.
    mean_magnitude = sum(magnitudes) / len(magnitudes)
    assert abs(mean_magnitude - 9.81) < 1.0, mean_magnitude
    mean_up = sum(message.linear_acceleration.z for _, message in imu) / len(imu)
    assert mean_up > 7.0, mean_up


def test_gyro_is_the_derivative_of_the_attitude(inav):
    """
    Check that each gyro axis matches the body rate the quaternions imply.

    That single check covers the axis signs, the rad/s conversion and the
    attitude convention at once, which is why it is worth the quaternion maths.
    """
    imu = inav['messages']['/inav/imu']
    assert len(imu) > 100, len(imu)
    implied = []
    reported = []
    for (stamp_a, a), (stamp_b, b) in zip(imu, imu[1:]):
        dt = (stamp_b - stamp_a) * 1e-9
        if not 1e-4 < dt < 0.5:
            continue
        implied.append(quaternion_body_rate(a.orientation, b.orientation, dt))
        gyro = b.angular_velocity
        reported.append((gyro.x, gyro.y, gyro.z))
    assert len(implied) > 100, len(implied)

    for axis, name in enumerate('xyz'):
        coefficient = correlation(
            [rate[axis] for rate in implied], [rate[axis] for rate in reported])
        assert coefficient > 0.4, (name, coefficient)

    # A deg/s confusion or a 0.1 LSB error would show up as a scale mismatch.
    total_implied = sum(abs(rate[axis]) for rate in implied for axis in range(3))
    total_reported = sum(abs(rate[axis]) for rate in reported for axis in range(3))
    assert 0.4 < total_implied / total_reported < 2.5, total_implied / total_reported


def test_status_carries_decoded_flags(inav):
    status = inav['messages']['/inav/status']
    assert status, 'no status messages'
    assert any(message.armed for _, message in status), 'the log never reports ARM'
    for _, message in status:
        if message.armed:
            assert 'ARM' in message.flight_mode_names
            break
    assert all(message.failsafe_phase for _, message in status)
    # navEPH is centimetres in the log and stays centimetres in the message.
    assert max(message.nav_eph for _, message in status) > 1.0


def test_paths_are_in_the_map_frame(inav):
    nav_path = inav['messages']['/inav/path/nav'][0][1]
    assert nav_path.header.frame_id == 'map'
    assert len(nav_path.poses) > 50, len(nav_path.poses)
    assert all(pose.header.frame_id == 'map' for pose in nav_path.poses[:10])

    gps_buckets = inav['messages'].get('/inav/path/gps')
    if not gps_buckets:
        pytest.skip('this log never reached a GPS fix, so it has no GPS track')
    gps_path = gps_buckets[0][1]
    assert len(gps_path.poses) > 10, len(gps_path.poses)
    # The GPS track is anchored into the nav frame, so the two must describe the
    # same area instead of sitting in unrelated origins.
    nav_last = nav_path.poses[-1].pose.position
    gps_last = gps_path.poses[-1].pose.position
    assert math.hypot(nav_last.x - gps_last.x, nav_last.y - gps_last.y) < 50.0


def test_nav_altitude_rises_with_the_barometer(inav):
    """The vertical axis must climb upwards: navPos[2] is up, not down."""
    import numpy

    baro = inav['messages'].get('/inav/plot/baroAlt_m')
    assert baro, 'the fixture should carry a barometric altitude curve'
    poses = inav['messages']['/inav/path/nav'][0][1].poses
    pose_stamps = numpy.array(
        [pose.header.stamp.sec * 1000000000 + pose.header.stamp.nanosec for pose in poses],
        dtype=numpy.int64)
    pose_z = numpy.array([pose.pose.position.z for pose in poses])

    stamps = numpy.array([stamp for stamp, _ in baro], dtype=numpy.int64)
    baro_m = numpy.array([message.data for _, message in baro])
    nearest = numpy.clip(numpy.searchsorted(pose_stamps, stamps), 0, len(poses) - 1)
    nav_z = pose_z[nearest]

    # Compare five second trends, not samples: the path carries the integrated
    # accelerometer's noise, which the barometer does not, and at that scale the
    # two series agree far better than their raw samples do.
    span = max(3, int(round(5.0 / (numpy.median(numpy.diff(stamps)) * 1e-9))))
    smoothed = (moving_average(baro_m, span), moving_average(nav_z, span))
    climbed = smoothed[0][span:] - smoothed[0][:-span]
    risen = smoothed[1][span:] - smoothed[1][:-span]
    moved = numpy.abs(climbed) > 1.0
    if moved.sum() < 5:  # pragma: no cover - the demo log does climb
        pytest.skip('this log never changes altitude by a metre')
    agrees = float(numpy.mean(numpy.sign(climbed[moved]) == numpy.sign(risen[moved])))
    assert agrees > 0.7, (
        f'only {agrees:.0%} of the climbs raise the path, so navPos[2] is read inverted')


def test_betaflight_log_converts_without_nav_or_attitude():
    info = min(list_logs(BF_LOG), key=lambda entry: entry['size'])
    directory = tempfile.mkdtemp(prefix='bf_bag_test_')
    bag = os.path.join(directory, 'bf.bag')
    output = convert(BF_LOG, bag, ['--index', str(info['index']), '--decimate', '20'])
    assert 'not an INAV log' in output

    types, messages = read_bag(bag)
    assert types, 'the Betaflight bag has no topics at all'
    # No attitude and no navigation estimator in a Betaflight log, so those
    # topic groups must be absent rather than filled with invented zeros.
    for topic in ['/inav/pose', '/inav/odom', '/inav/path/nav', '/inav/gps/fix']:
        assert topic not in types, 'unexpected ' + topic
    # The vendor-neutral ones are still there: the gyro, and the mode flags that
    # Betaflight also logs (as a bitmask, so the names stay numeric).
    assert '/inav/imu' in types, sorted(types)
    assert '/inav/status' in types, sorted(types)
    assert any(name.startswith('/inav/plot/') for name in types), sorted(types)
    assert messages, 'the Betaflight bag holds no messages'
