#!/usr/bin/env python3
# Copyright 2026 lunarifish
# SPDX-License-Identifier: MIT
"""
Re-derive the sign and unit conventions that inav_frames.hpp depends on.

INAV documents none of these, so this script recomputes them from a real log and
reports whether they still hold: the recorded IMU axes, the gyro's sign against
the logged attitude, the attitude LSB, and the navigation position and velocity
both being North/East/Up. Run it after a firmware change, or against a new
airframe, before trusting a bag built from a new log.

Some checks need the log to actually excite the axis in question (a log that
never rolls cannot say anything about the roll axis). Those are reported as
"skip" rather than passing silently. Call it as, for example,
`python3 tools/check_conventions.py demo_data/blackbox_log_2026-10-08_193942.TXT -i 3`.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import tempfile

import numpy

UNIT_SUFFIX = re.compile(r'\s*\([^)]*\)$')
DECODE_UNITS = [
    '--merge-gps',
    '--unit-frame-time', 'us',
    '--unit-rotation', 'deg/s',
    '--unit-acceleration', 'm/s2',
    '--unit-height', 'cm',
    '--unit-vbat', 'V',
    '--unit-gps-speed', 'm/s',
    '--unit-amperage', 'A',
    '--unit-flags', 'flags',
]


def decode_to_csv(log, index, decoder, destination):
    """
    Decode one log to a CSV on disk.

    Never call the decoder without --stdout: it writes .csv/.event/.gpx files
    next to the input instead of streaming.
    """
    command = [decoder, '--index', str(index), '--stdout'] + DECODE_UNITS + [log]
    with open(destination, 'w') as handle:
        result = subprocess.run(command, stdout=handle, stderr=subprocess.PIPE, text=True)
    if result.returncode != 0:
        raise SystemExit('blackbox_decode failed: ' + result.stderr.strip())
    return destination


def as_float(field):
    """Numeric value of a cell, or NaN for the ones holding flag names."""
    try:
        return float(field)
    except ValueError:
        return float('nan')


def read_columns(path):
    """Return {column name: numpy array} for one decoded CSV."""
    with open(path) as handle:
        header = [UNIT_SUFFIX.sub('', name) for name in handle.readline().strip().split(',')]
        count = len(header)
        columns = [[] for _ in header]
        for line in handle:
            fields = line.strip().split(',')
            if len(fields) != count:
                continue
            for index, field in enumerate(fields):
                columns[index].append(as_float(field))
    return {name: numpy.array(values) for name, values in zip(header, columns)}


def correlation(a, b):
    """Pearson correlation of two series, ignoring non-finite samples."""
    mask = numpy.isfinite(a) & numpy.isfinite(b)
    if mask.sum() < 10 or a[mask].std() < 1e-12 or b[mask].std() < 1e-12:
        return float('nan')
    return float(numpy.corrcoef(a[mask], b[mask])[0, 1])


def moving_average(series, window):
    """Windowed mean with the ends held, so no edge artefacts leak in."""
    pad = window // 2
    padded = numpy.concatenate([numpy.full(pad, series[0]), series, numpy.full(pad, series[-1])])
    return numpy.convolve(padded, numpy.ones(window) / window, mode='valid')[:len(series)]


class Checker:
    """Collect the convention results and print one line per check."""

    def __init__(self):
        """Start with nothing checked."""
        self.failures = 0
        self.skips = 0

    def check(self, name, value, expectation, description):
        """Record and print one measured value against its expectation."""
        ok = bool(expectation(value))
        self.failures += 0 if ok else 1
        verdict = 'ok  ' if ok else 'FAIL'
        print(f'  [{verdict}] {name:<44} {value:+.3f}   ({description})')

    def skip(self, name, description):
        """Record and print a check the log cannot answer."""
        self.skips += 1
        print(f'  [skip] {name:<44}         ({description})')

    def report(self):
        """Print the tally and return the process exit code."""
        note = f' ({self.skips} skipped for lack of excitation)' if self.skips else ''
        if self.failures == 0:
            print(f'\nall checked conventions hold{note}')
        else:
            print(f'\n{self.failures} convention(s) no longer hold{note}')
        return 1 if self.failures else 0


def main(argv=None):
    """Decode a log, check every convention, and return the exit code."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('log', nargs='?', help='blackbox file to decode and check')
    parser.add_argument('-i', '--index', type=int, default=1,
                        help='log inside the file (default 1)')
    parser.add_argument('--csv', help='check an already decoded CSV instead of decoding')
    parser.add_argument('--decoder', default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'bin', 'blackbox_decode'))
    args = parser.parse_args(argv)

    if not args.csv and not args.log:
        parser.error('give a blackbox file, or --csv with one already decoded')
    if args.csv:
        csv_path = args.csv
    else:
        csv_path = decode_to_csv(
            args.log, args.index, args.decoder,
            os.path.join(tempfile.mkdtemp(prefix='inav_conventions_'), 'decoded.csv'))

    columns = read_columns(csv_path)
    required = ['time', 'attitude[0]', 'attitude[1]', 'attitude[2]',
                'gyroADC[0]', 'gyroADC[1]', 'gyroADC[2]',
                'accSmooth[0]', 'accSmooth[1]', 'accSmooth[2]']
    missing = [name for name in required if name not in columns]
    if missing:
        raise SystemExit('this log has no INAV attitude/IMU columns; cannot check: '
                         + ', '.join(missing))

    time_seconds = columns['time'] * 1e-6
    duration = time_seconds[-1] - time_seconds[0]
    if duration <= 0:
        raise SystemExit('the log holds less than one sample')

    roll = columns['attitude[0]'] * 0.1 * math.pi / 180.0
    pitch = columns['attitude[1]'] * 0.1 * math.pi / 180.0
    heading = columns['attitude[2]'] * 0.1
    attitude = {'roll': roll, 'pitch': pitch, 'heading': heading}

    print(f'{os.path.basename(csv_path)}: {len(time_seconds)} rows, {duration:.1f} s')
    checker = Checker()

    print('\nattitude encoding')
    checker.check('attitude LSB is 0.1 deg', float(numpy.abs(columns['attitude[2]']).max()),
                  lambda value: value > 360.0,
                  'a heading in degrees would stay below 360')

    print('\nrecorded IMU axes are body x forward, y left, z up (FLU)')
    checker.check('mean accSmooth[2]', float(numpy.nanmean(columns['accSmooth[2]'])),
                  lambda value: value > 7.0, 'up axis carries +1 g')
    for axis, name, sense in ((1, 'roll', 1.0), (0, 'pitch', -1.0)):
        # The tilt shows up in the accelerometer as g*sin(angle); below about
        # 6 deg of motion that is under 1 m/s^2, less than the vibration the
        # same column carries, and the correlation is pure noise. A log that
        # only hovers says nothing here rather than saying something wrong.
        excitation = float(numpy.nanstd(numpy.sin(attitude[name])))
        if excitation < 0.1:
            checker.skip(f'corr(accSmooth[{axis}], sin({name}))',
                         f'only {excitation * 57.3:.1f} deg rms of {name} motion')
            continue
        measured = correlation(columns[f'accSmooth[{axis}]'], numpy.sin(attitude[name]))
        checker.check(f'corr(accSmooth[{axis}], sin({name}))', measured,
                      (lambda value, s=sense: value * s > 0.3),
                      'a rise to the left/back shows up in the side/back axis')

    print('\ngyro signs against the logged attitude')
    for axis, name in enumerate(('roll', 'pitch', 'heading')):
        rate = numpy.gradient(attitude[name], time_seconds)
        measured = correlation(columns[f'gyroADC[{axis}]'], rate)
        if name == 'heading':
            # Heading grows clockwise while ROS yaw grows counter-clockwise, so
            # these two must disagree.
            checker.check(f'corr(gyroADC[{axis}], d{name}/dt)', measured,
                          lambda value: value < -0.2, 'opposite sign: yaw_ros = 90 - heading')
        else:
            checker.check(f'corr(gyroADC[{axis}], d{name}/dt)', measured,
                          lambda value: value > 0.2, f'same sign as {name}')

    if all(f'navVel[{axis}]' in columns for axis in range(3)):
        print('\nnavVel is North, East, UP (cm/s)')
        if 'GPS_velned[0]' in columns:
            for axis, name, sense in ((0, 'north', 1.0), (1, 'east', 1.0), (2, 'down', -1.0)):
                gps_axis = columns[f'GPS_velned[{axis}]']
                if numpy.nanstd(gps_axis) < 1.0:
                    checker.skip(f'corr(navVel[{axis}], GPS_velned[{axis}])',
                                 'the craft never moved')
                    continue
                measured = correlation(columns[f'navVel[{axis}]'], columns[f'GPS_velned[{axis}]'])
                checker.check(f'corr(navVel[{axis}], GPS_velned[{axis}])', measured,
                              (lambda value, s=sense: value * s > 0.4),
                              f'navVel[{axis}] is {"up" if axis == 2 else name}')
        else:
            checker.skip('navVel vs GPS_velned', 'this log has no GPS velocity')

    if all(f'navPos[{axis}]' in columns for axis in range(3)):
        print('\nnavPos shares that convention: North, East, UP (cm)')
        # d(navPos)/dt must run the same way as navVel on *every* axis; the
        # horizontal axes are the control, since north is north either way.
        window = max(10, min(len(time_seconds) // 2, 500))
        for axis in range(3):
            position = moving_average(columns[f'navPos[{axis}]'], window)
            rate = numpy.gradient(position, time_seconds)
            if numpy.nanstd(columns[f'navVel[{axis}]']) < 1.0:
                checker.skip(f'corr(d(navPos[{axis}])/dt, navVel[{axis}])',
                             'nothing moved on this axis')
                continue
            measured = correlation(rate, columns[f'navVel[{axis}]'])
            checker.check(f'corr(d(navPos[{axis}])/dt, navVel[{axis}])', measured,
                          lambda value: value > 0.2, 'position and velocity agree in sign')

        if 'BaroAlt' in columns and numpy.nanstd(columns['BaroAlt']) > 100.0:
            # Only compare over stretches with a real altitude change; a hovering
            # log's baro noise says nothing about the vertical sign.
            span = max(10, int(5.0 * len(time_seconds) / duration))
            baro = moving_average(columns['BaroAlt'], span)
            position = moving_average(columns['navPos[2]'], span)
            delta_baro = baro[span:] - baro[:-span]
            delta_position = position[span:] - position[:-span]
            moved = numpy.abs(delta_baro) > 100.0
            if moved.sum() < 5:
                checker.skip('navPos[2] tracks BaroAlt', 'the log has no >1 m altitude change')
            else:
                agreement = float(numpy.mean(
                    numpy.sign(delta_baro[moved]) == numpy.sign(delta_position[moved])))
                checker.check('navPos[2] tracks BaroAlt (share of >1 m moves)',
                              agreement * 100.0, lambda value: value > 70.0,
                              'both grow together, so neither is inverted')
        else:
            checker.skip('navPos[2] tracks BaroAlt', 'no usable barometric altitude')

    return checker.report()


if __name__ == '__main__':
    sys.exit(main())
