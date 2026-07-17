#!/usr/bin/env python3
"""Calibrate hall-latch travel distance from GPIO edge counts."""

import argparse
import select
import sys
import termios
import time
import tty

import lgpio


GLITCH_FILTER_US = 100
EDGES_PER_SPUR_REVOLUTION = 8

edge_count = 0
edge_timestamps = []


def _edge_callback(chip, gpio, level, tick):
    global edge_count
    edge_count += 1
    edge_timestamps.append(time.monotonic())


def _reset_run():
    global edge_count
    edge_count = 0
    edge_timestamps.clear()


def _snapshot():
    count = edge_count
    timestamps = edge_timestamps[:]
    return count, timestamps


def _print_summary(end_time):
    count, timestamps = _snapshot()
    duration = end_time - timestamps[0] if timestamps else 0.0
    mean_rate = count / duration if duration > 0.0 else 0.0

    print(f'\nTotal edges: {count}')
    print(f'Duration: {duration:.6f} s')
    print(f'Mean edge rate: {mean_rate:.3f} Hz')

    while True:
        try:
            distance = float(input('Measured distance (m): '))
            if distance <= 0.0:
                raise ValueError
            if count == 0:
                print('Cannot calibrate: no edges were counted.')
                return
            break
        except ValueError:
            print('Enter a positive distance in metres.')

    metres_per_edge = distance / count
    print(f'Metres per edge: {metres_per_edge:.9f}')
    print(
        'Metres per spur revolution: '
        f'{metres_per_edge * EDGES_PER_SPUR_REVOLUTION:.9f}'
    )


def _parse_args():
    parser = argparse.ArgumentParser(
        description='Count hall-latch edges over a measured travel distance.'
    )
    parser.add_argument('gpio', type=int, help='BCM GPIO number')
    return parser.parse_args()


def _main():
    args = _parse_args()
    chip = lgpio.gpiochip_open(0)
    callback = None
    terminal_settings = None
    end_time = time.monotonic()

    try:
        lgpio.gpio_claim_alert(chip, args.gpio, lgpio.BOTH_EDGES)
        lgpio.gpio_set_debounce_micros(chip, args.gpio, GLITCH_FILTER_US)
        callback = lgpio.callback(
            chip, args.gpio, lgpio.BOTH_EDGES, _edge_callback
        )

        if sys.stdin.isatty():
            terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        print(
            f'GPIO {args.gpio}, both edges, '
            f'{GLITCH_FILTER_US} us glitch filter'
        )
        print(
            'Press any key to zero the counter and start a run. '
            'Ctrl-C to finish.'
        )

        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                sys.stdin.read(1)
                _reset_run()

            count, timestamps = _snapshot()
            if timestamps:
                elapsed = time.monotonic() - timestamps[0]
                rate = (
                    1.0 / (timestamps[-1] - timestamps[-2])
                    if len(timestamps) > 1
                    and timestamps[-1] > timestamps[-2]
                    else 0.0
                )
            else:
                elapsed = 0.0
                rate = 0.0
            print(
                f'\rEdges: {count:8d}  Elapsed: {elapsed:9.3f} s  '
                f'Instantaneous: {rate:9.1f} Hz',
                end='',
                flush=True,
            )
    except KeyboardInterrupt:
        end_time = time.monotonic()
    finally:
        if terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        if callback is not None:
            callback.cancel()
        lgpio.gpiochip_close(chip)

    _print_summary(end_time)


if __name__ == '__main__':
    _main()
