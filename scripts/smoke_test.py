#!/usr/bin/python3
"""Run a finite headless integration check and stop only its own launch processes."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=150.0,
                        help='Maximum wall time in seconds (default: 150)')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    log_dir = workspace / 'log' / 'smoke_test'
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / 'simulation.log'
    environment = os.environ.copy()
    environment.setdefault('ROS_DOMAIN_ID', '73')
    environment.setdefault('GAZEBO_MASTER_URI', 'http://127.0.0.1:11375')
    environment['ROS_LOG_DIR'] = str(log_dir / 'ros')
    print(f'Headless Gazebo check; log: {log_file}', flush=True)
    success = False
    process = None
    try:
        with log_file.open('w') as output:
            process = subprocess.Popen(
                ['ros2', 'launch', 'so_arm_gazebo', 'simulation.launch.py', 'gui:=false'],
                stdout=output, stderr=subprocess.STDOUT,
                env=environment, start_new_session=True,
            )
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                text = log_file.read_text(errors='replace')
                if ('PICK AND PLACE SUCCEEDED:' in text and
                        'Pick and place demo completed.' in text):
                    success = True
                    break
                if 'PICK AND PLACE FAILED:' in text or process.poll() is not None:
                    break
                time.sleep(0.25)
    finally:
        if process is not None:
            # This new process group contains only this check's ROS/Gazebo children.
            for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5),
                                 (signal.SIGKILL, 5)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    continue
    if success:
        print('PASS: Gazebo verified cube lift, placement, and rest state.')
        return 0
    print('FAIL: headless simulation did not complete successfully.', file=sys.stderr)
    print('\n'.join(log_file.read_text(errors='replace').splitlines()[-45:]), file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())
