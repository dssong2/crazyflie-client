"""
Record gyro data while the drone swings on the pendulum rig (motors never spin).

Usage (from the repo root):

    uv run python project-MOI/record.py project-MOI/runs/box_v2 x
    uv run python project-MOI/record.py project-MOI/runs/box_v2 y --duration 20

Writes <run_dir>/<axis>_data.json. Start the swing once it prints
"Recording...". Ctrl-C ends early and still saves what was collected.
"""

import argparse
import json
import os
from pathlib import Path

from ae483.myclients import MyCrazyflieClient

URI = 'radio://0/15/2M/E7E7E7E7E7'

VARIABLES = [
    'stateEstimate.roll',
    'stateEstimate.pitch',
    'stateEstimate.yaw',
    'gyro.x',
    'gyro.y',
    'gyro.z',
]

parser = argparse.ArgumentParser(description='Record a pendulum swing for one axis.')
parser.add_argument('run_dir', help='folder for this configuration (created if missing)')
parser.add_argument('axis', choices=['x', 'y', 'z'])
parser.add_argument('--duration', type=float, default=15., help='seconds to record (default 15)')
parser.add_argument('--uri', default=URI)
parser.add_argument('--force', action='store_true', help='overwrite an existing data file')
args = parser.parse_args()

run_dir = Path(args.run_dir)
run_dir.mkdir(parents=True, exist_ok=True)
data_filename = run_dir / f'{args.axis}_data.json'
if data_filename.exists() and not args.force:
    raise SystemExit(f'{data_filename} already exists - pass --force to overwrite')

drone_client = MyCrazyflieClient(args.uri, variables=VARIABLES)
save = True
try:
    drone_client.wait_until_ready()

    # A drone whose sensors never calibrated (moved while booting) reports
    # exactly 0.0 for everything. Real gyro noise is never exactly zero.
    recent = [v for k in ('gyro.x', 'gyro.y', 'gyro.z') for v in drone_client.data[k]['data'][-50:]]
    if not recent or all(v == 0.0 for v in recent):
        save = False
        raise SystemExit(
            '\nDrone is sending all zeros - its sensors did not calibrate.\n'
            'Fix: reboot it while it is held completely still (flat on the table, or held\n'
            'motionless on the rig), wait a few seconds, then run this again.\n'
            f'  uv run ae483-reboot "{args.uri}"\n'
        )

    print(f'\nRecording {args.axis} swing for {args.duration:.0f} s - give it a small push now.\n')
    drone_client.stop(args.duration)
except KeyboardInterrupt:
    print('\nInterrupted - saving whatever data were collected.')
finally:
    try:
        drone_client.close()
    except Exception as e:
        print(f'Error while closing the connection to the drone: {e}')

    if save:
        data = {
            'gains': None,
            'gains_created': None,
            'use_controller': False,
            'drone': drone_client.data,
            'mocap': {},
            'bodies': {},
        }
        tmp = str(data_filename) + '.partial'
        with open(tmp, 'w') as f:
            json.dump(data, f)
        os.replace(tmp, data_filename)
        print(f'Wrote {data_filename}')
    else:
        print(f'Nothing saved - {data_filename} left unchanged.')
