"""Read a Crazyflie's battery voltage over the radio (read-only: no gains, no motors)."""

import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

DEFAULT_URI = 'radio://0/15/2M/E7E7E7E7E7'

# pm.state values in the Crazyflie firmware
POWER_STATES = {0: 'on battery', 1: 'charging', 2: 'charged', 3: 'low power', 4: 'shutdown'}

# Resting voltage thresholds, from our Task 5 flights with the box: flights that
# started at 4.06-4.08 V went well, partial run 2 at 3.96 V ran out of motor
# headroom, and path run 2 at 3.77 V could not hold altitude.
GOOD = 4.05
MARGINAL = 3.95


def main():
    parser = argparse.ArgumentParser(
        prog='ae483-battery',
        description='Read the battery voltage of a Crazyflie over the radio.',
    )
    parser.add_argument('uri', nargs='?', default=DEFAULT_URI,
                        help=f'uri of the drone (default: {DEFAULT_URI})')
    args = parser.parse_args()

    cflib.crtp.init_drivers()
    logconf = LogConfig(name='battery', period_in_ms=100)
    logconf.add_variable('pm.vbat', 'float')
    logconf.add_variable('pm.state', 'int8_t')

    samples = []
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./__cfcache__')) as scf:
            with SyncLogger(scf, logconf) as logger:
                for _, data, _ in logger:
                    samples.append(data)
                    if len(samples) >= 20:  # 2 seconds
                        break
    except Exception as e:
        raise SystemExit(f'Could not read the drone at {args.uri} ({e}). '
                         'Is it powered on, and is the radio on that channel?')

    vbat = sum(s['pm.vbat'] for s in samples) / len(samples)
    state = POWER_STATES.get(samples[-1]['pm.state'], f"state {samples[-1]['pm.state']}")

    if vbat >= GOOD:
        verdict = 'good to fly'
    elif vbat >= MARGINAL:
        verdict = 'marginal - the box flights ran short of motor headroom around here'
    else:
        verdict = 'charge it first'
    print(f'{vbat:.2f} V ({state}): {verdict}')
    if state in ('charging', 'charged'):
        print('(still on the charger - expect a little lower once unplugged)')


if __name__ == '__main__':
    main()
