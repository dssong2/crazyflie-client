"""Collect the supplied 0.25-meter square demonstration with motion capture."""

###################################
# IMPORTS

import json
import os
from ae483.clients import QualisysClient
from ae483.myclients import MyCrazyflieClient


###################################
# PARAMETERS

uri = 'radio://0/15/2M/E7E7E7E7E7'

marker_deck_name = 'marker_deck_10'
marker_deck_ids = [11, 12, 13, 14]

use_mocap = True
use_controller = True
use_observer = False

# Save the demonstration recording beside this script
data_filename = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'lab007_square2.json5',
)


gains_filename = (
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gains_flight20.json')
    if use_controller else None
)


variables = [
    # State estimates (still from default observer)
    'ae483log.p_x',
    'ae483log.p_y',
    'ae483log.p_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Setpoint
    'ae483log.p_x_des',
    'ae483log.p_y_des',
    'ae483log.p_z_des',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
]
###################################
# FLIGHT CODE

if os.path.exists(data_filename):
    raise FileExistsError(
        f'{data_filename} already exists. Rename the previous recording '
        'before collecting another flight.'
    )

gains = None
gains_created = None

if gains_filename is not None:
    with open(gains_filename, 'r') as f:
        gains_file = json.load(f)

    gains = gains_file['gains']
    gains_created = gains_file['created']

    print(
        f'Using {len(gains)} gains from {gains_filename}, '
        f'written {gains_created}'
    )


drone_client = MyCrazyflieClient(
    uri,
    use_controller=use_controller,
    use_observer=use_observer,
    marker_deck_ids=marker_deck_ids if use_mocap else None,
    variables=variables,
)

mocap_client = None


try:

    drone_client.wait_until_ready()

    drone_client.set_gains(gains)

    # ==================== CHANGE 2 (2026-10-09) - START ====================
    # Wait until the drone confirms gains_are_set = 1 before going on. While
    # it is 0, controller_ae483.c holds the motors at zero but still
    # integrates e_x, e_y, e_z once takeoff is commanded, so they wind up.
    # The cached value only updates when the drone echoes the write back.
    # TO REVERT: delete everything from this START line to the END line.
    if gains is not None:
        import time
        wait_start = time.time()
        while int(float(drone_client.cf.param.get_value('ae483par.gains_are_set'))) != 1:
            if time.time() - wait_start > 5.0:
                raise Exception(
                    'Drone did not confirm gains_are_set = 1 within 5 seconds '
                    '- not flying (motors would stay off while e_x, e_y, e_z wind up)'
                )
            time.sleep(0.1)
        print(f'Drone confirmed gains_are_set = 1 after {time.time() - wait_start:.2f} s')
    # ==================== CHANGE 2 - END ====================================

    if use_mocap:
        mocap_client = QualisysClient([
            {
                'name': marker_deck_name,
                'callback': None
            }
        ])

    drone_client.arm()

    # Graceful takeoff
    drone_client.move(0.0, 0.0, 0.2, 0.0, 1.0)
    drone_client.move_smooth([0., 0., 0.2], [0., 0., 0.5], 0.0, 0.20)
    drone_client.move(0.0, 0.0, 0.5, 0.0, 1.0)

    # Move in a square (with a pause at each corner)
    drone_client.move_smooth([0.0, 0.0, 0.5], [0.5, 0.0, 0.5], 0.0, 0.20)
    drone_client.move(0.5, 0.0, 0.5, 0.0, 1.0)
    drone_client.move_smooth([0.5, 0.0, 0.5], [0.5, 0.5, 0.5], 0.0, 0.20)
    drone_client.move(0.5, 0.5, 0.5, 0.0, 1.0)
    drone_client.move_smooth([0.5, 0.5, 0.5], [0.0, 0.5, 0.5], 0.0, 0.20)
    drone_client.move(0.0, 0.5, 0.5, 0.0, 1.0)
    drone_client.move_smooth([0.0, 0.5, 0.5], [0.0, 0.0, 0.5], 0.0, 0.20)

    # Graceful landing
    drone_client.move(0.0, 0.0, 0.5, 0.0, 1.0)
    drone_client.move_smooth([0., 0., 0.50], [0., 0., 0.20], 0.0, 0.20)
    drone_client.move(0.0, 0.0, 0.20, 0.0, 1.0)
    # Pause after landing
    drone_client.stop(1.0)


except KeyboardInterrupt:

    print(
        '\nInterrupted - stopping the motors and '
        'saving whatever data were collected.'
    )


finally:

    try:
        drone_client.close()
    except Exception as e:
        print(f'Error while closing the connection to the drone: {e}')

    if mocap_client is not None:
        try:
            mocap_client.close()
        except Exception as e:
            print(
                f'Error while closing the connection '
                f'to the motion capture system: {e}'
            )

    data = {}

    data['gains'] = drone_client.gains

    data['gains_created'] = (
        gains_created
        if drone_client.gains is not None
        else None
    )

    data['use_controller'] = use_controller
    data['drone'] = drone_client.data

    data['mocap'] = (
        mocap_client.data.get(marker_deck_name, {})
        if use_mocap and mocap_client is not None
        else {}
    )

    data['bodies'] = (
        mocap_client.data
        if use_mocap and mocap_client is not None
        else {}
    )

    temporary_filename = data_filename + '.partial'

    with open(temporary_filename, 'w') as outfile:
        json.dump(
            data,
            outfile,
            sort_keys=False
        )

    os.replace(
        temporary_filename,
        data_filename
    )

    print(
        f'Wrote flight data to {data_filename}'
    )
    
