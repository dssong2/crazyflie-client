###################################
# IMPORTS

import json
import os
from ae483.clients import QualisysClient
from ae483.myclients import MyCrazyflieClient
import time
import numpy as np
from ae483.mytools import sample_locations, optimal_path, naive_path, fly_path_milestone1


###################################
# PARAMETERS

# -- PROBABLY THE SAME FOR EVERY FLIGHT --

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/59/2M/E7E7E7E7E7'

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_50'

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [51, 52, 53, 54]

# -- MAY CHANGE FROM FLIGHT TO FLIGHT --

# Which target set to fly (a key of target_sets below) and in which order:
# 'optimal' (Held-Karp) or 'naive' (greedy nearest-neighbor)
target_set = 2
order = 'optimal'

# Specify whether or not to use the motion capture system
use_mocap = True

# Specify whether or not to use a custom controller
use_controller = True

# Specify whether or not to use a custom observer
use_observer = False

# Flight data file, named from the target set and order. The script refuses to
# overwrite an existing file, so a re-fly needs a new name (e.g. add '_refly1').
data_filename = f'm2t1_set{target_set}_{order}.json'

# Specify the name of the file from which to read control gains, or None if
# this flight does not use a custom controller. This file is written by your
# design notebook.
gains_filename = 'gains.json' if use_controller else None


# Specify the variables you want to log at 100 Hz from the drone
variables = [
    # Battery voltage (read by the preflight battery check)
    'pm.vbat',
    # WHEN RUNNING DEFAULT CONTROLLER
    # State estimates
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
    # Desired position
    'ctrltarget.x',
    'ctrltarget.y',
    'ctrltarget.z',
    # WHEN RUNNING CUSTOM CONTROLLER
    # State
    'ae483log.p_x',
    'ae483log.p_y',
    'ae483log.p_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    # Desired position
    'ae483log.p_x_des',
    'ae483log.p_y_des',
    'ae483log.p_z_des',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
    # Mocap as received by the drone (custom observer)
    'ae483log.num_mocap',
    'ae483log.mocap_age',
    'ae483log.p_x_mocap',
    'ae483log.p_y_mocap',
    'ae483log.p_z_mocap',
    'ae483log.psi_mocap',
    'ae483log.theta_mocap',
    'ae483log.phi_mocap',
]

###################################
# PATH CODE

# Target sets for Milestone 2 Task 1: {set number: (number of targets, seed)}.
# Set 1 is the Milestone 1 set as a placeholder; fill in sets 1-5 once they are
# chosen.
target_sets = {
    1: (5, 11), # 5 targets, seed 11
    2: (5, 12), # 5 targets, seed 12
}

x_range = (-0.5, 0.5)
y_range = (-0.5, 0.5)
min_distance = 0.2
start_location = [0., 0.]
t_hover = 1.5
t_dropoff = 2.0

n, seed = target_sets[target_set]
locations = sample_locations(n, x_range, y_range, min_distance, start_location, seed)
if order == 'optimal':
    path, _ = optimal_path(start_location, locations)
elif order == 'naive':
    path, _ = naive_path(start_location, locations)
else:
    raise ValueError(f"order must be 'optimal' or 'naive', not {order!r}")

print(f'Target set {target_set} (n={n}, seed={seed}), {order} order')
print(f'Locations: {locations}')
print(f'Path: {path}')

if os.path.exists(data_filename):
    raise FileExistsError(f'{data_filename} already exists - change data_filename so you do not overwrite a recorded flight')


###################################
# FLIGHT CODE

# Read control gains, if this flight uses a custom controller. The date is
# printed so that you notice if you are about to fly with gains from an
# earlier design that you forgot to export.
gains = None
gains_created = None
if gains_filename is not None:
    with open(gains_filename, 'r') as f:
        gains_file = json.load(f)
    gains = gains_file['gains']
    gains_created = gains_file['created']
    print(f'Using {len(gains)} gains from {gains_filename}, written {gains_created}')

# Create and start the client that will connect to the drone
drone_client = MyCrazyflieClient(
    uri,
    use_controller=use_controller,
    use_observer=use_observer,
    marker_deck_ids=marker_deck_ids if use_mocap else None,
    variables=variables,
)

# Create this now so that it always exists, even if we never get far enough to
# connect to the motion capture system
mocap_client = None


# Everything from here until "finally" is what happens during your flight. If
# anything goes wrong - including if you press Ctrl-C - the code in the
# "finally" block still runs, stopping the motors, disarming the drone, and
# disconnecting.
try:
    # Wait until the client is fully connected to the drone and until the state
    # estimate has had time to converge
    drone_client.wait_until_ready()
    
        # Read the battery at rest, before arming or sending flight commands.
    battery_deadline = time.monotonic() + 5.0
    while not drone_client.data.get('pm.vbat', {}).get('data'):
        if time.monotonic() >= battery_deadline:
            raise RuntimeError('No battery voltage received before flight.')
        time.sleep(0.05)
    battery_voltage = drone_client.data['pm.vbat']['data'][-1]
    # Interpolate the announcement's charge curve. Clamp to its supplied
    # 0–90% range rather than inventing a voltage for 100% charge.
    battery_percentage = np.interp(
        battery_voltage,
        [3.00, 3.78, 3.83, 3.87, 3.89, 3.92, 3.96, 4.00, 4.04, 4.10],
        [0, 10, 20, 30, 40, 50, 60, 70, 80, 90],
    )
    print(
        f'Preflight battery: {battery_voltage:.2f} V, '
        f'{battery_percentage:.1f}%'
    )

    # Send the control gains to the drone and confirm they arrived. This does
    # nothing if gains is None. It happens before the motion capture system is
    # started so that as little time as possible passes between the start of
    # motion capture data and the start of flight - the smaller that gap, the
    # easier it is to align the two sets of data afterward.
    drone_client.set_gains(gains)

    # Create and start the client that will connect to the motion capture system
    # With the custom (mocap) observer, each mocap pose is also forwarded to the
    # drone, where ae483UpdateWithPose receives it
    if use_mocap:
        mocap_client = QualisysClient([{
            'name': marker_deck_name,
            'callback': drone_client.send_pose if use_observer else None,
        }])

    # Arm the drone. Brushless drones will not spin their motors until they are
    # armed. Brushed drones do not need to be armed, but arming them does no
    # harm, so the same flight code works for both.
    # drone_client.arm()

    # Pause before takeoff
    drone_client.stop(3.0)

    # Fly the path in the chosen order
    fly_path_milestone1(drone_client, path, t_hover=t_hover, t_dropoff=t_dropoff)

    # Pause after landing
    drone_client.stop(3.0)

except KeyboardInterrupt:
    print('\nInterrupted - stopping the motors and saving whatever data were collected.')

finally:
    # Stop the motors, disarm, and disconnect from the drone. Each step is
    # guarded so that a failure in one of them cannot prevent the others - and,
    # in particular, cannot prevent your flight data from being saved.
    try:
        drone_client.close()
    except Exception as e:
        print(f'Error while closing the connection to the drone: {e}')

    # Disconnect from the motion capture system
    if mocap_client is not None:
        try:
            mocap_client.close()
        except Exception as e:
            print(f'Error while closing the connection to the motion capture system: {e}')

    # Assemble flight data from both clients. The gains are saved along with the
    # data, so that every flight says for itself which controller flew it. The
    # path settings are saved too, so that every flight says which target set
    # and order it flew.
    data = {}
    data['gains'] = drone_client.gains
    data['gains_created'] = gains_created if drone_client.gains is not None else None
    data['use_controller'] = use_controller
    data['use_observer'] = use_observer
    data['path'] = {
        'target_set': target_set,
        'order': order,
        'n': n,
        'seed': seed,
        'start_location': list(start_location),
        'locations': locations.tolist(),
        'path': path.tolist(),
        't_hover': t_hover,
        't_dropoff': t_dropoff,
    }
    data['drone'] = drone_client.data
    data['mocap'] = mocap_client.data.get(marker_deck_name, {}) if use_mocap and mocap_client is not None else {}
    data['bodies'] = mocap_client.data if use_mocap and mocap_client is not None else {}

    # Write flight data to a file. We write to a temporary file first and then
    # rename it, which is an operation the operating system does all at once.
    # That way, if anything interrupts the writing, you are left with your
    # previous data file rather than with a half-written one that cannot be
    # read at all.
    temporary_filename = data_filename + '.partial'
    with open(temporary_filename, 'w') as outfile:
        json.dump(data, outfile, sort_keys=False)
    os.replace(temporary_filename, data_filename)
    print(f'Wrote flight data to {data_filename}')
