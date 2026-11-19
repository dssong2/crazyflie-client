"""Milestone 2 Task 5: fly the optimized delivery path with the payload box
(empty, partial, or full), using the custom controller and the mocap observer.

Sequence: take off to z_safe (0.5 m) -> hold t_hold above the start (this is
the "within 5 cm for 10 s" check) -> visit every target in path order (hover,
descend to 0.2 m, drop off, climb) -> return to start -> land."""

###################################
# IMPORTS

import json
import os
import re
from ae483.clients import QualisysClient
from ae483.myclients import MyCrazyflieClient
from ae483.mytools import sample_locations, optimal_path, naive_path, fly_path_milestone1


###################################
# PARAMETERS

# -- PROBABLY THE SAME FOR EVERY FLIGHT --

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/15/2M/E7E7E7E7E7'

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_10'

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [11, 12, 13, 14]

# The downward sensors are removed, so this flight needs the custom controller,
# the custom (mocap) observer, and mocap forwarded to the drone
use_mocap = True
use_controller = True
use_observer = True

# -- MAY CHANGE FROM FLIGHT TO FLIGHT --

# Box configuration: 'empty', 'partial', or 'full'
configuration = 'empty'

# Gains file for each configuration, written by controller_design.ipynb.
# 'empty': m = 39 g and J measured with box v1 (project-MOI/runs/box_v1), with
# k_F corrected. These gains need the reflashed firmware (corrected k_F and
# velocity feed-forward) - the script checks for it before flying.
# 'partial' and 'full': written by controller_design.ipynb with that
# configuration's mass (see CONFIGURATIONS there).
gains_files = {
    'empty': 'gains_box_empty.json',
    'partial': 'gains_box_partial.json',
    'full': 'gains_box_full.json',
}

# Run number for this configuration, or None for the next unused one. The
# script never overwrites a recorded flight.
run = None

# How long to hold above the start after takeoff (s). Task 5 needs 10 s within
# 5 cm of the setpoint; 15 s leaves time to settle after takeoff.
t_hold = 15.0

# Delivery path: same target generation as flight_milestone2.py.
# order: 'optimal' (Held-Karp path optimization) or 'naive' (greedy nearest-neighbor)
order = 'optimal'
n_targets, seed = 5, 11
x_range = (-1.0, 1.0)
y_range = (-1.0, 1.0)
min_distance = 0.2
start_location = [0., 0.]
t_hover = 1.5
t_dropoff = 2.0
z_safe = 0.5       # must match fly_path_milestone1

if run is None:
    # One past the highest existing run, so a deleted run's number is never reused
    existing = [int(m.group(1)) for f in os.listdir('.')
                if (m := re.fullmatch(rf'task5_path_{configuration}_run(\d+)\.json', f))]
    run = max(existing, default=0) + 1
data_filename = f'task5_path_{configuration}_run{run}.json'
gains_filename = gains_files[configuration]


# Specify the variables you want to log at 100 Hz from the drone. Only custom
# controller and observer variables: the default observer and ctrltarget values
# are not used here, and leaving them out saves radio bandwidth for mocap.
variables = [
    # State estimates (custom observer)
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
    # Thrust, torque commands, and integral states (headroom and windup)
    'ae483log.f_z',
    'ae483log.tau_x_cmd',
    'ae483log.tau_y_cmd',
    'ae483log.e_x',
    'ae483log.e_y',
    'ae483log.e_z',
    'ae483log.a_z',
    # Battery voltage (thrust per motor command drops as the battery sags)
    'pm.vbat',
    # Motor power commands (saturate at 65535)
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
    # Mocap as received by the drone
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
# CHECKS

if configuration not in gains_files:
    raise ValueError(f"configuration must be one of {list(gains_files)}, not {configuration!r}")

if os.path.exists(data_filename):
    raise FileExistsError(f'{data_filename} already exists - increase run so you do not overwrite a recorded flight')

locations = sample_locations(n_targets, x_range, y_range, min_distance, start_location, seed)
if order == 'optimal':
    path, _ = optimal_path(start_location, locations)
elif order == 'naive':
    path, _ = naive_path(start_location, locations)
else:
    raise ValueError(f"order must be 'optimal' or 'naive', not {order!r}")

print(f'Task 5 path: {configuration} box, run {run}, hold {t_hold:.0f} s at {z_safe} m, then {order} path')
print(f'Path: {path.round(3).tolist()}')


###################################
# FLIGHT CODE

# Read control gains. The date is printed so that you notice if you are about
# to fly with gains from an earlier design that you forgot to export.
with open(gains_filename, 'r') as f:
    gains_file = json.load(f)
gains = gains_file['gains']
gains_created = gains_file['created']
print(f'Using {len(gains)} gains from {gains_filename}, written {gains_created}')
print(f'Hover thrust feed-forward eq_f_z = {gains["eq_f_z"]:.4f} N')

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
vbat_takeoff = None


# Everything from here until "finally" is what happens during your flight. If
# anything goes wrong - including if you press Ctrl-C - the code in the
# "finally" block still runs, stopping the motors, disarming the drone, and
# disconnecting.
try:
    # Wait until the client is fully connected to the drone and until the state
    # estimate has had time to converge
    drone_client.wait_until_ready()
    vbat_takeoff = drone_client.battery_voltage()
    if vbat_takeoff is not None:
        print(f'Battery before takeoff: {vbat_takeoff:.2f} V (full charge is about 4.2 V)')

    # These gains assume the reflashed firmware: on the old firmware (k_F 14%
    # low) eq_f_z = m g would command almost full power just to hover
    if drone_client.cf.log.toc.get_element_by_complete_name('ae483log.v_x_des') is None:
        raise SystemExit('The drone has the old firmware (no ae483log.v_x_des) - reflash it before flying these gains')

    # Send the desired velocity with every move_smooth setpoint (feed-forward)
    drone_client.velocity_feedforward = True

    # Send the control gains to the drone and confirm they arrived
    drone_client.set_gains(gains)

    # Create and start the client that will connect to the motion capture
    # system, forwarding each pose to the drone for the mocap observer. The
    # first pose sets the start frame: origin on the ground below the drone,
    # x along its heading.
    mocap_client = QualisysClient([{
        'name': marker_deck_name,
        'callback': drone_client.send_pose,
    }])

    # Arm the drone. Brushless drones will not spin their motors until they are
    # armed. Brushed drones do not need to be armed, but arming them does no
    # harm, so the same flight code works for both.
    # drone_client.arm()

    # Pause before takeoff (also lets mocap reach the drone; the firmware will
    # not start the motors until it has a mocap start frame)
    drone_client.stop(3.0)

    # Take off, hold above the start, fly the path, land
    fly_path_milestone1(drone_client, path, t_hover=t_hover, t_dropoff=t_dropoff, t_hold_start=t_hold)

    # Pause after landing
    drone_client.stop(3.0)

except KeyboardInterrupt:
    print('\nInterrupted - stopping the motors and saving whatever data were collected.')

finally:
    vbat_end = drone_client.battery_voltage()
    if vbat_end is not None:
        print(f'Battery at the end: {vbat_end:.2f} V')

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

    # Assemble flight data from both clients. The gains and the flight settings
    # are saved along with the data, so that every flight says for itself what
    # it flew.
    data = {}
    data['gains'] = drone_client.gains
    data['gains_created'] = gains_created if drone_client.gains is not None else None
    data['gains_filename'] = gains_filename
    data['use_controller'] = use_controller
    data['use_observer'] = use_observer
    data['task5'] = {
        'configuration': configuration,
        'run': run,
        'flight': 'path',
        'z_hover': z_safe,
        't_hold': t_hold,
        'hold_point': [start_location[0], start_location[1], z_safe],
    }
    data['path'] = {
        'order': order,
        'n': n_targets,
        'seed': seed,
        'start_location': list(start_location),
        'locations': locations.tolist(),
        'path': path.tolist(),
        't_hover': t_hover,
        't_dropoff': t_dropoff,
    }
    data['battery'] = {'vbat_before_takeoff': vbat_takeoff, 'vbat_end': vbat_end}
    data['drone'] = drone_client.data
    data['mocap'] = mocap_client.data.get(marker_deck_name, {}) if mocap_client is not None else {}
    data['bodies'] = mocap_client.data if mocap_client is not None else {}

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
