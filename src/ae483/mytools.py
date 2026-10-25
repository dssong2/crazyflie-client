"""Functions that you write.

Add your own code here, not in tools.py. Keeping them separate means you can
`git pull` our updates all semester without conflicts.
"""

from time import time

import numpy as np
from scipy.spatial.transform import Rotation

# Makes everything in tools.py available here, just as if your code were at the
# bottom of that file.
from ae483.myclients import MyCrazyflieClient
from ae483.tools import *


def transform_data_mocap(raw_data):
    # Copy raw data
    data = {}
    for key, val in raw_data.items():
        data[key] = val.copy()

    # ZYX (yaw-pitch-roll) Euler angles <-> rotation matrix
    def R_from_ypr(psi, theta, phi):
        Rz = np.array([[np.cos(psi), -np.sin(psi), 0.],
                       [np.sin(psi),  np.cos(psi), 0.],
                       [0.,           0.,          1.]])
        Ry = np.array([[ np.cos(theta), 0., np.sin(theta)],
                       [ 0.,            1., 0.],
                       [-np.sin(theta), 0., np.cos(theta)]])
        Rx = np.array([[1., 0.,           0.],
                       [0., np.cos(phi), -np.sin(phi)],
                       [0., np.sin(phi),  np.cos(phi)]])
        return Rz @ Ry @ Rx

    def ypr_from_R(R):
        theta = np.arcsin(np.clip(-R[2, 0], -1., 1.))
        phi = np.arctan2(R[2, 1], R[2, 2])
        psi = np.arctan2(R[1, 0], R[0, 0])
        return psi, theta, phi

    # Define parameters
    d_1 = 0.016
    d_2 = 0.009
    
    # Pose of drone body frame in active marker frame
    R_inA_ofB = np.eye(3)
    p_inA_ofB = np.array([0., 0., -d_1])

    ####################################
    # START OF ANALYSIS AT TIME STEP 0
    #
    
    # Pose of drone world frame in active marker frame
    R_inA_ofW = np.eye(3)
    p_inA_ofW = np.array([0., 0., -(d_1 + d_2)])

    # Get measurements of (x, y, z) and (psi, theta, phi) from mocap
    x, y, z = data['x'][0], data['y'][0], data['z'][0]
    psi, theta, phi = data['yaw'][0], data['pitch'][0], data['roll'][0]

    # Pose of active marker frame in mocap world frame
    R_inQ_ofA = R_from_ypr(psi, theta, phi)
    p_inQ_ofA = np.array([x, y, z])
    
    # Pose of drone world frame in mocap world frame
    R_inQ_ofW = R_inQ_ofA @ R_inA_ofW
    p_inQ_ofW = p_inQ_ofA + R_inQ_ofA @ p_inA_ofW
    
    # Pose of mocap world frame in drone world frame
    R_inW_ofQ = R_inQ_ofW.T
    p_inW_ofQ = -R_inQ_ofW.T @ p_inQ_ofW

    #
    # END OF ANALYSIS AT TIME STEP 0
    ####################################

    for i in range(len(data['time'])):

        ####################################
        # START OF ANALYSIS AT TIME STEP i
        #

        # Get measurements of (x, y, z) and (psi, theta, phi) from mocap
        x, y, z = data['x'][i], data['y'][i], data['z'][i]
        psi, theta, phi = data['yaw'][i], data['pitch'][i], data['roll'][i]

        # Pose of active marker deck in mocap world frame
        R_inQ_ofA = R_from_ypr(psi, theta, phi)
        p_inQ_ofA = np.array([x, y, z])

        # Pose of drone body frame in mocap world frame
        R_inQ_ofB = R_inQ_ofA @ R_inA_ofB
        p_inQ_ofB = p_inQ_ofA + R_inQ_ofA @ p_inA_ofB

        # Pose of drone body frame in drone world frame
        R_inW_ofB = R_inW_ofQ @ R_inQ_ofB
        p_inW_ofB = p_inW_ofQ + R_inW_ofQ @ p_inQ_ofB

        # Replace measurements of (x, y, z) and (phi, theta, psi) from mocap
        data['x'][i], data['y'][i], data['z'][i] = p_inW_ofB
        data['yaw'][i], data['pitch'][i], data['roll'][i] = ypr_from_R(R_inW_ofB)

        #
        # END OF ANALYSIS AT TIME STEP i
        ####################################
    
    # Return the result
    return data


def precise_time_shift(raw_data_mocap, t, z_drone, t_shift_guess,
                       do_transform=True, t_min_offset=0.,
                       t_shift_rng=1., t_shift_res=0.05):
    """
    Find a value of t_shift that precisely aligns mocap data with
    drone data by sampling candidate values in the interval

        [t_shift_guess - t_shift_rng, t_shift_guess + t_shift_rng]
    
    at the resolution t_shift_res and returning the value that
    produces the minimum RMSE between z_drone and z_mocap.
    """

    # Create an array of time shifts over which to search
    t_shifts = np.linspace(
        t_shift_guess - t_shift_rng,
        t_shift_guess + t_shift_rng,
        int(1 + np.ceil((2 * t_shift_rng) / t_shift_res)),
    )

    # Print a warning if computation time is likely to be long
    if len(t_shifts) > 50:
        print(f'========================================================================')
        print(f'WARNING (precise_time_shift)\n')
        print(f'Searching the interval\n')
        print(f'  t_shift_bnds=[{min(t_shifts):4.2f}, {max(t_shifts):4.2f}]\n')
        print(f'with the resolution\n')
        print(f'  t_shift_res={t_shift_res:4.2f}\n')
        print(f'means computing RMSE for {len(t_shifts)} possible time shifts, which is likely')
        print(f'to be slow. You may want to decrease the size of the search interval')
        print(f'or increase the resolution.')
        print(f'========================================================================\n')

    # Create an array to hold the RMSE for each time shift
    RMSEs = np.empty_like(t_shifts)

    # Find the RMSE for each time shift
    for i, t_shift in enumerate(t_shifts):
        # Resample mocap data with time shift
        resampled_data_mocap = resample_data_mocap(
            raw_data_mocap,
            t,
            t_shift=t_shift,
            t_min_offset=t_min_offset,
        )

        # Transform mocap data
        if do_transform:
            transformed_data_mocap = transform_data_mocap(resampled_data_mocap)
        else:
            transformed_data_mocap = resampled_data_mocap

        # Get z estimate from mocap data
        z_mocap = transformed_data_mocap['z']

        # Find RMSE between z_mocap and z_drone
        RMSEs[i] = np.sqrt(np.mean((z_mocap - z_drone)**2))

    # Find the index of the minimum RMSE
    i_min = np.argmin(RMSEs)

    # Find the minimum RMSE
    RMSE_min = RMSEs[i_min]

    # Find the time shift that gives the minimum RMSE
    t_shift_min = t_shifts[i_min]
    
    # Return the result
    return t_shift_min


def sync_data_mocap(raw_data_mocap, t, z_drone,
                    do_transform=True, t_min_offset=0.,
                    t_shift_rng=1., t_shift_res=0.05):

    # Get rough guess at time shift
    t_shift_guess = rough_time_shift(
        raw_data_mocap,
        t,
        z_drone,
        t_min_offset=t_min_offset,
    )

    # Refine the rough guess to get the time shift that
    # minimizes RMSE between z_drone and z_mocap
    t_shift_min = precise_time_shift(
        raw_data_mocap,
        t,
        z_drone,
        t_shift_guess=t_shift_guess,
        do_transform=do_transform,
        t_min_offset=t_min_offset,
        t_shift_rng=t_shift_rng,
        t_shift_res=t_shift_res,
    )

    # Resample mocap data with this time shift
    resampled_data_mocap = resample_data_mocap(
        raw_data_mocap,
        t,
        t_shift=t_shift_min,
        t_min_offset=t_min_offset,
    )
    
    # Transform mocap data
    if do_transform:
        transformed_data_mocap = transform_data_mocap(resampled_data_mocap)
    else:
        transformed_data_mocap = resampled_data_mocap

    # Return the result
    return transformed_data_mocap


################################################
# Project Functions
################################################
def sample_locations(n: int, x_range: tuple, y_range: tuple, min_distance: float, start_location: np.ndarray, seed: int = None) -> np.ndarray:
    """Sample random locations within the specified x and y ranges.

    Args:
        n (int): Number of random locations to sample.
        x_range (tuple): A tuple specifying the range of x values (min_x, max_x).
        y_range (tuple): A tuple specifying the range of y values (min_y, max_y).
        min_distance (float): Minimum distance between sampled locations.
        start_location (np.ndarray): A tuple specifying the starting location (x0, y0).
        seed (int, optional): Random seed for reproducibility. Defaults to None.

    Returns:
        np.ndarray: An array of shape (n, 2) containing the sampled locations.
    """
    if seed is not None:
        np.random.seed(seed)
    locations = []
    while len(locations) < n:
        # Sample a random location within the specified ranges
        x = np.random.uniform(x_range[0], x_range[1])
        y = np.random.uniform(y_range[0], y_range[1])
        new_location = np.array([x, y])
        
        # Check if the new location is at least min_distance away from existing locations and the start location
        if all(np.linalg.norm(new_location - loc) >= min_distance for loc in locations) and np.linalg.norm(new_location - start_location) >= min_distance:
            locations.append(new_location)
    
    return np.array(locations)

def optimize_path(start_location: np.ndarray, locations: np.ndarray) -> np.ndarray:
    """Optimize the path through the given locations. Always starts and ends at the start location. TSP problem solved using the Held-Karp dynamic programming formulation (Held & Karp, 1962, Sec. 1.2).

    Args:   
        start_location (np.ndarray): The starting location.
        locations (np.ndarray): An array of shape (n, 2) containing the locations to visit.

    Returns:
        np.ndarray: An array of shape (n+2, 2) containing the locations in the optimized order, beginning and ending at the start location.
    """
    # City 1 (index 0) is the start; cities 2..n (indices 1..n-1) are the locations
    P = np.vstack([np.asarray(start_location, float), np.asarray(locations, float).reshape(-1, 2)])
    n = len(P)
    if n == 1:
        return P[[0, 0]]
    a = np.linalg.norm(P[:, None] - P[None], axis=2) # cost matrix a_ij
    A = a[1:, 1:] # costs among cities 2..n

    # S is a subset of {2, ..., n} as a bitmask; bit b corresponds to city index b+1
    m = n - 1
    N = 1 << m
    C = np.full((N, m), np.inf) # C[S, l]: min cost from city 1 through all of S, ending at l

    # ---- Phase 1: compute C(S, l) recursively ----
    # (5a) n(S) = 1: C({l}, l) = a_1l
    for l in range(m):
        C[1 << l, l] = a[0, l + 1]

    masks = np.arange(N)
    size = np.zeros(N, dtype=int) # n(S)
    for b in range(m):
        size += (masks >> b) & 1

    # (5b) n(S) > 1: C(S, l) = min_{m in S-l} [C(S-l, m) + a_ml], in increasing n(S)
    for s in range(2, m + 1):
        S_s = masks[size == s]
        for l in range(m):
            S = S_s[((S_s >> l) & 1) == 1]
            C[S, l] = np.min(C[S ^ (1 << l)] + A[:, l], axis=1) # inf where m not in S-l

    # (6) optimum cost: min_l [C({2..n}, l) + a_l1]
    full = N - 1
    opt = np.min(C[full] + a[1:, 0])

    # ---- Phase 2: recover optimum permutation (i_n first, then i_{n-1}, ..., i_2) ----
    # (7) i_n satisfies opt = C({2..n}, i_n) + a_{i_n 1}
    l = int(np.flatnonzero(np.isclose(C[full] + a[1:, 0], opt))[0])
    tour, S = [l], full
    # (8) i_p satisfies C(S, i_{p+1}) = C(S - i_{p+1}, i_p) + a_{i_p i_{p+1}}
    for _ in range(m - 1):
        S_prev = S ^ (1 << l)
        l = int(np.flatnonzero(np.isclose(C[S_prev] + A[:, l], C[S, l]))[0])
        S = S_prev
        tour.append(l)

    order = [0] + [i + 1 for i in reversed(tour)] + [0]
    return P[order]

def plot_path(locations: np.ndarray, start_location: np.ndarray, path: np.ndarray, x_range: tuple, y_range: tuple):
    """Plot the optimized path through the given locations.

    Args:
        locations (np.ndarray): An array of shape (n, 2) containing the locations to visit.
        start_location (np.ndarray): The starting location.
        path (np.ndarray): The optimized path.
        x_range (tuple): The range of x-coordinates to display.
        y_range (tuple): The range of y-coordinates to display.
    """
    plt.figure()
    plt.plot(*locations.T, 'o', label='Locations')
    plt.plot(*start_location, 's', label='Start')
    plt.quiver(
        path[:-1, 0], path[:-1, 1],
        np.diff(path[:, 0]), np.diff(path[:, 1]),
        angles='xy', scale_units='xy', scale=1,
        color='purple', width=0.005
    )
    plt.xlim(x_range)
    plt.ylim(y_range)
    plt.legend(loc='upper left', bbox_to_anchor=(1.02, 1))
    plt.title(f'Optimized path through {len(locations)} locations')
    plt.grid()
    plt.show()
    
def fly_path_milestone1(drone_client: MyCrazyflieClient, path: np.ndarray, t_hover: float, t_dropoff: float):
    """Fly the drone through the given path by calling drone_client.move()

    Args:
        drone_client (MyCrazyflieClient): The drone client object.
        path (np.ndarray): An array of shape (n, 2) containing the locations to visit in optimal order.
        t_hover (float): The time to hover at each location (in seconds).
        t_dropoff (float): The time to hover at delivery height at each location (in seconds).
    """
    # TODO: implement this function as required by Task 3 of Milestone #1
    # Drone sequence:
        # Take off to a safe height (e.g., 0.5 m)
        # for each location in the path:
            # Fly to the location at safe height (e.g., 0.5 m)
            # Hover at the location for t_hover seconds
            # Descend to delivery height (e.g., 0.2 m)
            # Hover at delivery height for t_dropoff seconds
            # Ascend back to safe height (e.g., 0.5 m)
        # Return to the starting location at safe height (e.g., 0.5 m)
        # Land at the starting location