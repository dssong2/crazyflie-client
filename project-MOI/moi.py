"""
Moment-of-inertia estimation for the drone + payload (same pendulum method as Lab 3).

Usage (from the repo root):

    uv run python project-MOI/moi.py project-MOI/runs/box_v1

Everything about a configuration (masses, CG locations, rig pivot locations,
uncertainties, trimming) lives in <run_dir>/config.json. This script reads it,
analyzes x_data.json / y_data.json / z_data.json, saves a sanity-check plot per
axis, and writes <run_dir>/results.json.

Geometry convention (all in the drone's body frame, meters):
  * origin = CG of the drone ALONE (no box, no payload)
  * each part in "parts" has a mass and a CG position
  * each rig axis has a "pivot": any point on the pivot (hinge) axis
The combined CG is computed from the parts, and r for each rig is the distance
from the pivot axis to the combined CG (axis component removed). So if you
change the box or payload, you only edit masses / CG positions - r updates.

Pendulum model:  J = m g r T^2 / (4 pi^2) - m r^2   (J about the combined CG)

Uncertainty is the same first-order worst-case bound as Lab 3,
    delta_J = sum_i |dJ/dq_i| * delta_q_i,
taken over every scalar input (each mass, each CG coordinate, each pivot
coordinate, and T). Derivatives are computed numerically, so nothing needs to
be re-derived when the geometry changes.

Can also be used from a notebook:

    from moi import analyze
    results = analyze('runs/box_v1', show=True)
"""

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib

AXES = ['x', 'y', 'z']
AXIS_VECTORS = {'x': np.array([1., 0., 0.]), 'y': np.array([0., 1., 0.]), 'z': np.array([0., 0., 1.])}


###################################
# CONFIG

def _vec3(value, name):
    """Accept a scalar (same for all three coordinates) or a length-3 list."""
    a = np.array(value, dtype=float)
    if a.ndim == 0:
        a = np.full(3, float(a))
    if a.shape != (3,):
        raise ValueError(f'{name} must be a number or a list of 3 numbers, got {value}')
    return a


def load_config(run_dir):
    run_dir = Path(run_dir)
    with open(run_dir / 'config.json') as f:
        cfg = json.load(f)

    # Drone alone sits at the origin by definition
    parts = [{
        'name': 'drone',
        'mass': float(cfg['drone']['mass']),
        'delta_mass': float(cfg['drone'].get('delta_mass', 0.)),
        'cg': np.zeros(3),
        'delta_cg': np.zeros(3),
    }]
    for p in cfg.get('parts', []):
        parts.append({
            'name': p['name'],
            'mass': float(p['mass']),
            'delta_mass': float(p.get('delta_mass', 0.)),
            'cg': _vec3(p['cg'], f"{p['name']}.cg"),
            'delta_cg': _vec3(p.get('delta_cg', 0.), f"{p['name']}.delta_cg"),
        })

    axes = {}
    for axis, a in cfg['axes'].items():
        if axis not in AXES:
            raise ValueError(f'unknown axis "{axis}" in config (use x, y, z)')
        axes[axis] = {
            'data': run_dir / a.get('data', f'{axis}_data.json'),
            'pivot': _vec3(a['pivot'], f'axes.{axis}.pivot'),
            'delta_pivot': _vec3(a.get('delta_pivot', 0.), f'axes.{axis}.delta_pivot'),
            't_min_offset': float(a.get('t_min_offset', 0.)),
            't_max_offset': float(a.get('t_max_offset', 0.)),
            'prominence': a.get('prominence', 0.3),
        }

    return {
        'run_dir': run_dir,
        'description': cfg.get('description', run_dir.name),
        'g': float(cfg.get('g', 9.81)),
        'parts': parts,
        'axes': axes,
    }


###################################
# PHYSICS

def combined_mass_and_cg(masses, cgs):
    m = np.sum(masses)
    c = (masses[:, None] * cgs).sum(axis=0) / m
    return m, c


def pivot_distance(c, pivot, axis):
    """Distance from the pivot axis (through `pivot`, along `axis`) to point c."""
    u = AXIS_VECTORS[axis]
    v = c - pivot
    return np.linalg.norm(v - np.dot(v, u) * u)


def J_from(params, axis, g):
    """J about the combined CG, from a flat parameter vector (see _pack)."""
    masses, cgs, pivot, T = _unpack(params)
    m, c = combined_mass_and_cg(masses, cgs)
    r = pivot_distance(c, pivot, axis)
    return m * g * r * T**2 / (4 * np.pi**2) - m * r**2


def _pack(masses, cgs, pivot, T):
    return np.concatenate([masses, cgs.ravel(), pivot, [T]])


def _unpack(params):
    n = (len(params) - 4) // 4
    masses = params[:n]
    cgs = params[n:4 * n].reshape(n, 3)
    pivot = params[4 * n:4 * n + 3]
    T = params[-1]
    return masses, cgs, pivot, T


def worst_case_delta(f, x, dx, rel_step=1e-6):
    """sum_i |df/dx_i| * dx_i with central differences (skips inputs with dx_i = 0)."""
    total = 0.
    for i in np.flatnonzero(dx):
        h = rel_step * max(abs(x[i]), 1e-3)
        xp, xm = x.copy(), x.copy()
        xp[i] += h
        xm[i] -= h
        total += abs((f(xp) - f(xm)) / (2 * h)) * dx[i]
    return total


###################################
# DATA

def load_gyro(filename, t_min_offset=0., t_max_offset=0.):
    # Imported here so the physics above can be used without the ae483 package
    from ae483.tools import load_hardware_data, resample_data_drone

    raw_drone, _, _ = load_hardware_data(filename)
    data = resample_data_drone(raw_drone, t_min_offset=t_min_offset, t_max_offset=t_max_offset)
    t = data['time']
    w = {a: np.deg2rad(data[f'gyro.{a}']) for a in AXES}
    return t, w


def estimate_period(t, w, prominence=0.3):
    """
    Period from mean peak-to-peak time. `prominence` is a fraction of the
    signal's peak amplitude (so it works regardless of how hard you swung it).
    """
    from scipy.signal import find_peaks

    amp = np.max(np.abs(w))
    i_peaks, _ = find_peaks(w, prominence=prominence * amp)
    if len(i_peaks) < 3:
        raise RuntimeError(
            f'only found {len(i_peaks)} peaks - check trimming / lower "prominence" in config.json'
        )
    t_peaks = t[i_peaks]
    t_diff = np.diff(t_peaks)
    T = np.mean(t_diff)
    # Worst-case deviation of any single period from the mean, in EITHER direction
    delta_T = np.max(np.abs(t_diff - T))
    return T, delta_T, i_peaks, t_diff


###################################
# ANALYSIS

def analyze_axis(cfg, axis, show=False):
    import matplotlib.pyplot as plt

    a = cfg['axes'][axis]
    g = cfg['g']
    parts = cfg['parts']

    t, w = load_gyro(a['data'], a['t_min_offset'], a['t_max_offset'])
    if all(np.all(v == 0.0) for v in w.values()):
        raise RuntimeError(
            f"{a['data']} is all zeros - the drone's sensors never calibrated during that recording. "
            'Reboot the drone while it is held still, then re-record this axis with --force.'
        )
    w_axis = w[axis]
    T, delta_T, i_peaks, t_diff = estimate_period(t, w_axis, a['prominence'])

    # How clean was the swing? Off-axis RMS as a fraction of on-axis RMS.
    rms = {k: np.sqrt(np.mean(v**2)) for k, v in w.items()}
    off_axis = max(rms[k] for k in AXES if k != axis) / rms[axis]

    masses = np.array([p['mass'] for p in parts])
    cgs = np.array([p['cg'] for p in parts])
    x = _pack(masses, cgs, a['pivot'], T)
    dx = _pack(
        np.array([p['delta_mass'] for p in parts]),
        np.array([p['delta_cg'] for p in parts]),
        a['delta_pivot'],
        delta_T,
    )

    m, c = combined_mass_and_cg(masses, cgs)
    r = pivot_distance(c, a['pivot'], axis)
    J = J_from(x, axis, g)
    delta_J = worst_case_delta(lambda p: J_from(p, axis, g), x, dx)
    delta_r = worst_case_delta(
        lambda p: pivot_distance(combined_mass_and_cg(*_unpack(p)[:2])[1], _unpack(p)[2], axis), x, dx
    )

    # Breakdown: how much of delta_J comes from each kind of input
    n = len(parts)
    groups = {'mass': slice(0, n), 'cg': slice(n, 4 * n), 'pivot': slice(4 * n, 4 * n + 3), 'T': slice(-1, None)}
    breakdown = {}
    for name, s in groups.items():
        dx_g = np.zeros_like(dx)
        dx_g[s] = dx[s]
        breakdown[name] = worst_case_delta(lambda p: J_from(p, axis, g), x, dx_g)

    # Sanity-check plot (same three plots as the Lab 3 notebook, stacked)
    fig, axs = plt.subplots(3, 1, figsize=(9, 9), tight_layout=True)
    for k in AXES:
        axs[0].plot(t, w[k], label=f'$w_{k}$', lw=1.5 if k == axis else 1, alpha=1 if k == axis else 0.6)
    axs[0].set_title(f'{cfg["description"]} - {axis} rig (off-axis/on-axis RMS = {off_axis:.2f})')
    axs[0].set_ylabel('rad / s')
    axs[0].legend()
    axs[1].plot(t, w_axis, label=f'$w_{axis}$')
    axs[1].plot(t[i_peaks], w_axis[i_peaks], '.', ms=12, label=f'{len(i_peaks)} peaks')
    axs[1].set_ylabel('rad / s')
    axs[1].legend()
    axs[2].plot(t[i_peaks][:-1], t_diff, '.', ms=12, label='peak time differences')
    axs[2].axhline(T, color='C1', label=f'mean T = {T:.4f} s')
    axs[2].axhspan(T - delta_T, T + delta_T, color='C1', alpha=0.15, label=f'$\\pm\\Delta T$ = {delta_T:.4f} s')
    axs[2].set_xlabel('time (s)')
    axs[2].set_ylabel('period (s)')
    axs[2].legend()
    for ax in axs:
        ax.grid()
    plot_file = cfg['run_dir'] / f'{axis}_check.png'
    fig.savefig(plot_file, dpi=110)
    if show:
        plt.show()
    else:
        plt.close(fig)

    warnings = []
    if off_axis > 0.25:
        warnings.append(f'off-axis rotation is {off_axis:.0%} of on-axis - swing may not be clean')
    if delta_T / T > 0.05:
        warnings.append(f'period scatter is {delta_T / T:.1%} of T - check peaks / trimming')
    if J <= 0:
        warnings.append('J <= 0 - r or T is wrong (check pivot location / units)')

    return {
        'J': J, 'delta_J': delta_J,
        'T': T, 'delta_T': delta_T, 'n_peaks': int(len(i_peaks)),
        'r': r, 'delta_r': delta_r,
        'off_axis_ratio': off_axis,
        'delta_J_breakdown': breakdown,
        'plot': plot_file.name,
        'warnings': warnings,
    }


def analyze(run_dir, show=False):
    if not show:
        matplotlib.use('Agg')
    cfg = load_config(run_dir)

    masses = np.array([p['mass'] for p in cfg['parts']])
    cgs = np.array([p['cg'] for p in cfg['parts']])
    m, c = combined_mass_and_cg(masses, cgs)
    delta_m = sum(p['delta_mass'] for p in cfg['parts'])

    results = {
        'description': cfg['description'],
        'm': m, 'delta_m': delta_m,
        'cg_offset_from_drone_cg': c.tolist(),
        'axes': {},
    }
    for axis in AXES:
        if axis not in cfg['axes']:
            continue
        if not cfg['axes'][axis]['data'].exists():
            print(f"Skipping {axis}: {cfg['axes'][axis]['data']} not recorded yet")
            continue
        results['axes'][axis] = analyze_axis(cfg, axis, show=show)

    with open(cfg['run_dir'] / 'results.json', 'w') as f:
        json.dump(results, f, indent=2, default=float)

    print_summary(results)
    return results


def print_summary(res):
    print(f"\n=== {res['description']} ===")
    print(f"m   = {res['m']:.4f} +/- {res['delta_m']:.4f} kg")
    cx, cy, cz = (1e3 * np.array(res['cg_offset_from_drone_cg'])).tolist()
    print(f"CG shift from drone-alone CG = ({cx:.2f}, {cy:.2f}, {cz:.2f}) mm")
    print()
    print(f"{'axis':>4} {'T (s)':>16} {'r (cm)':>16} {'J (kg m^2)':>22}   largest dJ source")
    for axis, a in res['axes'].items():
        worst = max(a['delta_J_breakdown'], key=a['delta_J_breakdown'].get)
        print(f"{axis:>4} {a['T']:7.4f} +/- {a['delta_T']:.4f}"
              f" {1e2 * a['r']:6.3f} +/- {1e2 * a['delta_r']:.3f}"
              f" {a['J']:.3e} +/- {a['delta_J']:.2e}   {worst}")
    for axis, a in res['axes'].items():
        for w in a['warnings']:
            print(f'  WARNING ({axis}): {w}')
    if res['axes']:
        print('\nPlots: ' + ', '.join(a['plot'] for a in res['axes'].values()) + '  |  results.json written')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Estimate J_x, J_y, J_z from pendulum swing data.')
    parser.add_argument('run_dir', help='folder with config.json and x/y/z_data.json')
    parser.add_argument('--show', action='store_true', help='also open the plots in a window')
    args = parser.parse_args()
    analyze(args.run_dir, show=args.show)
