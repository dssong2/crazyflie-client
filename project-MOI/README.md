# Moment of inertia (drone + box + payload)

Same pendulum method as Lab 3. Each box/payload configuration gets its own folder under `runs/`.

| run | configuration | m (kg) | J_x (kg m²) | J_y (kg m²) | J_z (kg m²) |
|---|---|---|---|---|---|
| `runs/box_v1` | box v1, empty | 0.0390 ± 0.0010 | 1.85e-5 ± 0.45e-5 | 1.90e-5 ± 0.64e-5 | 3.14e-5 ± 0.63e-5 |

## Measure a new configuration

1. **Record** the swings from the repo root, with the drone hanging on each rig. Boot the drone
   while it is held still, or its sensors will not calibrate and it will send all zeros.
   `record.py` checks for this and refuses to record.
   ```bash
   uv run python project-MOI/record.py project-MOI/runs/<name> x   # push it once it says "Recording"
   uv run python project-MOI/record.py project-MOI/runs/<name> y
   uv run python project-MOI/record.py project-MOI/runs/<name> z
   ```
   Add `--force` to redo an axis, or `--duration 20` to record longer.
2. **Analyze**: open `moi_analysis.ipynb`, set `RUN = '<name>'`, and click Run All. The first
   run creates `runs/<name>/config.json` as a copy of the box v1 config. Edit the masses and
   positions in it, then click Run All again.
3. **Check the plots**: every peak should have a dot and nothing else should. If a peak or spike
   from your push at the start gets counted, set that axis's `t_min_offset` in `config.json` to
   just after it.
4. Paste the printed `m` and `J_*` into `project/controller_design.ipynb`.

`moi.py` also works without the notebook: `uv run python project-MOI/moi.py project-MOI/runs/<name>`.
Each run folder gets `results.json` and `x/y/z_check.png`.

## config.json

All values are in the body frame, in meters, with the origin at the **CG of the drone alone**
(+x forward, +y left, +z up).

- `drone`: mass of the bare drone, plus its uncertainty.
- `parts`: one entry per thing you attach (box, payload, ...). Each entry has `mass`, `cg`,
  `delta_mass`, and `delta_cg` (`delta_cg` is a number or `[dx, dy, dz]`).
- `axes.{x,y,z}.pivot`: any point on that rig's hinge axis. `r` comes from the combined CG,
  so moving a payload updates `r` automatically.
- `t_min_offset` / `t_max_offset`: seconds to trim from the start / end of a recording.
- `prominence`: the peak threshold, as a fraction of the largest |w|.

Uncertainty is the Lab 3 first-order worst-case bound, Σ|∂J/∂q|Δq, taken over every input. The
derivatives are numerical, so nothing needs to be re-derived when the geometry changes. ΔT is
the largest deviation of any one period from the mean (in either direction).
