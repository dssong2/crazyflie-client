# Multi-objetive Drone Delivery Workspace

Contents:
- milestone1/ - Milestone 1 flight script, path_optimization.ipynb, and flight data
- flight_milestone2.py - Milestone 2 flight script
- milestone2_analysis.ipynb - Milestone 2 analysis of the loaded flight (10 s / 5 cm check, path tracking, saturation,
  battery), plus a Task 5 results table with one flight per box configuration (`TASK5_FLIGHTS`)
- controller_design.ipynb - LQR controller design. Set `configuration` ('empty', 'partial', 'full') and Run All to
  write gains_box_<configuration>.json
- flight_task5_hover.py - Task 5 hover with the box (custom controller + mocap observer), with a soft landing
- flight_task5_path.py - Task 5 flight: takeoff, 15 s hold (the 10 s / 5 cm check), then the delivery path (optimal or
  naive order)
- task5_empty_review.ipynb - review of every empty-box flight and open questions, for the TA
- ../project-MOI/ - mass and moment of inertia measurements (Task 3)

## Flying Task 5

1. Check the battery: `uv run ae483-battery`. Flights that started at 4.05 V or more went well, and below about
   3.95 V the box flights ran out of motor headroom.
2. In the flight script, set `configuration`, then run it from this folder: `uv run python flight_task5_hover.py`.
   `run = None` picks one past the highest existing run for that configuration, so recorded flights are never overwritten.
   Keep failed flights: Task 5 asks for all flight data to be logged.
3. Set `filename` in milestone2_analysis.ipynb and Run All. Put the flight that counts in `TASK5_FLIGHTS`.

Secure the payload cubes so they can't slide: a loose cube shifts the CG during flight, which pins one motor at 65535
(partial run 2).

## Box configurations

| configuration | mass | gains |
|---|---|---|
| empty | 39.0 g (drone + box v1) | gains_box_empty.json |
| partial (1 cube) | 39.3 g | gains_box_partial.json |
| full (3 cubes) | 39.9 g | gains_box_full.json |

A cube is about 0.29 g (12 weigh 3.5 g). J is the box v1 measurement for all three (project-MOI/runs/box_v1). The cubes
add under 1 g, so all three use the empty-box feedback gains (R is sized from the empty-box mass), and only
eq_f_z = m g changes.

Hover takes about 85% of full motor command with the empty box. By the thrust model, the drone cannot hover above
about 46 g.

## Firmware

controller_c_code/controller_ae483.c is a copy of the file in crazyflie-firmware. The gains above need this firmware:
- k_F and k_M are scaled up by 0.3826 / 0.3286 from the Lab 4 values. In flight, the drone hovered at a mean f_z of
  0.3286 N in Lab 4 units, not m g = 0.3826 N. The model hover command (55,015 for 39 g) now matches flight.
- Velocity feed-forward: the controller uses v - v_des. The flight scripts set `drone_client.velocity_feedforward = True`
  so that `move_smooth` sends the desired velocity. Without it, the drone lags a moving setpoint by about 30 cm at 0.3 m/s.
  Plain position setpoints have v_des = 0, so older scripts fly the same as before.
- J_x, J_y for the torque estimate are the box v1 values.

The flight scripts refuse to fly these gains if the drone doesn't have this firmware. gains_box_empty_ffz.json is only for
the OLD firmware (no-reflash workaround: eq_f_z scaled by 0.859).
