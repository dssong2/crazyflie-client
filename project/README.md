# Multi-objetive Drone Delivery Workspace

Contents:
- milestone1/ - Milestone 1 flight script, path_optimization.ipynb, and flight data
- flight_milestone2.py - Milestone 2 flight script
- milestone2_analysis.ipynb - Milestone 2 analysis (hover and path flights: 10 s / 5 cm check, path tracking, saturation)
- controller_design.ipynb - LQR controller design; exports gains_box_<configuration>*.json
- flight_task5_hover.py - Task 5 hover test with the box (custom controller + mocap observer)
- flight_task5_path.py - Task 5 flight: takeoff, 15 s hold (the 10 s / 5 cm check), then the delivery path (optimal or naive order)
- ../project-MOI/ - mass and moment of inertia measurements for each box configuration (Task 3)

Task 5 empty box: m = 0.0390 kg and J from project-MOI/runs/box_v1, gains in gains_box_empty_ffz.json.
The firmware's k_F is about 14% low, so eq_f_z is set to the measured hover thrust (`thrust_correction`
in controller_design.ipynb). Set `thrust_correction = 1.0` after k_F is corrected and the firmware is reflashed.
