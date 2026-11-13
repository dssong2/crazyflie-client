"""Client code that you write."""

import time
import numpy as np
from ae483.clients import CrazyflieClient


class MyCrazyflieClient(CrazyflieClient):
    # When True, move_smooth also sends the desired velocity (full-state
    # setpoints), which the custom controller uses as feed-forward on v - v_des.
    # Without it the velocity feedback pulls toward zero velocity, so the drone
    # lags a moving setpoint by about speed * (k_v / k_p). Firmware without the
    # feed-forward simply ignores the velocity, so this is safe either way.
    velocity_feedforward = False

    def battery_voltage(self, n=50):
        """Mean of the last n logged pm.vbat samples (0.5 s at 100 Hz), or None
        if pm.vbat is not among the logged variables or has no data yet."""
        vbat = self.data.get('pm.vbat', {}).get('data', [])
        if not vbat:
            return None
        recent = vbat[-n:]
        return sum(recent) / len(recent)

    def move_smooth(self, p_inW_1, p_inW_2, yaw, v):
        print(f'Move smoothly from {p_inW_1} to {p_inW_2} at {v} meters / second')

        p_inW_1 = np.array(p_inW_1, dtype=float)
        p_inW_2 = np.array(p_inW_2, dtype=float)

        d = np.linalg.norm(p_inW_2 - p_inW_1)
        if d == 0.:
            # Nothing to move (and d / v would divide by zero below)
            self._send_setpoint(p_inW_2, np.zeros(3), yaw)
            return
        dt = d / v
        v_inW_move = (p_inW_2 - p_inW_1) / dt
        start_time = time.time()
        while True:
            t = time.time()
            s = np.clip((t-start_time) / dt, 0, 1 )
            p_inW_des = (1 - s) * p_inW_1 + s * p_inW_2
            v_inW_des = v_inW_move if s < 1 else np.zeros(3)

            self._send_setpoint(p_inW_des, v_inW_des, yaw)

            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def _send_setpoint(self, p_inW_des, v_inW_des, yaw):
        if self.velocity_feedforward:
            # Yaw (degrees) as a quaternion [qx, qy, qz, qw]
            half_yaw = np.radians(yaw) / 2
            orientation = [0., 0., np.sin(half_yaw), np.cos(half_yaw)]
            self.cf.commander.send_full_state_setpoint(
                list(p_inW_des), list(v_inW_des), [0., 0., 0.], orientation, 0., 0., 0.
            )
        else:
            self.cf.commander.send_position_setpoint(
                p_inW_des[0], p_inW_des[1], p_inW_des[2], yaw
            )
