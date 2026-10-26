"""Client code that you write."""

import time
import numpy as np
from ae483.clients import CrazyflieClient


class MyCrazyflieClient(CrazyflieClient):
    def move_smooth(self, p_inW_1, p_inW_2, yaw, v):
        print(f'Move smoothly from {p_inW_1} to {p_inW_2} at {v} meters / second')

        p_inW_1 = np.array(p_inW_1)
        p_inW_2 = np.array(p_inW_2)

        d = np.linalg.norm(p_inW_2 - p_inW_1)
        dt = d / v
        start_time = time.time()
        while True:
            t = time.time()
            s = np.clip((t-start_time) / dt, 0, 1 )
            p_inW_des = (1 - s) * p_inW_1 + s * p_inW_2

            self.cf.commander.send_position_setpoint(
                p_inW_des[0], p_inW_des[1], p_inW_des[2], yaw
            )

            if s >= 1:
                return
            else:
                time.sleep(0.1)