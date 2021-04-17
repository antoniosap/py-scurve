import asyncio
import numpy as np
import logging
from matplotlib import pyplot as plt
from pyscurve import ScurvePlanner, plot_trajectory


class SCurveAxisMotion(ScurvePlanner):
    ACCELERATION_ID = 0
    SPEED_ID = 1
    POSITION_ID = 2

    def __init__(self, debug=False):
        super().__init__(debug)
        self.cam_pan_range = (32, 120)
        self.cam_tilt_range = (70, 185)
        self.q0 = [32.]
        self.q1 = [120.]
        self.v0 = [1.]
        self.v1 = [5.]
        self.v_max = 20.
        self.a_max = 15.
        self.j_max = 100.
        self.r_profiles = None

    def do_plan_trajectory(self, dt):
        traj = self.plan_trajectory(self.q0, self.q1, self.v0, self.v1, self.v_max, self.a_max, self.j_max)
        dof = traj.dof
        timesteps = int(max(traj.time) / dt)
        time = np.linspace(0, max(traj.time), timesteps)

        # NOW
        # profiles[t]           --- profiles for each DOF at time x[t]
        # profiles[t][d]        --- profile for d DOF at time x[t]
        # profiles[t][d][k]     --- accel/vel/pos profile for d DOF at time x[t]
        p_list = [traj(t) for t in time]
        profiles = np.asarray(p_list)

        # NEED
        # profiles[d]       --- profiles for each DOF 0 <= d <= DOF number
        # profiles[d][k]    --- accel/vel/pos profile for DOF d where j
        # profiles[d][k][t] --- accel/vel/pos at time x[k] for DOF i
        # profiles = np.reshape(profiles, (dof, 3, timesteps))
        self.r_profiles = np.zeros((dof, 3, timesteps))
        for d in range(dof):
            for p in range(3):
                self.r_profiles[d, p, :] = profiles[:, d, p]

        print(timesteps)
        print(time)
        print(max(traj.time))

    def do_exec_trajectory(self):
        print(self.r_profiles[0][self.POSITION_ID])


if __name__ == "__main__":
    p = SCurveAxisMotion(debug=True)
    p.do_plan_trajectory(dt=0.3)
    p.do_exec_trajectory()