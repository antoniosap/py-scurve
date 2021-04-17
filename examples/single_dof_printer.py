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
        self._q0 = [32.]
        self._q1 = [120.]
        self._v0 = [1.]
        self._v1 = [5.]
        self._v_max = 20.
        self._a_max = 15.
        self._j_max = 100.
        self.r_profiles = None

    @property
    def q0(self):
        return self._q0[0]

    @q0.setter
    def q0(self, value):
        self._q0[0] = value

    @property
    def q1(self):
        return self._q1[0]

    @q1.setter
    def q1(self, value):
        self._q1[0] = value

    @property
    def v0(self):
        return self._v0[0]

    @v0.setter
    def v0(self, value):
        self._v0[0] = value

    @property
    def v1(self):
        return self._v1[0]

    @v0.setter
    def v1(self, value):
        self._v1[0] = value

    def do_plan_trajectory(self, dt):
        traj = self.plan_trajectory(self._q0, self._q1, self._v0, self._v1, self._v_max, self._a_max, self._j_max)
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
