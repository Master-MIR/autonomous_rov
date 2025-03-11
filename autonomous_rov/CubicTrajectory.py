import numpy as np

class CubicTrajectory:
    def __init__(self, z_init=0.0, z_final=0.5):
        """
        Initialize cubic trajectory parameters.
        
        :param z_init: Initial depth position
        :param z_final: Final depth position
        """
        self.z_init = z_init
        self.z_final = z_final

    def get_waypoint(self, t, time_init, time_final):
        """
        Compute the desired depth (z) and velocity (z_dot) for a given time t.
        
        :param t: Current time in seconds
        :param time_init: Start time of the trajectory
        :param time_final: End time of the trajectory
        :return: (z, z_dot)
        """
        if t < time_init:
            return self.z_init, 0.0
        elif t < time_final:
            a2 = (3.0 * (self.z_final - self.z_init) / ((time_final-time_init)**2))
            a3 = (-2.0 * (self.z_final - self.z_init) / ((time_final-time_init)**3))
            z = self.z_init + (a2 * (t - time_init)**2) + (a3 * (t - time_init)**3)
            z_dot = (2 * a2 * (t - time_init)) + (3 * a3 * (t - time_init)**2)
            return z, z_dot
        else:####
            return self.z_final, 0.0
