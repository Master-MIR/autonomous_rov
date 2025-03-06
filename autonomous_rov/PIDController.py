from cmath import pi
import numpy as np
from copy import deepcopy


class PIDController:
    def __init__(self, k_p=0.0, k_i=0.0, k_d=0.0, sat=10.0, type='linear'):
        # variables declaration
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.sat = sat

        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.err = 0.0
        self.prev_err = 0.0
        self.diff_err = 0.0
        self.int_err = 0.0

        self.t = 0.0
        self.prev_t = -1.0

        self.type = type

    def calculate_error(self, desired, actual, t):
        # calculate error

        self.err = desired - actual
        if self.type == 'linear':
            pass
        elif self.type == 'angular':
            if self.err > np.pi:
                self.err = self.err - (2.0 * np.pi)
            elif self.err < -np.pi:
                self.err = self.err + (2.0 * np.pi)

        self.t = t
        dt = self.t - self.prev_t

        if (self.prev_t == -1.0):  # first time

            self.diff_err = 0.0
            self.int_err = 0.0

        elif dt > 0.0:

            # calculate derivative error

            self.diff_err = (self.err - self.prev_err) / dt

            # calucalte integral error

            self.int_err = self.int_err + \
                ((self.err + self.prev_err) * dt / 2.0)

    def calculate_pid(self, desired, actual, t):
        # calculate PID
        self.calculate_error(desired, actual, t)

        self.P = self.k_p * self.err
        self.I = self.k_i * self.int_err
        self.D = self.k_d * self.diff_err
        PID = self.P + self.I + self.D

        self.prev_err = deepcopy(self.err)
        self.prev_t = deepcopy(t)

        # calculate saturation

        # if (np.linalg.norm(PID) > self.sat):
        #     PID = self.sat * PID / np.linalg.norm(PID)
        #     self.int_err = 0.0

        return PID

    def reconfig_param(self, k_p, k_i, k_d):
        # update the variables
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        # reset integral
        self.int_err = 0.0
