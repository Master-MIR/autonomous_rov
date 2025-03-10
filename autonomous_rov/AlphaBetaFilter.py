import numpy as np
from copy import deepcopy

class AlphaBetaFilter():
    def __init__(self, alpha=0.85, beta=0.005):
        self.alpha = alpha
        self.beta = beta
        self.position_estimate = None
        self.velocity_estimate = 0.0
        self.last_time = 0.0

    def filter(self, measurement, current_time):
        # measurement -> depth sensor data

        # if first time
        # position_est = position_measure
        # velocity_est = 0

        if self.position_estimate is None:
            # print("First time")
            self.position_estimate = measurement
            # print(self.position_estimate)
            self.last_time = deepcopy(current_time)
            # print(self.last_time)
            return self.position_estimate, self.velocity_estimate
        
        dt = current_time - self.last_time
        if dt <= 0:
            # print("dt <= 0")
            return self.position_estimate, self.velocity_estimate

        # Predict next position and velocity
        
        position_est_1 = self.position_estimate + dt * self.velocity_estimate
        velocity_est_1 = self.velocity_estimate

        # Update the position and velocity estimates
        # position_hat = position_est_1 + alpha * (position_measure - position_est_1)
        # velocity_hat = velocity_est_1 + beta * ((position_measure - position_est_1) / dt)
        residual = measurement - position_est_1
        position_hat = position_est_1 + self.alpha * residual
        velocity_hat = velocity_est_1 + (self.beta * residual) / dt

        self.last_time = deepcopy(current_time)

        return position_hat, velocity_hat
