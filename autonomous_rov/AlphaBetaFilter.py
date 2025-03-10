import numpy as np

class AlphaBetaFilter():
    def __init__(self, alpha, beta):
        self.alpha = alpha
        self.beta = beta
        self.position_estimate = 0.0
        self.velocity_estimate = 0.0

    def filter(self, measurement, dt):
        # if first time
        # position_est = position_measure
        # velocity_est = 0

        # Predict next position and velocity
        # position_est_1 = position_est + dt * velocity_est
        # velocity_est_1 = velocity_est

        # Update the position and velocity estimates
        # position_hat = position_est_1 + alpha * (position_measure - position_est_1)
        # velocity_hat = velocity_est_1 + beta * ((position_measure - position_est_1) / dt)

        
