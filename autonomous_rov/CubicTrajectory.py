import numpy as np

class CubicTrajectory():
    def __init__(self, time_init, time_final, z_init=0.0, z_final=0.5, dt=0.1):
        self.time_init = time_init
        self.time_final = time_final
        self.z_init = z_init
        self.z_final = z_final
        self.dt = dt
        self.time_values = np.arange(self.time_init, self.time_final , self.dt) 
        self.z_desired = []
        self.z_dot_desired = []
        self.a2 = (3.0 * (self.z_final - self.z_init) / (self.time_final**2))
        self.a3 = (-2.0 * (self.z_final - self.z_init) / (self.time_final**3))
        
    def generateCubicTrajectory(self):
        for i in self.time_values:
            if i < self.time_final:
                z = self.z_init + (self.a2 * i**2) + (self.a3 * i**3)
                z_dot = self.z_init + (2 * self.a2 * i) + (3 * self.a3 * i**2)
                self.z_dot_desired.append(z_dot)
                self.z_desired.append(z)
            elif i >= self.time_final: 
                self.z_desired.append(self.z_final)
                self.z_dot_desired.append(0.0)
            else:
                print("Error")
        return self.z_desired, self.z_dot_desired