import numpy as np
import matplotlib.pyplot as plt

time_init = 0
time_now = 0
time_final = 20
dt = 0.01

# Generate time array
time_values = np.arange(time_init, time_final , dt) 

z_desired = []
z_init = 0.0
z_final = 0.5

z_dot_desired = []

a2 = (3.0 * (z_final - z_init) / (time_final**2))
a3 = (-2.0 * (z_final - z_init) / (time_final**3))

for i in time_values:

    print("Time: ", time_now)
    # print(i)
    if i < time_final:
        z = z_init + (a2 * time_now**2) + (a3 * time_now**3)
        z_dot = z_init + (2 * a2 * time_now) + (3 * a3 * time_now**2)
        z_dot_desired.append(z_dot)
        z_desired.append(z)

    elif i >= time_final: 
        print("time now >= time final")
        z_desired.append(z_final)
        z_dot_desired.append(0.0)
        # print(z_desired)

    else:
        print("Error")

    time_now += dt

# Plot the cubic trajectory
plt.plot(time_values, z_desired, label="Cubic Spline")
plt.plot(time_values, z_dot_desired, label="Cubic Spline Velocity")
plt.xlabel("Time")
plt.ylabel("Position (z)")
plt.title("Cubic Spline Trajectory")
plt.legend()
plt.grid()
plt.show()