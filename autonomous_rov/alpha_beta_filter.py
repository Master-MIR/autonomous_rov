import numpy as np
import matplotlib.pyplot as plt


def alpha_beta_filter(depth_measurements,alpha=0.1,beta = 0.005,dt=1/58,position_estimate = 0.0,velocity_estimate = 0.0):

    """
    Alpha-Beta Filter for Depth Measurements
    
    Parameters:
    - depth_measurements: List or array of measured depth values
    - alpha: Position update weight (default: 0.1)
    - beta: Velocity update weight (default: 0.005)
    - dt: Time step in seconds (default: 1/58)
    - position_estimate: Initial position estimate (default: 0.0)
    - velocity_estimate: Initial velocity estimate (default: 0.0)
    
    Returns:
    - position_estimates: Filtered position values
    - velocity_estimates: Filtered velocity values
    - time: Time array corresponding to measurements
    """
      
    # Lists to store the results
    position_estimates = []
    velocity_estimates = []

    # Apply alpha-beta filter
    for p_measured in depth_measurements:
        # Predict next position and velocity
        p_predicted = position_estimate + dt * velocity_estimate
        v_predicted = velocity_estimate

        # Update the position and velocity estimates
        position_estimate = p_predicted + alpha * (p_measured - p_predicted)
        velocity_estimate = v_predicted + beta * (p_measured - p_predicted)

        # Store the results
        position_estimates.append(position_estimate)
        velocity_estimates.append(velocity_estimate)

    # Convert to numpy arrays for easier handling
    position_estimates = np.array(position_estimates)
    velocity_estimates = np.array(velocity_estimates)
    
    time = np.arange(0,len(depth_measurements))*dt
    return position_estimates,velocity_estimates,time

def plot_alpha_beta(thruster_command,depth_measurements,position_estimates,velocity_estimates,time):

    '''
    This function is used to visualise the alpha beta filter and to discuss its results with the commanded thrust.
    '''

    # Plot the results
    plt.figure(figsize=(10, 12))

    plt.subplot(3, 1, 1)
    
    plt.plot(time, thruster_command, label="Thruster command", linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Thrust [N]')
    plt.grid(True)
    plt.legend()
    plt.title('Thrust command')


    plt.subplot(3, 1, 2)
    plt.plot(time, depth_measurements, label="Noisy Depth Measurement", alpha=0.5)
    plt.plot(time, position_estimates, label="Filtered Position Estimate", linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Depth [m]')
    plt.grid(True)
    plt.legend()
    plt.title('Depth (Position) Estimation')

    plt.subplot(3, 1, 3)
    plt.plot(time, velocity_estimates, label="Heave estimation", color='r', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.legend()
    plt.title('Vertical Speed (Heave) Estimation')
    plt.grid(True)
    plt.tight_layout()
    plt.show()