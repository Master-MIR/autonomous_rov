import pandas as pd
import matplotlib.pyplot as plt
import ast
import re

# 🔧 **Configurable Variables** (Change These Easily)
CSV_FILE = "_bluerov2_cmd_vel.csv"  # Path to CSV
PLOT_AXES = [0, 2]  # Select which velocity components to plot
PLOT_TITLE = "Linear X and Z Velocities Over Time"
LINE_WIDTH = 2  # Set line thickness
LEGEND_POSITION = "lower right"  # Position of legend
X_LABEL = "Time (s)"  # X-axis label
Y_LABEL = "Velocity (m/s)"  # Y-axis label

def parse_vector3(vector_str):
    """Extract x, y, z values from a ROS geometry_msgs.msg.Vector3 string."""
    match = re.findall(r"x=(-?\d+\.?\d*), y=(-?\d+\.?\d*), z=(-?\d+\.?\d*)", vector_str)
    if match:
        return [float(match[0][0]), float(match[0][1]), float(match[0][2])]
    return [0.0, 0.0, 0.0]  # Default in case of error

def load_joystick_data(csv_file):
    """Load and preprocess joystick data from a CSV file."""
    df = pd.read_csv(csv_file)

    # Convert timestamp to seconds (relative to first timestamp)
    df["Time"] = (df["timestamp"] - df["timestamp"].min()) / 1e9

    # Convert `_linear` and `_angular` columns into lists of floats
    df["_linear"] = df["_linear"].apply(parse_vector3)
    df["_angular"] = df["_angular"].apply(parse_vector3)

    return df

def plot_velocity_components(df, axes_indices, title):
    """Plot linear velocity components over time."""
    plt.figure(figsize=(10, 5))
    
    labels = ["Linear X", "Linear Y", "Linear Z"]
    for idx in axes_indices:
        plt.plot(df["Time"], df["_linear"].apply(lambda x: x[idx]), label=labels[idx], linewidth=LINE_WIDTH)

    # ✅ **Updated Axis Labels**
    plt.xlabel(X_LABEL, fontsize=20)
    plt.ylabel(Y_LABEL, fontsize=20)

    # ✅ **Updated Legend Position**
    plt.legend(fontsize=14, loc=LEGEND_POSITION)

    # ✅ **Improved Grid**
    plt.grid(visible=True, which='both', linestyle='--', linewidth=0.7)

    # ✅ **Tighter Layout for Better Readability**
    plt.tick_params(axis='both', labelsize=16)
    plt.tight_layout()
    plt.show()

# 🚀 **Run the Script**
df = load_joystick_data(CSV_FILE)
plot_velocity_components(df, axes_indices=PLOT_AXES, title=PLOT_TITLE)
