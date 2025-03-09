import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# Load the Excel file
file_path = 'autonomous_rov/resource/T200-Public-Performance-Data-10-20V-September-2019.xlsx'
xls = pd.ExcelFile(file_path)

# Display sheet names to identify the correct one for 12V
print("Available sheets:", xls.sheet_names)

# Load the sheet corresponding to 12V operation
# Replace '12V' with the actual sheet name if it's different
df = pd.read_excel(xls, sheet_name='12 V')

# Display the first few rows to understand the structure
print(df.head())
#print(df.keys())

# Extract PWM and Thrust columns
# Extract relevant columns (ensure column names match exactly)
pwm_values = df[' PWM (µs)']
thrust_kgf = df[' Force (Kg f)']  # Force in kgf

# Convert thrust from kgf to Newtons (1 kgf ≈ 9.80665 N)
# # Convert thrust from kgf to Newtons (1 kgf = 10 N)
thrust_newtons = [force * 10 for force in thrust_kgf]

# Create the plot
plt.figure(1,figsize=(10, 6))
plt.plot(thrust_newtons, pwm_values,  color='C0', label='PWM vs. Thrust')

# Labels and title
plt.xlabel('Thrust $[N]$')
plt.ylabel('PWM $[\mu s]$')
plt.title('PWM Signal vs. Thrust at 12V')
plt.legend()
plt.grid(True)

# Show the plot
pwm_values=np.array(pwm_values)
thrust_newtons=np.array(thrust_newtons)
# Split the data based on PWM threshold
threshold = 0  # Example: you might use a different value based on your data
Thrust_neg = thrust_newtons[thrust_newtons < threshold]
Thrust_pos = thrust_newtons[thrust_newtons >= threshold]

PWM_neg = pwm_values[thrust_newtons < threshold]  # Negative PWM values
PWM_pos = pwm_values[thrust_newtons >= threshold]  # Positive PWM values





# To calculate uncertainties, we will use the covariance of the fit.
# Use np.polyfit with full=True to retrieve the covariance matrix
m_neg, b_neg = np.polyfit(Thrust_neg,PWM_neg, 1)
m_pos, b_pos= np.polyfit(Thrust_pos,PWM_pos, 1)
_, cov_neg = np.polyfit(Thrust_neg,PWM_neg , 1, cov=True)
_, cov_pos = np.polyfit(Thrust_pos,PWM_pos , 1, cov=True)

# Uncertainty is the square root of the diagonal of the covariance matrix
m_neg_uncertainty = np.sqrt(cov_neg[0, 0])
b_neg_uncertainty = np.sqrt(cov_neg[1, 1])

m_pos_uncertainty = np.sqrt(cov_pos[0, 0])
b_pos_uncertainty = np.sqrt(cov_pos[1, 1])
print(m_neg_uncertainty,m_pos_uncertainty,b_neg_uncertainty,b_pos_uncertainty)
# Plot the original cubic curve and the linear fits
plt.figure(2,figsize=(10, 6))
plt.plot(thrust_newtons,pwm_values,  label='Original 12V PWM curve', color='C0', linestyle='dashed')

b_pos = round(b_pos / 10) * 10
b_neg = round(b_neg / 10) * 10
# Plot the linear approximations
plt.plot(Thrust_neg, m_neg * Thrust_neg + b_neg, label=f'Linear Fit for Negative Thrust PWM(f) = {b_neg:.0f} + {m_neg:.0f} * f', color='red')
plt.plot(Thrust_pos, m_pos * Thrust_pos + b_pos, label=f'Linear Fit for Positive Thrust PWM(f) = {b_pos:.0f} + {m_pos:.0f} * f', color='green')

plt.xlabel('Thrust $[N]$')
plt.ylabel('PWM $[\mu s]$')
plt.title('PWM Signal vs. Thrust at 12V')
plt.legend()
plt.grid(True)
plt.show()

