import pandas as pd
import numpy as np

# Config parameters
DATA_FILE = 'gyro_data.csv'  # Path to your CSV file
fs = 100  # Sampling rate in Hz (adjust to your data)
ts = 1.0 / fs  # Time step

# Load CSV into a DataFrame
data = pd.read_csv(DATA_FILE)
gx = data['x'].to_numpy()  # Angular velocity around X-axis [deg/s]
gy = data['y'].to_numpy()  # Angular velocity around Y-axis [deg/s]
gz = data['z'].to_numpy()  # Angular velocity around Z-axis [deg/s]

# Initialize Roll, Pitch, Yaw (starting from zero)
roll = np.cumsum(gx) * ts  # Roll angle [deg]
pitch = np.cumsum(gy) * ts  # Pitch angle [deg]
yaw = np.cumsum(gz) * ts  # Yaw angle [deg]

# Save to CSV (Optional)
output = pd.DataFrame({'roll': roll, 'pitch': pitch, 'yaw': yaw})
output.to_csv('euler_angles.csv', index=False)

# Plot results (Optional)
import matplotlib.pyplot as plt

plt.figure()
plt.plot(roll, label='Roll [deg]')
plt.plot(pitch, label='Pitch [deg]')
plt.plot(yaw, label='Yaw [deg]')
plt.xlabel('Sample Index')
plt.ylabel('Angle [deg]')
plt.legend()
plt.title('Relative Roll, Pitch, and Yaw from Gyroscope')
plt.grid(True)
plt.show()
