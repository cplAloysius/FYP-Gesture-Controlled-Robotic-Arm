import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Config parameters
DATA_FILE = 'gyrocal.csv'  # Path to your CSV file
fs = 100  # Sampling rate in Hz (adjust to your data)
ts = 1.0 / fs  # Time step

# Load CSV without column headers
data = pd.read_csv(DATA_FILE, header=None)

# Extract gyro data (assuming 3 columns: gx, gy, gz)
gx = data.iloc[:, 0].to_numpy()
gy = data.iloc[:, 1].to_numpy()
gz = data.iloc[:, 2].to_numpy()

# Initialize Roll, Pitch, Yaw (relative)
roll = np.cumsum(gx) * ts  # Roll [deg]
pitch = np.cumsum(gy) * ts  # Pitch [deg]
yaw = np.cumsum(gz) * ts  # Yaw [deg]

# Save the computed Euler angles to a new CSV file (Optional)
output = pd.DataFrame({'roll': roll, 'pitch': pitch, 'yaw': yaw})
output.to_csv('euler_angles.csv', index=False, header=False)

# Plot the results (Optional)
plt.figure()
plt.plot(roll, label='Roll [deg]')
plt.plot(pitch, label='Pitch [deg]')
plt.plot(yaw, label='Yaw [deg]')
plt.xlabel('Sample Index')
plt.ylabel('Angle [deg]')
plt.legend()
plt.title('Gyroscope Drift')
plt.grid(True)
plt.show()
