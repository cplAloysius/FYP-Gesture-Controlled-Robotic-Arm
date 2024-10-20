import numpy as np
import matplotlib.pyplot as plt

# Config. params
DATA_FILE = 'ahrs.csv'  # CSV data file containing Euler angles "theta_x, theta_y, theta_z"
fs = 100  # Sample rate [Hz]

def AllanDeviation(dataArr, fs, maxNumM=100):
    ts = 1.0 / fs
    N = len(dataArr)
    Mmax = 2**np.floor(np.log2(N/2))
    M = np.logspace(np.log10(1), np.log10(Mmax), num=maxNumM)
    M = np.ceil(M)
    M = np.unique(M)

    taus = M * ts
    allanVar = np.zeros(len(M))
    for i, mi in enumerate(M):
        twoMi = int(2 * mi)
        mi = int(mi)
        allanVar[i] = np.sum(
            (dataArr[twoMi:N] - (2.0 * dataArr[mi:N-mi]) + dataArr[0:N-twoMi])**2
        )
    
    allanVar /= (2.0 * taus**2) * (N - (2.0 * M))

    return(taus, np.sqrt(allanVar))

# Load CSV into np array
dataArr = np.genfromtxt(DATA_FILE, delimiter=',')
ts = 1.0 / fs

# Separate into Euler angle arrays
thetax = dataArr[:, 0]  # [degrees]
thetay = dataArr[:, 1]  # [degrees]
thetaz = dataArr[:, 2]  # [degrees]

# Compute Allan deviations for Euler angles
(taux, adx) = AllanDeviation(thetax, fs, maxNumM=200)
(tauy, ady) = AllanDeviation(thetay, fs, maxNumM=200)
(tauz, adz) = AllanDeviation(thetaz, fs, maxNumM=200)

# Plot data on log-scale
plt.figure()
plt.title('AHRS Euler Angle Allan Deviations')
plt.plot(taux, adx, label='theta_x')
plt.plot(tauy, ady, label='theta_y')
plt.plot(tauz, adz, label='theta_z')
plt.xlabel(r'$\tau$ [sec]')
plt.ylabel('Deviation [degrees]')
plt.grid(True, which="both", ls="-", color='0.65')
plt.legend()
plt.xscale('log')
plt.yscale('log')
plt.show()
