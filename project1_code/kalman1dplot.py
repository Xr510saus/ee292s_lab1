import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


df = pd.read_csv('kalman1d.csv')
plt.figure()

# state estimate data
t = df['time'].to_numpy()
x = df['x'].to_numpy()
vx = df['vx'].to_numpy()
ax = df['ax'].to_numpy()

gnd_truth = np.full(t.size, 0.305) # 1 foot distance GND truth

z = df['z'].to_numpy() # measured accel data
ma_z = df['ma_z']
# plot the dataframe

print(f'variance = {np.var(z)}')

plt.title('Acceleration X')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.plot(t, z)
plt.plot(t, ma_z)
plt.plot(t, ax)
plt.legend(['Measured' ,'Avg_Meas', 'StateVar'])

plt.figure()
plt.title('Position X')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.plot(t, x)
plt.plot(t, gnd_truth)
plt.legend(['GND Truth', 'StateVar'])


plt.figure()
plt.title('Velocity X')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.plot(t, vx)
plt.legend(['StateVar'])

plt.show()