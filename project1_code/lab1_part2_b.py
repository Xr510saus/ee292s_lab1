from ICM20948 import *
import numpy as np
import math
import time
from filterpy.kalman import KalmanFilter
import pandas as pd

dt = 0.1 # delta time for loop updates, 10 samples/sec
g = 9.81 # gravity, m/s^2

### combines gyro and accel measurements
def complimentary_filter(dt, oldAngle, accelTilt, gyroTilt, alpha=0.2):
    phi = 0 #roll
    pitch = 0 #theta
    yaw = 0 # psi

    return accelTilt*alpha + (1-alpha)*(oldAngle + gyroTilt)

### calculate tilt angle using acceleration
def calc_accel_tilt(imu):
    x = imu[0]
    y = imu[1]
    z = imu[2]
    return math.atan2(math.sqrt(x**2 + y**2), z) * 180/math.pi



icm20948=ICM20948()

gyroXangle = 0
gyroYangle = 0
gyroZangle = 0

kf = KalmanFilter(dim_x=8,dim_z=3)      # state vector is [x v], v=xdot=velocity

# state matrix - Kinematics modeling
kf.F = np.array([[1.,  dt., 0.5*dt*dt.,   0., 0., 0., 0., 0.],
                 [0.,  1.,  dt.,          0., 0., 0., 0., 0.],
                 [0.,  0.,  1.,           0., 0., 0., 0., 0.],
                 [0.,  0., 0.,   1., dt., 0.5*dt*dt, 0., 0.],
                 [0.,  0., 0.,   0., 1.,  dt.,       0., 0.],
                 [0.,  0., 0.,   0., 0.,  1.,        0., 0.],
                 [0.,  0., 0., 0., 0., 0.,           1., dt.],
                 [0.,  0., 0., 0., 0., 0.,           0., 1.]])

kf.H = np.array([[0.,0.,1.,      0.,0.,1.,      0., 1.]]) # measurement matrix grabs ax, ay, and omega
kf.x = np.array([0., 0., 0.,      0., 0., 0.,   0., 0.]) # guess: position = 0m, velocity = 1m/s, accelereation = 0.2 m/s^2
kf.P = np.array([[0.5,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0.5, 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0.5, 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0.5, 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0.5, 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0.5, 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.]])

kf.Q = np.array([[0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.],
                 [0.,  0., 0., 0., 0., 0., 0., 0.]])            # No process noise (very stable airplane), np.covariance

measVar = 0.1 #TODO: wild ass guess for now, take measurements later from IMU
kf.R = np.array([[ measVar ]])          # Variance of measurement
kf.B = np.array([0., 0., 0.])

# Utility arrays to save off Kalman state over time
ts=[] # time
xs=[] # position state
vxs=[] # velocity state
axs=[] # acceleration state
zs = [] # sensor output
kgs=[] # kalman gains

ay = []
print("Calibrating...get ready")
cal_time = 2 # seconds for calibration
# find average at rest for bias to subtract
for x in range(int(cal_time / dt)):
    theta = calc_accel_tilt()
    ay.append((Accel[0] / 16384 * g)) # + g * np.sin(theta)) * np.cos(theta)) # convert IMU data to g's and then m/s^2 w/gravity compensation
ay = np.array(ay)
bias = np.mean(ay)
print(f'Bias={bias} m/s^2')
print("STARTING LOGGING")


# field names
# fields = ['Time', 'X', 'V_x', 'A_x']
# filename = "kalman1d.csv"
# writing to csv file
# with open(filename, 'w') as csvfile:
#     # creating a csv writer object
#     csvwriter = csv.writer(csvfile)
#     # writing the fields
#     csvwriter.writerow(fields)
#     # writing the data rows
#     csvwriter.writerows(rows)

currTime = 0
while True:
    icm20948.icm20948_Gyro_Accel_Read()
    time.sleep(dt) # delay by delta t
    currTime += dt

    ts.append(currTime)

    ####  KALMAN FILTERING STEP   ####
    ax_meas = Accel[0] / 16384  - bias # get acceleration data in g's

    # TODO: smooth AX with MOVING AVERAGE FILTER/LOW PASS, possibly HIGHPASS for bias
    kf.predict()                        # State prediction
    kf.update(ax_meas)                        # Update
    kgs.append(kf.K)                    # Save off Kalman gain
    zs.append(ax_meas)                        # Save off sensor output
    xs.append(kf.x[0])                  # Save off position estimate
    vxs.append(kf.x[1])                  # Save off velocity estimate
    axs.append(kf.x[2])                 # save off acceleration estimate
    # print(kf.K)

    # print(currTime)
    if currTime > 30: # 5 seconds passed, break
        # save to csv file
        axs = np.array(axs)
        vxs = np.array(vxs)
        xs = np.array(xs)
        ts = np.array(ts)
        z = np.array(zs)
        # kgs = np.array(kgs)

        # print(axs.size)
        # print(vxs.size)
        # print(xs.size)
        # print(ts.size)
        # dump arrays to csv file
        df = pd.DataFrame({"time" : ts, "x" : xs, "vx" : vxs, "ax" : axs, "z":zs, "kgs":kgs})
        df.to_csv("kalman1d.csv", index=False)
        print("DONE")
        break # end program

    # accel_tilt = calc_accel_tilt(icm20948) # calculate level using accelerometer
    # gyroXdps = (Gyro[0] / 32.8)
    # gyroYdps = (Gyro[1] / 32.8)
    # gyroZdps = (Gyro[2] / 32.8)

    #TODO: moving average AND bias removal

    # gyroXangle = (gyroXangle + dt*gyroXdps)
    # gyroYangle = (gyroYangle + dt*gyroYdps)
    # gyroZangle = (gyroZangle + dt*gyroZdps)
    # gyroAngles = [gyroXangle, gyroYangle, gyroZangle]

    # print(f'gyroX={Gyro[0]}, gyroY={Gyro[1]}, gyroZ={Gyro[2]}')
    # print(f'gyroXAngle={gyroAngles[0]}, gyroYAngle={gyroAngles[1]}, gyroZAngle={gyroAngles[2]}')
    # print(accel_tilt_sense(icm20948))