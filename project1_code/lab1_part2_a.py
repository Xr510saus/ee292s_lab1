from ICM20948 import *
import numpy as np
import math
import time
from filterpy.kalman import KalmanFilter
import pandas as pd

from ma_filter import maf

dt = 0.1 # delta time for loop updates, 10 samples/sec
g = 9.81 # gravity, m/s^2

### combines gyro and accel measurements
def complimentary_filter(dt, oldAngle, accelTilt, gyroTilt, alpha=0.2):
    phi = 0 #roll
    pitch = 0 #theta
    yaw = 0 # poseidon thing psi?

    return accelTilt*alpha + (1-alpha)*(oldAngle + gyroTilt)


### calculate tilt angle
def calc_accel_tilt():
    x = Accel[0]
    y = Accel[1]
    z = Accel[2]
    return math.atan2(math.sqrt(x**2 + y**2), z) * 180/math.pi


icm20948=ICM20948() # instatiate IMU object

gyroXangle = 0
gyroYangle = 0
gyroZangle = 0

kf = KalmanFilter(dim_x=3,dim_z=1)      # state vector is [x v], v=xdot=velocity

# state matrix - Kinematics modeling
kf.F = np.array([[1., dt, 0.5*dt*dt],       # pos_new = old_pos + dt*vel + 1/2*accel*dt^2
                 [0., 1., dt],              # vel_new = old_vel + dt*accel
                 [0., 0,  1. ]])              # accel = accel (constant model)

kf.H = np.array([[0.,0.,1.]]) # measurement matrix checks acceleration
kf.x = np.array([0., 0., 0.]) # guess: position = 0m, velocity = 1m/s, accelereation = 0.2 m/s^2
kf.P = np.array([[ 0.05,  0.,  0.],
                 [ 0.,  0.05,  0.] ,
                 [ 0.,  0.,  0.05]])

kf.Q = np.full((3,3), 10.)

##               ([[5.00000000e-07, 1.25000000e-05, 1.66666667e-04],
##                 [1.25000000e-05, 3.33333333e-04, 5.00000000e-03],
##                 [1.66666667e-04, 5.00000000e-03, 1.00000000e-01]])

measVar = 3e-5 #TODO: wild ass guess for now, take measurements later from IMU Allan plot
kf.R = np.array([[ measVar ]])          # Variance of measurement
kf.B = np.array([0., 0., 0.])

# Utility arrays to save off Kalman state over time
ts=[]
xs=[]
vxs=[]
axs=[]
zs = [] # sensor output
kgs=[]

ay = []
print("Calibrating...get ready")
cal_time = 2 # seconds for calibration
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

g = 9.8 # m/s^2
ma_zs = []
currTime = 0
while True:
    icm20948.icm20948_Gyro_Accel_Read()
    time.sleep(dt) # delay by delta t
    currTime += dt

    ts.append(currTime)

    ####  KALMAN FILTERING STEP   ####
    theta = calc_accel_tilt()
    ax_meas = (Accel[0] / 16384 * 9.8 - bias) #+ g * np.sin(theta)) * np.cos(theta) - bias # get acceleration data in g's

    # dead-banding accel measurements if very low--considered stationary noise
    if abs(ax_meas) < 0.02:
        ax_meas = 0

    ma_len = 5
    zs.append(ax_meas)                        # Save off sensor output
    if len(zs) > ma_len - 1:
        ax_meas = maf(zs, ax_meas, fil_len=ma_len) # 3-point moving average to smoothen accel
    ma_zs.append(ax_meas) # append smoothened value

    ## attempted g-compensation
    #ax_meas = (Accel[0] / 16384 * 9.8 - bias) + g * np.sin(theta)) * np.cos(theta) - bias
    kf.predict()                        # State prediction
    kf.update(ax_meas)                        # Update
    kgs.append(kf.K)                    # Save off Kalman gain
    xs.append(kf.x[0])                  # Save off position estimate
    vxs.append(kf.x[1])                  # Save off velocity estimate
    axs.append(kf.x[2])                 # save off acceleration estimate
    # print(ax_meas)
    # print(kf.K)

    # print(currTime)
    if currTime > 8: # 5 seconds passed, break
        # save to csv file
        axs = np.array(axs)
        vxs = np.array(vxs)
        xs = np.array(xs)
        ts = np.array(ts)
        z = np.array(zs)
        ma_zs = np.array(ma_zs)
        print(f'length: {len(axs)} samples')
        # kgs = np.array(kgs)

        # print(axs.size)
        # print(vxs.size)
        # print(xs.size)
        # print(ts.size)
        # dump arrays to csv file
        df = pd.DataFrame({"time" : ts, "x" : xs, "vx" : vxs, "ax" : axs, "z":zs, "kgs":kgs, "ma_z":ma_zs})
        df.to_csv("kalman1d.csv", index=False)
        print("DONE")
        break # end program

    # accel_tilt = calc_accel_tilt(icm20948) # calculate level using accelerometer
    # gyroXdps = (Gyro[0] / 32.8)
    # gyroYdps = (Gyro[1] / 32.8)
    # gyroZdps = (Gyro[2] / 32.8)

    # gyroXangle = (gyroXangle + dt*gyroXdps)
    # gyroYangle = (gyroYangle + dt*gyroYdps)
    # gyroZangle = (gyroZangle + dt*gyroZdps)
    # gyroAngles = [gyroXangle, gyroYangle, gyroZangle]

    # print(f'gyroX={Gyro[0]}, gyroY={Gyro[1]}, gyroZ={Gyro[2]}')
    # print(f'gyroXAngle={gyroAngles[0]}, gyroYAngle={gyroAngles[1]}, gyroZAngle={gyroAngles[2]}')
    # print(accel_tilt_sense(icm20948))