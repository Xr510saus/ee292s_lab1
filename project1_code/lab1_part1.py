from ICM20948 import *
import numpy as np
import math
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation

dt = 0.1 # delta time for loop updates, 10 samples/sec

### combines gyro and accel measurements
def complimentary_filter(dt, oldAngle, accelTilt, gyroTilt, alpha=0.2):
    '''
    DOES NOT WORK AS GYROTILT DOES NOT WORK
    Inputs:
        dt: float # Sampling period
        oldAngle: float # Previous board angle
        accelTilt: float # Current angle, according to the accelerometer
        gyroTilt: float # Current angle, according to the accelerometer
        alpha: float # Weight of the accelTilt. Weight of gyroTilt is 1-alpha
    Outputs:
        (Output): float # Fused angle
    '''

    phi = 0 #roll
    pitch = 0 #theta
    yaw = 0 # poseidon thing psi?

    return accelTilt*alpha + (1-alpha)*(oldAngle + gyroTilt)


# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    '''
    Inputs:
        i: int # Frame number
        xs: list # x-axis values
        ys: list # y-axis values
    '''
    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Tilt Sensor')
    plt.ylabel('Degrees')
    plt.xlabel('Time (s)')


### calculate tilt angle
def calc_accel_tilt():
    '''
    Outputs:
        (Output): float # Angle calculated by accelerometer values
    '''
    x = Accel[0] # a_x
    y = Accel[1] # a_y
    z = Accel[2] # a_z

    return math.atan2(math.sqrt(x**2 + y**2), z) * 180/math.pi # computes angle relative to global XY plane (orthog to z-axis)

def calc_gyro_tilt(gx, gy, gz, dt, C):
    '''
    DOES NOT WORK
    Inputs:
        gx: float # Degrees per second around the x-axis
        gy: float # Degrees per second around the y-axis
        gz: float # Degrees per second around the z-axis
        dt: float # Sampling period
        C: 3x3 matrix of floats # Previous rotation matrix
    Outputs:
        Cnew: 3x3 matrix of floats # New rotation matrix
        theta: float # Angle of the board
    Description:
        I found some code online that calculated the rotation
        matrix relative to the original plane. Thus, I tried using
        the vector (1, 1, 0) as a "reference vector," getting the
        orientation of the "new vector" wrt the reference vector,
        then calculating the angle of the reference vector wrt
        the xy-plane. If the new vector passed the x=y hyperplane,
        then the angle would be subtracted from 180 degrees.
    '''
    B = np.matrix([[0, -gz*dt, gy*dt],
                   [gz*dt, 0, -gx*dt],
                   [-gy*dt, gx*dt, 0]])

    Cnew = np.matmul(C, np.exp(B))

    new_vec = np.matmul(Cnew, [[1], [1], [0]])

    theta = math.atan(new_vec[2] / np.sqrt(new_vec[0]**2 + new_vec[1]**2))

    if new_vec[0] < new_vec[1]:
        theta = 180 - theta

    return Cnew, theta


# Initialize communication with IMU
icm20948=ICM20948()

gyroXangle = 0 # Set the current angle (according to gyro) to be 0
gyroYangle = 0
gyroZangle = 0

C = np.matrix([[1, 0, 0], # Rotation matrix. Does not work.
               [0, 1, 0],
               [0, 0, 1]])

while True:
    icm20948.icm20948_Gyro_Accel_Read()
    time.sleep(dt) # delay by delta t
    currTime += dt

    ts.append(currTime)
    accel_tilt.append(calc_accel_tilt()) # calculate level using accelerometer

    gyroXdps = (Gyro[0] / 32.8) # Convert bit readings to dps
    gyroYdps = (Gyro[1] / 32.8)
    gyroZdps = (Gyro[2] / 32.8)

    # C, gth = calc_gyro_tilt(gyroXdps, gyroYdps, gyroZdps, dt, C)

    gyroXangle = (gyroXangle + dt*gyroXdps) # Add new integrated gryo angle to old values
    gyroYangle = (gyroYangle + dt*gyroYdps)
    gyroZangle = (gyroZangle + dt*gyroZdps)
    gyroAngles = [gyroXangle, gyroYangle, gyroZangle]

    # print(f'gyroX={Gyro[0]}, gyroY={Gyro[1]}, gyroZ={Gyro[2]}')
    # print(f'gyroXAngle={gyroAngles[0]}, gyroYAngle={gyroAngles[1]}, gyroZAngle={gyroAngles[2]}')

    print('Acc Angle: ', calc_accel_tilt())
    print(f'Gyro X Angle = {gyroAngles[0]}')
    print(f'Gyro Y Angle = {gyroAngles[1]}')
    # print(f'Gyro Z Angle = {gyroAngles[2]}')
    print('---------')

    # Set up plot to call animate() function periodically
    # ani = animation.FuncAnimation(fig, animate, fargs=(ts, accel_tilt), interval=1000)
    # print(accel_tilt_sense(icm20948))









