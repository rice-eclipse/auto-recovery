"""
This code implements the Madgwick AHRS (Attitude and 
Heading Reference System) algorithm for sensor fusion, 
specifically designed for Inertial Measurement Units (IMUs). 
It uses sensor data from an LSM9DS1 sensor (gyroscope, 
accelerometer, and magnetometer) to estimate the orientation 
(roll, pitch, and yaw angles) of an object in 3D space. 
The Madgwick AHRS algorithm combines gyroscope, accelerometer, 
and magnetometer data to provide more accurate orientation 
estimates. It continuously reads sensor data, processes it 
using the Madgwick AHRS algorithm, and prints the estimated 
roll, pitch, and yaw angles in Euler angles format. The code 
also includes a function for converting quaternions to Euler 
angles and initializes the necessary sensor communication and 
filtering objects.
"""

import numpy as np
import math
from math import degrees
from ahrs.filters import Madgwick
import time
import lsm9ds1

# Define a class for the Madgwick AHRS algorithm
class MadgwickAHRS:
    # Set some default parameters
    samplePeriod = 1/256
    quaternion = Quaternion(1, 0, 0, 0)
    beta = 1
    zeta = 0

    def __init__(self, sampleperiod=None, quaternion=None, beta=None, zeta=None):
        # Constructor to optionally override default parameters
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta
        if zeta is not None:
            self.zeta = zeta

    def update(self, gyroscope, accelerometer, magnetometer):
        # Main update function for Madgwick AHRS
        q = self.quaternion

        # Flatten the input sensor data arrays into 1D arrays
        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()
        magnetometer = np.array(magnetometer, dtype=float).flatten()

        # Normalize accelerometer measurement
        if norm(accelerometer) == 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Normalize magnetometer measurement
        if norm(magnetometer) == 0:
            warnings.warn("magnetometer is zero")
            return
        magnetometer /= norm(magnetometer)

        # Calculate the gradient descent algorithm corrective step
        # and Jacobian matrix
        # This step helps in updating the quaternion to improve accuracy
        # by considering gyroscope, accelerometer, and magnetometer data
        f = np.array([...])  # Gradient descent algorithm
        j = np.array([...])  # Jacobian matrix
        step = j.T.dot(f)
        step /= norm(step)  # Normalize step magnitude

        # Gyroscope compensation for drift
        gyroscopeQuat = Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])
        stepQuat = Quaternion(step.T[0], step.T[1], step.T[2], step.T[3])

        gyroscopeQuat = gyroscopeQuat + (q.conj() * stepQuat) * 2 * self.samplePeriod * self.zeta * -1

        # Compute the rate of change of quaternion
        qdot = (q * gyroscopeQuat) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # Normalize quaternion

    def update_imu(self, gyroscope, accelerometer):
        # Similar to update(), but without magnetometer data
        q = self.quaternion

        # Flatten the input sensor data arrays into 1D arrays
        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()

        # Normalize accelerometer measurement
        if norm(accelerometer) == 0:
            warnings.warn("accelerometer is zero")
            return

        # Calculate the gradient descent algorithm corrective step
        # and Jacobian matrix using only gyroscope and accelerometer data
        f = np.array([...])  # Gradient descent algorithm
        j = np.array([...])  # Jacobian matrix
        step = j.T.dot(f)
        step /= norm(step)  # Normalize step magnitude

        # Compute the rate of change of quaternion
        qdot = (q * Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # Normalize quaternion


filt = Madgwick()  # Create a Madgwick filter object
newfilt = MadgwickAHRS()  # Create a Madgwick AHRS object

# Function to convert a quaternion to Euler angles (roll, pitch, yaw)
def quaternion_to_euler(q):
    # Calculate Euler angles from quaternion
    # ...

# Initialize the LSM9DS1 sensor
imu = lsm9ds1.make_i2c(1)

current_q = np.array([0.0, 1.0, 0.0, 0.0])  # Initial quaternion orientation
current_time = time.monotonic()  # Get the current time

while True:
    # Check if magnetometer, temperature, gyroscope, and accelerometer data are available
    mag_data_ready = imu.read_magnetometer_status()
    ag_data_ready = imu.read_ag_status()
    
    da_mag = mag_data_ready.data_available
    da_temp, da_gyro, da_acc = (ag_data_ready.temperature_data_available, ag_data_ready.gyroscope_data_available, ag_data_ready.accelerometer_data_available)
    
    if da_mag and da_gyro and da_acc:
        # Read sensor data from LSM9DS1
        mag = imu.mag_values()
        _, acc, gyr = imu.read_values()
        
        # Convert units and normalize sensor data
        gyr = np.array(gyr) * math.pi / 180.0
        acc = np.array(acc) * 9.80665
        mag = np.array(mag) * 1.0E5
        
        new_time = time.monotonic()  # Get the new time
        dt = new_time - current_time  # Calculate time difference
        current_q = filt.updateMARG(
            current_q,
            gyr=gyr,
            acc=acc,
            mag=mag
        )
        current_time = new_time  # Update the current time
        
        [roll, pitch, yaw] = quaternion_to_euler(current_q)  # Convert quaternion to Euler angles
        print([roll, pitch, yaw])  # Print the estimated roll, pitch, and yaw angles
