import numpy as np
import math
from math import degrees
from ahrs.filters import Madgwick
import time
import lsm9ds1

class MadgwickAHRS:
    samplePeriod = 1/256
    quaternion = Quaternion(1, 0, 0, 0)
    beta = 1
    zeta = 0

    def __init__(self, sampleperiod=None, quaternion=None, beta = None, zeta = None):
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta
        if zeta is not None:
            self.zeta = zeta

    def update(self, gyroscope, accelerometer, magnetometer):
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()
        magnetometer = np.array(magnetometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Normalise magnetometer measurement
        if norm(magnetometer) is 0:
            warnings.warn("magnetometer is zero")
            return
        magnetometer /= norm(magnetometer)

        h = q * (Quaternion(0, magnetometer[0], magnetometer[1], magnetometer[2]) * q.conj())
        b = np.array([0, norm(h[1:3]), 0, h[3]])

        # Gradient descent algorithm corrective step
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2],
            2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]) - magnetometer[0],
            2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]) - magnetometer[1],
            2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - magnetometer[2]
        ])
        j = np.array([
            [-2*q[2],                  2*q[3],                  -2*q[0],                  2*q[1]],
            [2*q[1],                   2*q[0],                  2*q[3],                   2*q[2]],
            [0,                        -4*q[1],                 -4*q[2],                  0],
            [-2*b[3]*q[2],             2*b[3]*q[3],             -4*b[1]*q[2]-2*b[3]*q[0], -4*b[1]*q[3]+2*b[3]*q[1]],
            [-2*b[1]*q[3]+2*b[3]*q[1], 2*b[1]*q[2]+2*b[3]*q[0], 2*b[1]*q[1]+2*b[3]*q[3],  -2*b[1]*q[0]+2*b[3]*q[2]],
            [2*b[1]*q[2],              2*b[1]*q[3]-4*b[3]*q[1], 2*b[1]*q[0]-4*b[3]*q[2],  2*b[1]*q[1]]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude

        # Gyroscope compensation drift
        gyroscopeQuat = Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])
        stepQuat = Quaternion(step.T[0], step.T[1], step.T[2], step.T[3]

        gyroscopeQuat = gyroscopeQuat + (q.conj() * stepQuat) * 2 * self.samplePeriod * self.zeta * -1

        # Compute rate of change of quaternion
        qdot = (q * gyroscopeQuat) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion

    def update_imu(self, gyroscope, accelerometer):
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Gradient descent algorithm corrective step
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2]
        ])
        j = np.array([
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude

        # Compute rate of change of quaternion
        qdot = (q * Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion



filt = Madgwick()
newfilt = MadgwickAHRS()

def quaternion_to_euler(q):

    rotation0 = 2 * (q[3] * q[0] + q[1] * q[2])
    rotation1 = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
    roll_x = math.atan2(rotation0, rotation1)

    rotation2 = 2 * (q[3] * q[1] - q[2] * q[0])
    if rotation2 > 1:
        rotation2 = 1
    if rotation2 < -1:
        rotation2 = -1
    pitch_y = math.asin(rotation2)

    rotation3 = 2 * (q[3] * q[2] + q[0] * q[1])
    rotation4 = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    yaw_z = math.atan2(rotation3, rotation4)

    return (roll_x, pitch_y, yaw_z)  # in radians

#vals = madgwick.updateIMU([0.7071, 0.0, 0.7071, 0.0],[-141,-53,-137],[-3306,-3978,15271])
#val = quaternion_to_euler(vals)
#print(val)
imu = lsm9ds1.make_i2c(1)

current_q = np.array([0.0, 1.0, 0.0, 0.0])
current_time = time.monotonic()

while True:
	mag_data_ready = imu.read_magnetometer_status()
	ag_data_ready = imu.read_ag_status()
	
	da_mag = mag_data_ready.data_available
	da_temp, da_gyro, da_acc = (ag_data_ready.temperature_data_available, ag_data_ready.gyroscope_data_available, ag_data_ready.accelerometer_data_available)
	
	if da_mag and da_gyro and da_acc:
		# in milli gauss
		mag = imu.mag_values()
		_, acc, gyr = imu.read_values()
		gyr = np.array(gyr) * math.pi / 180.0
		acc = np.array(acc) * 9.80665
		mag = np.array(mag) * 1.0E5
		
		#print(f"mag={mag} acc={acc} gyr={gyr}")
		
		new_time = time.monotonic()
		dt = new_time - current_time
		current_q = filt.updateMARG(
			current_q,
			gyr = gyr,
			acc = acc,
			mag = mag
		)
		current_time = new_time
		[roll, pitch, yaw] = quaternion_to_euler(current_q)
		print([roll,pitch,yaw])
		#print(f"roll={degrees(roll):.3f} pitch={degrees(pitch):.2f} yaw={degrees(yaw):.2f}")
		#mxc = (mx - MAG_MX_BIAS) / MAG_MX_NORM
		#myc = (my - MAG_MY_BIAS) / MAG_MY_NORM
		#mzc = (mz - MAG_MZ_BIAS) / MAG_MZ_NORM
	
