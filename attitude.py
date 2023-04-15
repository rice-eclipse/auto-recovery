import numpy as np
import math
from math import degrees
from ahrs.filters import Madgwick
import time
import lsm9ds1

filt = Madgwick()

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
		print(current_q)
		#print(f"roll={degrees(roll):.3f} pitch={degrees(pitch):.2f} yaw={degrees(yaw):.2f}")
		#mxc = (mx - MAG_MX_BIAS) / MAG_MX_NORM
		#myc = (my - MAG_MY_BIAS) / MAG_MY_NORM
		#mzc = (mz - MAG_MZ_BIAS) / MAG_MZ_NORM
	
