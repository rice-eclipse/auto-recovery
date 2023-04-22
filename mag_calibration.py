import lsm9ds1
import numpy as np
import time
import json
import math
PI = math.pi

def calibrate_mag(duration_seconds: float, north_fix_seconds: float):
	print("rotate along X, Y, Z axis")
	imu = lsm9ds1.make_i2c(1)
	minimums = np.ones(3) * np.inf
	maximums = np.ones(3) * (-np.inf)
	start_time = time.monotonic()
	num_samples = 0
	while time.monotonic() - start_time < duration_seconds:
		mag_data_ready = imu.read_magnetometer_status()
		da_mag = mag_data_ready.data_available
		
		if da_mag:
			reading = imu.mag_values()
			minimums = np.minimum(minimums, reading)
			maximums = np.maximum(maximums, reading)
			num_samples += 1

	print(f"OK. Samples: {num_samples}")
	print("Now point to the North")

	start_time = time.monotonic()

	biases = (minimums + maximums) / 2
	norms = (maximums - minimums) / 2
	time.sleep(north_fix_seconds)
	while True:
		mag_data_ready = imu.read_magnetometer_status()
		da_mag = mag_data_ready.data_available
		
		if da_mag:
			reading = imu.mag_values()
			reading = (np.array(reading) - biases) / norms
			north = mag_to_heading(reading)
			return biases, norms, north
	
	
SAVE_FILE_NAME = "last-calibration.json"


class MagFixer:
	def __init__(self):
		with open(SAVE_FILE_NAME, 'r') as f:
			saved = json.load(f)
		self.biases = np.array(saved["biases"])
		self.norms = np.array(saved["norms"])
		self.north_offset = saved["north"]

	def fix_mag(self, mag):
		return (np.array(mag) - self.biases) / self.norms
	
	def fixed_mag_to_heading(self, mag):
		return (2 * PI) - (mag_to_heading(mag) - self.north_offset) % (2 * PI)
		

def mag_to_heading(mag):
	[mx, my, mz] = mag
	mag = math.sqrt(mx**2 + mz**2)
	theta = math.acos(mz / mag)
	phi = math.acos(mx / mag)
	if phi >= PI/2:
		return theta
	else:
		return 2*PI - theta

if __name__ == '__main__':
	biases, norms, north = calibrate_mag(30.0, 15.0)
	with open(SAVE_FILE_NAME, 'w') as f:
		json.dump({
			"biases": list(biases),
			"norms": list(norms),
			"north": north
		}, f)

