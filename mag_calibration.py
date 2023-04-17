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
			maximums = np.minimum(maximums, reading)
			num_samples += 1

	print(f"OK. Samples: {num_samples}")
	print("Now point to the North")

	start_time = time.monotonic()

	headings = []
	while time.monotonic() - start_time < north_fix_seconds:
		mag_data_ready = imu.read_magnetometer_status()
		da_mag = mag_data_ready.data_available
		
		if da_mag:
			reading = imu.mag_values()
			h = mag_to_heading(reading)
			headings.append(h)

	return minimums, maximums, np.median(headings)
	
SAVE_FILE_NAME = "last-calibration.json"

if __name__ == '__main__':
	minimums, maximums, north = calibrate_mag(30.0)
	with open(SAVE_FILE_NAME, 'w') as f:
		json.dump({
			"mins": list(minimums),
			"maxs": list(maximums),
			"north": north
		}, f)

class MagFixer:
	def __init__(self):
		with open(SAVE_FILE_NAME, 'r') as f:
			saved = json.load(f)
		mins = np.array(saved["mins"])
		maxs = np.array(saved["maxs"])
		self.biases = (maxs + mins) / 2
		self.norms = (maxs - mins) / 2
		self.north_offset = saved["north"]

	def fix_mag(self, mag):
		return (mag - self.biases) / self.norms
	
	def fixed_mag_to_heading(self, mag):
		return mag_to_heading(mag) - self.north_offset
		

def mag_to_heading(mag):
	[mx, my, mz] = mag
	mag = math.sqrt(mx**2 + mz**2)
	theta = math.acos(mz / mag)
	phi = math.acos(mx / mag)
	if phi >= PI/2:
		return theta
	else:
		return 2*PI - theta

