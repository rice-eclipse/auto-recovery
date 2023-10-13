"""
    This code callibrates a magnetometer sensor and using
    the calibrated data to fix and calculate the heading
    based on magnetometer readings. It utilizes the 
    lsm9ds1 library for sensor data and numpy for numerical 
    operations. The calibration process involves collecting 
    magnetometer data while rotating the sensor along the 
    X, Y, and Z axes and then pointing it to the North direction. 
    The collected data is used to calculate biases and norms for 
    calibration. After calibration, the MagFixer class is defined, 
    which can fix raw magnetometer data by applying the calibration 
    and calculate the heading direction. The calibration data is saved 
    to a JSON file for future use. The script provides an entry point for 
    performing magnetometer calibration and saving the calibration data.
"""

import lsm9ds1
import numpy as np
import time
import json
import math

# Define the value of pi
PI = math.pi

# Function to calibrate the magnetometer
def calibrate_mag(duration_seconds: float, north_fix_seconds: float):
    print("rotate along X, Y, Z axis")
    imu = lsm9ds1.make_i2c(1)
    minimums = np.ones(3) * np.inf
    maximums = np.ones(3) * (-np.inf)
    start_time = time.monotonic()
    num_samples = 0
    
    # Collect magnetometer data for calibration
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

    # Calculate biases and norms for calibration
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

# Define a filename to save calibration data
SAVE_FILE_NAME = "last-calibration.json"

# Class to handle magnetometer data after calibration
class MagFixer:
    def __init__(self):
        with open(SAVE_FILE_NAME, 'r') as f:
            saved = json.load(f)
        self.biases = np.array(saved["biases"])
        self.norms = np.array(saved["norms"])
        self.north_offset = saved["north"]

    # Function to apply the calibration to magnetometer data
    def fix_mag(self, mag):
        return (np.array(mag) - self.biases) / self.norms
    
    # Function to calculate the heading after calibration
    def fixed_mag_to_heading(self, mag):
        return (2 * PI) - (mag_to_heading(mag) - self.north_offset) % (2 * PI)

# Function to convert magnetometer data to heading
def mag_to_heading(mag):
    [mx, my, mz] = mag
    mag = math.sqrt(mx**2 + mz**2)
    theta = math.acos(mz / mag)
    phi = math.acos(mx / mag)
    if phi >= PI/2:
        return theta
    else:
        return 2*PI - theta

# Entry point of the program
if __name__ == '__main__':
    # Calibrate the magnetometer and save calibration data to a file
    biases, norms, north = calibrate_mag(30.0, 15.0)
    with open(SAVE_FILE_NAME, 'w') as f:
        json.dump({
            "biases": list(biases),
            "norms": list(norms),
            "north": north
        }, f)
