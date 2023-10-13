#region Explanation
    #This code demonstrates how to read data from an 
    #LSM9DS1 sensor, specifically accelerometer, gyroscope, 
    #and magnetometer data, and uses the MagFixer class to 
    #fix and display magnetometer data. The main loop 
    #continuously checks for magnetometer data availability 
    #and reads the data when available.
#endregion

# Import necessary libraries for time, math, LSM9DS1 sensor, and MagFixer class
import time
import math
import lsm9ds1
from mag_calibration import MagFixer

# Class for the LSM9DS1 sensor example
class SimpleExample:
    X_IND = 1
    Y_IND = 2
    Z_IND = 0

    PITCH_IND = 1
    ROLL_IND = 0
    YAW_IND = 2

    def __init__(self):
        # Initialize the LSM9DS1 sensor driver and the MagFixer for magnetometer data
        self.driver = lsm9ds1.make_i2c(1)
        self.mf = MagFixer()

    # Main function for the example
    def main(self):
        try:
            count = 0
            while True:
                # Check if magnetometer data is available
                mag_data_ready = self.driver.read_magnetometer_status().data_available
                if mag_data_ready:
                    self.read_magnetometer()
                    count += 1
                time.sleep(0.05)
        finally:
            # Close the driver when done
            self.driver.close()

    # Function to read accelerometer and gyroscope data
    def read_ag(self):
        temp, acc, gyro = self.driver.read_values()
        print("Temp: %.1f °F" % temp)
        print("Gyro Roll: %.4f, Pitch: %.4f, Yaw: %.4f" % (gyro[SimpleExample.ROLL_IND],
                                                           gyro[SimpleExample.PITCH_IND],
                                                           gyro[SimpleExample.YAW_IND]))
        print("X: %.4f, Y: %.4f, Z: %.4f" % (acc[SimpleExample.X_IND],
                                             acc[SimpleExample.Y_IND],
                                             acc[SimpleExample.Z_IND]))

    # Function to read magnetometer data and apply MagFixer
    def read_magnetometer(self):
        m = self.driver.mag_values()
        m = self.mf.fix_mag(m)
        hdg = math.degrees(self.mf.fixed_mag_to_heading(m))
        (x, y, z) = m
        print(f"{x:.2f}    {y:.2f}    {z:.2f}    heading={hdg:.2f}")

# Entry point of the program
if __name__ == '__main__':
    # Create an instance of the SimpleExample class and call its main method
    SimpleExample().main()





