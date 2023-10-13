"""
This code interfaces with an LSM9DS1 sensor via
the I2C interface and continuously reads data from 
the sensor in an infinite loop. It first imports the lsm9ds1 
module for working with the LSM9DS1 sensor. Then, it creates 
an instance of the sensor, specifying the I2C bus to use. 
Inside the loop, it reads the status of the magnetometer and 
the accelerometer/gyroscope, and extracts data availability 
flags for each sensor component. If all required sensor data 
(magnetometer, gyroscope, and accelerometer) is available, 
it proceeds to read magnetic field data in milli gauss (mag), 
as well as temperature, accelerometer (acc), and gyroscope 
(gyr) data. Finally, it prints the sensor data to the console 
in a formatted manner. This code continuously monitors and 
displays data from the LSM9DS1 sensor.
"""

# Import the lsm9ds1 module, working with an LSM9DS1 sensor.
import lsm9ds1

# Create an instance of the LSM9DS1 sensor using the I2C interface on bus 1.
imu = lsm9ds1.make_i2c(1)

# Create an infinite loop to continuously read data from the sensor.
while True:
    # Read the status of the magnetometer and accelerometer/gyroscope.
    mag_data_ready = imu.read_magnetometer_status()
    ag_data_ready = imu.read_ag_status()

    # Extract data availability flags from the sensor readings.
    da_mag = mag_data_ready.data_available
    da_temp, da_gyro, da_acc = (ag_data_ready.temperature_data_available,
                                ag_data_ready.gyroscope_data_available,
                                ag_data_ready.accelerometer_data_available)

    # Check if all required sensor data (magnetometer, gyroscope, accelerometer) is available.
    if da_mag and da_gyro and da_acc:
        # Read magnetic field data in milli gauss.
        mag = imu.mag_values()

        # Read temperature, accelerometer, and gyroscope data.
        _, acc, gyr = imu.read_values()

        # Print the sensor data to the console.
        print(f"mag={mag} acc={acc} gyr={gyr}")
