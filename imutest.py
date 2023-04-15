import lsm9ds1

imu = lsm9ds1.make_i2c(1)

while True:
	mag_data_ready = imu.read_magnetometer_status()
	ag_data_ready = imu.read_ag_status()
	
	da_mag = mag_data_ready.data_available
	da_temp, da_gyro, da_acc = (ag_data_ready.temperature_data_available, ag_data_ready.gyroscope_data_available, ag_data_ready.accelerometer_data_available)
	
	if da_mag and da_gyro and da_acc:
		# in milli gauss
		mag = imu.mag_values()
		_, acc, gyr = imu.read_values()
		print(f"mag={mag} acc={acc} gyr={gyr}")
