import serial
import adafruit_gps
import lsm9ds1
import time
import threading
import math
from gpiozero import Servo
import datetime
from mag_calibration import MagFixer

PI = math.pi

NO_DATA = "?"

class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.running = True
		uart = serial.Serial("/dev/ttyS0", baudrate=38400, timeout=10)
		self.start_time = time.monotonic()
		self.gps = adafruit_gps.GPS(uart, debug=False)
		self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
		self.gps.send_command(b'PMTK220,200')
		self.current_lon = 0.0
		self.current_lat = 0.0
		self.current_alt = 0.0
		self.has_fix = False
		self.file = open('output-{date:%Y-%m-%d_%H:%M:%S}-gps.txt'.format(date=datetime.datetime.now()), 'w', buffering=512)

	def get_current_value(self):
		return (self.current_lat, self.current_lon)

	def get_current_alt(self):
		return self.current_alt

	def run(self):
		try:
			while self.running and not self.gps.has_fix:
				time.sleep(0.5)
				self.gps.update()
			self.has_fix = True
			while self.running:
				if self.gps.update():
					self.current_lat = self.gps.latitude
					self.current_lon = self.gps.longitude
					self.current_alt = self.gps.altitude_m
					self.has_fix = self.gps.has_fix

					self.file.write(f"t={time.monotonic()}, fix={self.has_fix}, lat={self.current_lat}, lon={self.current_lon}, alt={self.current_alt}\n")
		finally:
			self.file.close()

class ImuPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.running = True
		self.start_time = time.monotonic()
		self.imu = lsm9ds1.make_i2c(1)
		self.current_heading = 0.0
		self.current_acc_x = 0.0
		self.current_tilt = 0.0
		self.file = open('output-{date:%Y-%m-%d_%H:%M:%S}-imu.txt'.format( date=datetime.datetime.now() ), 'w', buffering=4096)
		self.magfixer = MagFixer()

	def get_current_heading(self):
		return self.current_heading
	
	def get_current_vertical_acc(self):
		return self.current_acc_x
		
	def get_current_tilt(self):
		return self.current_tilt
	
	def run(self):
		try:
			self.file.write(f"start={self.start_time}\n")
			mag = temp = acc = gyro = None

			while self.running:
				mag_data_ready = self.imu.read_magnetometer_status()
				ag_data_ready = self.imu.read_ag_status()

				da_mag = mag_data_ready.data_available
				da_temp, da_gyro, da_acc = (ag_data_ready.temperature_data_available, ag_data_ready.gyroscope_data_available, ag_data_ready.accelerometer_data_available)

				if da_mag:
					# in milli gauss
					mag = self.imu.mag_values()
					mag = self.magfixer.fix_mag(mag)
					self.current_heading = self.magfixer.fixed_mag_to_heading(mag) % (2 * PI)

				if da_temp or da_gyro or da_acc:
					# temp is in ???
					# acc is in g_0
					# gyro is in degree per second
					temp, acc, gyro = self.imu.read_values()
					self.current_acc_x = acc[0]
					self.current_tilt = compute_tilt(acc)

				if da_temp or da_gyro or da_acc or da_mag:
					self.file.write(f"t={time.monotonic()}, temp={temp if da_temp else NO_DATA}, acc={acc if da_acc else NO_DATA}, gyro={gyro if da_gyro else NO_DATA}, mag={mag if da_mag else NO_DATA}\n")

		finally:
			self.file.close()

def compute_tilt(acc):
	(ax, ay, az) = acc
	acc_m = math.sqrt(ax**2 + ay**2 + az**2)
	if acc_m < 0.001: return 0.0
	# tilt close to zero == pointing downward
	return math.cos(ax / acc_m)

def lat_lon_to_rad(lat_lon_in_deg):
	(lat_deg, lon_deg) = lat_lon_in_deg
	return (math.radians(lat_deg), math.radians(lon_deg))


target_lat_lon = lat_lon_to_rad((29.721116, -95.401288)) # middle of OEDK
#target_lat_lon = lat_lon_to_rad((29.716897, -95.410912)) # north of greenbriar lot
#target_lat_lon = lat_lon_to_rad((29.714922, -95.410879)) # south of greenbriar lot

gps = GpsPoller()
gps.start()
imu = ImuPoller()
imu.start()
servo_l = Servo(23)
servo_r = Servo(24)
servo_l.min()
servo_r.min()

def clockwise_angle_distance(target, current):
	regular_distance = (target - current) % (2 * PI)
	if regular_distance <= PI: return regular_distance
	else: return regular_distance - (2 * PI)

# might not work near the meridian
def bearing_from_to(from_lat_lon, to_lat_lon):
	(from_lat, from_lon) = from_lat_lon
	(to_lat, to_lon) = to_lat_lon
	return math.atan2(
		math.sin(to_lon - from_lon) * math.cos(to_lat),
		math.cos(from_lat) * math.sin(to_lat) - math.sin(from_lat) * math.cos(to_lat) * math.cos(to_lon - from_lon)
	)

EARTH_RADIUS = 6373000.0 # in meters

def distance_between(point1, point2):
	(lat1, lon1) = point1
	(lat2, lon2) = point2
	dlat = lat2 - lat1
	dlon = lon2 - lon1
	a = math.sin(dlat / 2.0)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0)**2
	c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
	return EARTH_RADIUS * c



TURN_POWER = 0.3 / PI

try:
	logfile = open('output-{date:%Y-%m-%d_%H:%M:%S}-main.txt'.format(date=datetime.datetime.now()), 'w', buffering=0)
	logfile.write(f"started at {time.monotonic()}\n")
	while not gps.has_fix:
		print("Waiting for GPS...")
		time.sleep(0.5)
	
	print("GPS Ready")
	logfile.write(f"ready at {time.monotonic()}\n")
	
	servo_l.mid()
	servo_r.mid()
	deployed = False
	while gps.get_current_alt() > 3500 or imu.get_current_vertical_acc() < 0.0:
		pass
	
	print("deployed")
	logfile.write(f"deployed at {time.monotonic()}\n")
	
	prev_distance = distance_between(gps.get_current_value(), target_lat_lon)
	prev_time = time.monotonic()
	num_off_course = 0
	while True:
	
		# 0.5 rad = 30 deg
		if imu.get_current_tilt() > 0.5:
			continue
		from_lat_lon = gps.get_current_value()

		bearing = bearing_from_to(from_lat_lon, target_lat_lon)
		heading = imu.get_current_heading()

		dif_heading = clockwise_angle_distance(bearing, heading)
		
		# dead zone
		# 0.1745 rad = 10 degrees
		if dif_heading > 0.1745:
			servo_r.value = 0.8 - dif_heading * TURN_POWER
			servo_l.value = 1.0
		elif dif_heading < -0.1745:
			servo_l.value = 0.8 + dif_heading * TURN_POWER
			servo_r.value = 1.0
		else:
			servo_l.value = 1.0
			servo_r.value = 1.0
		
		current_time = time.monotonic()
		if current_time - prev_time > 5.0:
			prev_time = current_time
			current_distance = distance_between(from_lat_lon, target_lat_lon)
			if current_distance >= prev_distance:
				num_off_course += 1
				if num_off_course >= 10:
					# Suicide mode
					print("Sprialing down!")
					logfile.write(f"spiraling down at {time.monotonic()}\n")

					servo_l.value = 0.3
					while True:
						time.sleep(1.0)
			else:
				num_off_course = 0
			prev_distance = current_distance


finally:
	print("Stopping threads.")
	logfile.write(f"ending at {time.montonic()}")
	logfile.close()
	gps.running = False
	imu.running = False
	gps.join()
	imu.join()

print("Done.\nExiting.")
