import board
import serial
import adafruit_gps
import lsm9ds1
import os
import time
import threading
import math
from gpiozero import Servo
import lzma
import datetime

PI = math.pi

class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.running = True
		uart = serial.Serial("/dev/ttyS0", baudrate=38400, timeout=10)
		self.start_time = time.monotonic()
		self.gps = adafruit_gps.GPS(uart, debug=False)
		self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
		self.gps.send_command(b'PMTK220,100')
		self.current_lon = 0.0
		self.current_lat = 0.0
		self.current_alt = 0.0
		self.ready = False
		self.file = lzma.open('output-{date:%Y-%m-%d_%H:%M:%S}-gps.txt'.format( date=datetime.datetime.now() ), 'w	b')
		
	def get_current_value(self):
		return (self.current_lat, self.current_lon)

	def get_ready(self):
		return self.ready

	def run(self):
		try:
			self.file.write(f"start={self.start_time}\n".encode('utf-8'))
			while self.running and not self.gps.has_fix:
				print('Waiting for fix...')
				time.sleep(0.5)
				self.gps.update()
			
			self.ready = True
			while self.running:
				if self.gps.update():
					self.current_lat = self.gps.latitude
					self.current_lon = self.gps.longitude
					self.current_alt = self.gps.altitude_m
					
					self.file.write(f"t={time.monotonic()}, lat={self.current_lat}, lon={self.current_lon}, alt={self.current_alt}\n".encode('utf-8'))
					
			self.file.close()
			
		except StopIteration:
			pass

MAG_MX_BIAS = 0.45215
MAG_MX_NORM = 0.20125
MAG_MZ_BIAS = 0.40305
MAG_MZ_NORM = 0.15865
# don't know the y calibration parameters. don't care.
MAG_MY_BIAS = 0.0
MAG_MY_NORM = 1.0

HEADING_BIAS = math.radians(260.0)

NO_DATA = "?"

class ImuPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.running = True
		self.start_time = time.monotonic()
		self.imu = lsm9ds1.make_i2c(1)
		self.current_heading = 0.0
		self.file = lzma.open('output-{date:%Y-%m-%d_%H:%M:%S}-imu.txt'.format( date=datetime.datetime.now() ), 'wb')
		
	def get_current_heading(self):
		return self.current_heading
		
	def run(self):
		try:
			self.file.write(f"start={self.start_time}\n".encode('utf-8'))
			mx = my = mz = temp = acc = gyro = None
			
			while self.running:
				mag_data_ready = self.imu.read_magnetometer_status()
				ag_data_ready = self.imu.read_ag_status()
				
				da_mag = mag_data_ready.data_available
				da_temp, da_gyro, da_acc = (ag_data_ready.temperature_data_available, ag_data_ready.gyroscope_data_available, ag_data_ready.accelerometer_data_available)
				
				if da_mag:
					# in milli gauss
					mx, my, mz = self.imu.mag_values()
					mxc = (mx - MAG_MX_BIAS) / MAG_MX_NORM
					myc = (my - MAG_MY_BIAS) / MAG_MY_NORM
					mzc = (mz - MAG_MZ_BIAS) / MAG_MZ_NORM
					self.current_heading = (ImuPoller.mag_to_heading(mxc, myc, mzc) - HEADING_BIAS) % (2 * PI)
				
				if da_temp or da_gyro or da_acc:
					# temp is in farenheit lol
					# acc is in milli g_0
					# gyro is in milli degree per second
					temp, acc, gyro = self.imu.read_values()

				if da_temp or da_gyro or da_acc or da_mag:
					mag = [mx, my, mz]
					self.file.write(f"t={time.monotonic()}, temp={temp if da_temp else NO_DATA}, acc={acc if da_acc else NO_DATA}, gyro={gyro if da_gyro else NO_DATA}, mag={mag if da_mag else NO_DATA}\n".encode('utf-8'))

			self.file.close()
			
		except StopIteration:
			pass
		
	@staticmethod
	def mag_to_heading(mx, my, mz):
		mag = math.sqrt(mx**2 + mz**2)
		theta = math.acos(mz / mag)
		phi = math.acos(mx / mag)
		if phi >= PI/2:
			return theta
		else:
			return 2*PI - theta

def lat_lon_to_rad(lat_lon_in_deg):
	(lat_deg, lon_deg) = lat_lon_in_deg
	return (math.radians(lat_deg), math.radians(lon_deg))


target_lat_lon = lat_lon_to_rad((29.716897, -95.410912)) # north of greenbriar lot
#target_lat_lon = lat_lon_to_rad((29.714922, -95.410879)) # south of greenbriar lot

gps = GpsPoller()
gps.start()
imu = ImuPoller()
imu.start()
servo_l = Servo(23)
servo_r = Servo(24)
servo_l.max()
servo_r.max()

while not gps.get_ready():
	time.sleep(0.25)

servo_l.mid()
servo_r.mid()


def clockwise_angle_distance(target, current):
	regular_distance = (target - current) % (2 * PI)
	if regular_distance <= PI: return regular_distance
	else: return regular_distance - (2 * PI)
	
def bearing_from_to(from_lat_lon, to_lat_lon):
	(from_lat, from_lon) = from_lat_lon
	(to_lat, to_lon) = to_lat_lon
	return math.atan2(
		math.sin(to_lon - from_lon) * math.cos(to_lat),
		math.cos(from_lat) * math.sin(to_lat) - math.sin(from_lat) * math.cos(to_lat) * math.cos(to_lon - from_lon)
	)
	
while True:
	from_lat_lon = gps.get_current_value()

	# might not work near the meridian
	bearing = bearing_from_to(from_lat_lon, target_lat_lon)
	heading = imu.get_current_heading()
	
	dif_heading = clockwise_angle_distance(bearing, heading)
	
	# dead zone
	# 0.087 rad = 5 degrees
	if dif_heading > 0.087:
		servo_r.value = (PI - dif_heading) * 0.8 / PI
		servo_l.value = 1.0
	elif dif_heading < -0.087:
		servo_l.value = (PI + dif_heading) * 0.8 / PI
		servo_r.value = 1.0
	else:
		servo_l.value = 1.0
		servo_r.value = 1.0
	
'''
try:
	gpsp = GpsPoller()
	gpsp.start()
	prev_lat = 0
	prev_lon = 0
	imu = lsm9ds1.make_i2c(1)
	while True:
		(lat, lon) = gpsp.get_current_value()
		if (lat is None) or (lon is None):
			continue
		try:
			print("polling gps")
			v_y = lat - prev_lat
			prev_lat = lat
			v_x = lon - prev_lon
			prev_lon = lon
			
			v_prime_x = -v_y
			v_prime_y = v_x
			
			r_x = target_lon - lon
			r_y = target_lat - lat
			
			v_prime_dot_r = r_x * v_prime_x + r_y * v_prime_y
			
			v_dot_r = r_x * v_x + r_y * v_y
			
			v_mag = math.sqrt(v_x**2 + v_y**2)
			r_mag = math.sqrt(r_x**2 + r_y**2)
			mag_product = v_mag * r_mag

			if mag_product == 0:
				continue

			abs_angle = math.acos(v_dot_r / mag_product)
			if (mag_product == 0 or abs_angle < (15/180 * math.pi)):
				print("doing nothing")
				servo_l.mid()
				servo_r.mid()
				angle = 0
			else:
				if (v_prime_dot_r <= 0):
					print(f"turning right by {abs_angle * 180 / math.pi}deg")
					angle = abs_angle
					servo_r.min()
					servo_l.mid()
				elif (v_prime_dot_r > 0):
					print(f"turning left by {abs_angle * 180 / math.pi}deg")
					angle = -abs_angle
					servo_l.min()
					servo_r.mid()

			target_heading = None
			while True:
				mag_data_ready = imu.read_magnetometer_status().data_available
				if mag_data_ready:
					mx, my, mz = imu.mag_values()
					mx = (mx + 0.7366) * 4.26167
					mz = (mz + 0.3655) * 5.58036
					heading = mag_to_heading(mx, my, mz)
					if target_heading is None:
						target_heading = (heading + angle) % (2 * math.pi)
					else:
						heading_difference = abs(target_heading - heading)
						heading_difference = min(heading_difference, (2*math.pi) - heading_difference)
						print(f"still turning - heading difference = {heading_difference * 180 / math.pi}deg")
						if heading_difference < (15/180 * math.pi):
							servo_l.mid()
							servo_r.mid()
							break
				time.sleep(0.1)
			print("turn complete")
			time.sleep(2.0)
			time.sleep(0.25)
		except(AttributeError, KeyError):
			pass
		time.sleep(0.25)

except(KeyboardInterrupt, SystemExit):
	print("\nKilling Thread..")
	gpsp.running = False
	gpsp.join()
'''
print("Done.\nExiting.")
