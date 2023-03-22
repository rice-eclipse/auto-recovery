import board
import serial
import adafruit_gps
import lsm9ds1
import os
import time
import threading
import math
from gpiozero import Servo


class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		uart = serial.Serial("/dev/ttyS0", baudrate=38400, timeout=30)
		self.running = True
		self.last_print = time.monotonic()
		self.gps = adafruit_gps.GPS(uart, debug=False)
		self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
		self.gps.send_command(b'PMTK220,100')
		self.current_lon = None
		self.current_lat = None

	def get_current_value(self):
		return (self.current_lat, self.current_lon)

	def run(self):
		try:
			while self.running:
				self.gps.update()
				current_time = time.monotonic()
				if current_time - self.last_print >= 0.1:
					self.last_print = current_time
					if not self.gps.has_fix:
						print('Waiting for fix...')
						continue
					
					self.current_lat = self.gps.latitude
					self.current_lon = self.gps.longitude

		except StopIteration:
			pass

servo_l = Servo(23)
servo_r = Servo(24)
servo_l.max()
servo_r.max()

time.sleep(2.0)

servo_l.mid()
servo_r.mid()

#target_lat = 29.720558946001436
#target_lon = -95.40286318177948
target_lat = 0
target_lon = 0
target_lat, target_lon = (29.716897, -95.410912) # north of greenbriar lot
target_lat, target_lon = (29.714922, -95.410879) # south of greenbriar lot
#target_lat = 29.72079227210781
#target_lon = -95.40199533786942

def clockwise_distance(target, current):
	regular_distance = target - current
	if regular_distance >= 0: return regular_distance
	else: return (2 * math.pi) + regular_distance

def mag_to_heading(mx, my, mz):
	mag = math.sqrt(mx**2 + mz**2)
	theta = math.acos(mz / mag)
	phi = math.acos(mx / mag)
	#theta = math.acos(bound_to_range(mz))
	#phi = math.acos(bound_to_range(mx))
	if phi >= math.pi/2:
		return theta
	else:
		return 2*math.pi - theta

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

print("Done.\nExiting.")
