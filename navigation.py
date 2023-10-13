#region Explanation
    # This code is a Python script designed for a Raspberry
    # Pi-based navigation system. It combines GPS and IMU 
    # (Inertial Measurement Unit) data to control servo motors
    # for navigation. It first sets up two threads, one for 
    # polling GPS data and another for IMU data. The GPS 
    # thread initializes and configures a GPS module, waits 
    # for a GPS fix, and continuously logs GPS data to a file.
    # The IMU thread reads data from an LSM9DS1 IMU sensor,
    # including accelerometer, gyroscope, and magnetometer data,
    # and logs this data as well. The main part of the code 
    # calculates the heading angle, computes the difference
    # between the desired heading and the current heading,
    # and adjusts servo motors accordingly to steer a vehicle.
    # It also includes functions for angle distance calculation,
    #     bearing calculation between two points, and distance
    # calculation between GPS coordinates. Overall, this code
    # integrates sensor data to implement basic navigation and
    # steering control
#endregion

import serial
import adafruit_gps
import lsm9ds1
import time
#parallel gps, imu, servo
import threading
import math
import gpiozero
from gpiozero import Servo
import datetime
from collections import deque
from mag_calibration import MagFixer

# Define constants
PI = math.pi
NO_DATA = "?"

# Create a class for GPS data polling
class GpsPoller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        # Initialize the GPS UART communication
        uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
        self.start_time = time.monotonic()
        self.gps = adafruit_gps.GPS(uart, debug=False)
        # Configure the GPS settings
        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.gps.send_command(b'PMTK220,200')
        self.current_lon = 0.0
        self.current_lat = 0.0
        self.current_alt = 0.0
        self.has_fix = False
        # Create a file for GPS data logging
        self.file = open('output-{date:%Y-%m-%d_%H:%M:%S}-gps.txt'.format(date=datetime.datetime.now()), 'w', buffering=512)

    def get_current_value(self):
        return (self.current_lat, self.current_lon)

    def get_current_alt(self):
        return self.current_alt

    def run(self):
        try:
            # Wait for a GPS fix
            while self.running and not self.gps.has_fix:
                time.sleep(0.5)
                self.gps.update()
            self.has_fix = True
            # Continuously update GPS data
            while self.running:
                if self.gps.update():
                    self.current_lat = self.gps.latitude
                    self.current_lon = self.gps.longitude
                    self.current_alt = self.gps.altitude_m
                    self.has_fix = self.gps.has_fix

                    # Log GPS data to a file
                    self.file.write(f"t={time.monotonic()}, fix={self.has_fix}, lat={self.current_lat}, lon={self.current_lon}, alt={self.current_alt}\n")
        finally:
            self.file.close()

# Create a class for IMU (Inertial Measurement Unit) data polling
class ImuPoller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.start_time = time.monotonic()
        self.imu = lsm9ds1.make_i2c(1)
        self.current_heading = 0.0
        self.current_acc_x = -1.0
        self.current_tilt = 0.0
        # Create a file for IMU data logging
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
                # Check if new magnetometer, temperature, accelerometer, or gyroscope data is available
                mag_data_ready = self.imu.read_magnetometer_status()
                ag_data_ready = self.imu.read_ag_status()

                da_mag = mag_data_ready.data_available
                da_temp, da_gyro, da_acc = (ag_data_ready.temperature_data_available, ag_data_ready.gyroscope_data_available, ag_data_ready.accelerometer_data_available)

                if da_mag:
                    mag = self.imu.mag_values()
                    mag = self.magfixer.fix_mag(mag)
                    self.current_heading = self.magfixer.fixed_mag_to_heading(mag)

                if da_temp or da_gyro or da_acc:
                    # temp is in ???
                    # acc is in g_0
                    # gyro is in degree per second
                    temp, acc, gyro = self.imu.read_values()
                    if da_acc:
                        self.current_acc_x = acc[1]
                        self.current_tilt = compute_tilt(acc)

                if da_temp or da_gyro or da_acc or da_mag:
                    # Log IMU data to a file
                    self.file.write(f"t={time.monotonic()}, temp={temp if da_temp else NO_DATA}, acc={acc if da_acc else NO_DATA}, gyro={gyro if da_gyro else NO_DATA}, mag={mag if da_mag else NO_DATA}\n")
        finally:
            self.file.close()
    # Function to compute tilt based on accelerometer data
    def compute_tilt(acc):
        (az, ax, ay) = acc
        acc_m = math.sqrt(ax**2 + ay**2 + az**2)
        if acc_m < 0.001:
            return 0.0
        # Tilt close to zero means pointing downward
        return math.acos(ax / acc_m)

    # Function to convert latitude and longitude from degrees to radians
    def lat_lon_to_rad(lat_lon_in_deg):
        (lat_deg, lon_deg) = lat_lon_in_deg
        return (math.radians(lat_deg), math.radians(lon_deg))

    # Define the target latitude and longitude in radians
    target_lat_lon = lat_lon_to_rad((29.223328, -95.094566))  # Example: Southsite

    # Create instances of GPS and IMU pollers and start their threads
    gps = GpsPoller()
    gps.start()
    imu = ImuPoller()
    imu.start()

    # Function to calculate clockwise angle distance between two angles
    def clockwise_angle_distance(target, current):
        regular_distance = (target - current) % (2 * PI)
        if regular_distance <= PI:
            return regular_distance
        else:
            return regular_distance - (2 * PI)

    # Function to calculate bearing from one point to another
    def bearing_from_to(from_lat_lon, to_lat_lon):
        (from_lat, from_lon) = from_lat_lon
        (to_lat, to_lon) = to_lat_lon
        return math.atan2(
            math.sin(to_lon - from_lon) * math.cos(to_lat),
            math.cos(from_lat) * math.sin(to_lat) - math.sin(from_lat) * math.cos(to_lat) * math.cos(to_lon - from_lon)
        )

    # Earth radius in meters
    EARTH_RADIUS = 6373000.0

    # Function to calculate the distance between two points on Earth
    def distance_between(point1, point2):
        (lat1, lon1) = point1
        (lat2, lon2) = point2
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat / 2.0)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0)**2
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return EARTH_RADIUS * c

    # Constants for turning
    TURN_POWER = 0.5 / PI

    # Create a logfile for main program
    logfile = open('output-{date:%Y-%m-%d_%H:%M:%S}-main.txt'.format(date=datetime.datetime.now()), 'w', buffering=16)

    try:
        logfile.write(f"started at {time.monotonic()}\n")
        logfile.write(f"ready at {time.monotonic()}\n")

        # Initialize a deque for gravity values and compute the sum
        gravs = deque([-1.0] * 40)
        gravs_sum = -40.0
        while gravs_sum < 0.0:
            grav_now = imu.get_current_vertical_acc()
            gravs.append(grav_now)
            gravs_sum += grav_now - gravs.popleft()
            time.sleep(0.05)

        print("deployed")
        logfile.write(f"deployed at {time.monotonic()}\n")
        
        # Initialize servo motors
        pin_factory = gpiozero.pins.pigpio.PiGPIOFactory()
        servo_l = Servo(24, pin_factory=pin_factory)
        servo_r = Servo(23, pin_factory=pin_factory)
        servo_l.min()
        servo_r.min()

        prev_time = time.monotonic()
        num_off_course = 0
        while True:

            # 0.5 rad = 30 degrees, check for excessive tilt
            if imu.get_current_tilt() > 0.35:
                print("too much tilt. ignoring")
                continue

            bearing = math.radians(90)
            heading = imu.get_current_heading()

            # Calculate the difference in heading angles
            dif_heading = clockwise_angle_distance(bearing, heading)

            # Dead zone, 0.1745 rad = 10 degrees
            if dif_heading > 0.35:
                # Turn right
                servo_r.value = -1.0
                servo_l.value = -1.0 + dif_heading * TURN_POWER
            elif dif_heading < -0.35:
                # Turn left
                servo_l.value = -1.0
                servo_r.value = -1.0 - dif_heading * TURN_POWER
            else:
                servo_l.value = -1.0
                servo_r.value = -1.0

            current_time = time.monotonic()

    finally:
        print("Stopping threads.")
        logfile.write(f"ending at {time.monotonic()}")
        logfile.close()
        gps.running = False
        imu.running = False
        gps.join()
        imu.join()

    print("Done.\nExiting.")
