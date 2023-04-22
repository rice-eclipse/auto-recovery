import time
import board
import serial

import adafruit_gps

uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)

gps = adafruit_gps.GPS(uart, debug=False)

gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,200')

last_print = time.monotonic()
while True:
	if gps.update():
		if not gps.has_fix:
			print('Waiting for fix...')
			time.sleep(0.5)
			continue
		print('=' * 40)  # Print a separator line.
		print(f"Latitude: {gps.latitude:.6f} degrees")
		print(f"Longitude: {gps.longitude:.6f} degrees")

