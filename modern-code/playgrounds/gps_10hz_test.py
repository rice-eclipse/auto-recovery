import time
import board
import serial

import adafruit_gps

uart = serial.Serial("/dev/ttyS0", baudrate=38400, timeout=30)

gps = adafruit_gps.GPS(uart, debug=False)

gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,100')

last_print = time.monotonic()
while True:
    gps.update()

    current = time.monotonic()
    if current - last_print >= 0.1:
        last_print = current
        if not gps.has_fix:
            print('Waiting for fix...')
            continue
        print('=' * 40)  # Print a separator line.
        print('Latitude: {0:.6f} degrees'.format(gps.latitude))
        print('Longitude: {0:.6f} degrees'.format(gps.longitude))

