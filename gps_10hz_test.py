"""
In essence, this code sets up communication with a GPS module, 
configures it, and continuously reads and prints latitude and 
longitude data when a valid GPS fix is obtained.
"""

import time
import board
import serial
import adafruit_gps

# Set up a serial connection to the GPS module on the Raspberry Pi's GPIO serial port (ttyS0) with a baud rate of 9600.
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=30)

# Create a GPS object using the Adafruit GPS library and the UART (serial) connection.
gps = adafruit_gps.GPS(uart, debug=False)

# Configure the GPS module with specific settings using PMTK commands.
# These commands set various parameters related to how the GPS module operates.
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,200')

# Initialize a variable to keep track of the last time data was printed.
last_print = time.monotonic()

# Continuously loop to read and process GPS data.
while True:
    # Check if there's new data from the GPS module.
    if gps.update():
        # Check if the GPS has a valid fix on its location.
        if not gps.has_fix:
            print('Waiting for fix...')
            time.sleep(0.5)
            continue  # If no fix, go back to the beginning of the loop and keep waiting.

        # Print a separator line.
        print('=' * 40)
        # Print latitude and longitude data from the GPS module.
        print(f"Latitude: {gps.latitude:.6f} degrees")
        print(f"Longitude: {gps.longitude:.6f} degrees")
