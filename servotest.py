#region Explanation
    #This code continuously moves two servo motors
    #between their minimum, middle, and maximum positions 
    #in a loop, with pauses between each movement. 
    #It stops when the program is interrupted by a keyboard
    #input (Ctrl+C) and prints "Program stopped" when that happens.
#endregion

from gpiozero import Servo
from time import sleep
import gpiozero

# Initialize a pin factory for GPIO control using pigpio
pin_factory = gpiozero.pins.pigpio.PiGPIOFactory()

# Create two servo motor objects, one for the left (connected to GPIO pin 24) and one for the right (connected to GPIO pin 23)
servo_l = Servo(24, pin_factory=pin_factory)
servo_r = Servo(23, pin_factory=pin_factory)

try:
    while True:
        # Set both servo motors to their minimum positions (assuming -90-degree position)
        servo_l.min()
        servo_r.min()
        print("min")
        # Pause the program for 8 seconds
        sleep(8)

        # Set both servo motors to their middle positions (assuming 0-degree position)
        servo_l.mid()
        servo_r.mid()
        print("mid")
        # Pause the program for 8 seconds
        sleep(8)

        # Set both servo motors to their maximum positions (assuming 90-degree position)
        servo_l.max()
        servo_r.max()
        print("max")
        # Pause the program for 8 seconds
        sleep(8)

except KeyboardInterrupt:
    print("Program stopped")




