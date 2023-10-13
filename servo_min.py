#region Explanation
    # This Python code utilizes the gpiozero library to 
    # control two servo motors connected to a Raspberry 
    # Pi using the pigpio backend. It first initializes 
    # the GPIO pins and creates servo motor objects for 
    # both left and right servos. The code then sets both
    # servos to their middle positions, pauses for 5 seconds, 
    # moves them to their minimum positions (left), waits for 
    # an additional 5 seconds, prints "right" to the console, 
    # and subsequently moves both servos to their minimum positions
    # (right), where it waits for 10 seconds. 
#endregion

# Import necessary libraries for GPIO control and servo motor
import gpiozero
from gpiozero import Servo
import time

# Initialize a pin factory for GPIO control using pigpio
pin_factory = gpiozero.pins.pigpio.PiGPIOFactory()

# Create two servo motor objects, one for the left (connected to GPIO pin 24) and one for the right (connected to GPIO pin 23)
servo_l = Servo(24, pin_factory=pin_factory)
servo_r = Servo(23, pin_factory=pin_factory)

# Set both servo motors to their middle positions (assuming 0-degree position)
servo_l.mid()
servo_r.mid()

# Pause the program for 5 seconds
time.sleep(5)

# Print "left" to the console
print("left")

# Move the left servo motor to its minimum position (assuming -90-degree position)
servo_l.min()

# Pause the program for 5 seconds
time.sleep(5)

# Print "right" to the console
print("right")

# Move the right servo motor to its minimum position (assuming -90-degree position)
servo_r.min()

# Pause the program for 10 seconds
time.sleep(10)
