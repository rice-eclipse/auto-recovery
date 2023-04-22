import gpiozero
from gpiozero import Servo
import time

pin_factory = gpiozero.pins.pigpio.PiGPIOFactory()
servo_l = Servo(24, pin_factory=pin_factory)
servo_r = Servo(23, pin_factory=pin_factory)
servo_l.mid()
servo_r.mid()

print("left")
servo_l.min()
time.sleep(5)
print("right")
servo_r.min()
time.sleep(10)
