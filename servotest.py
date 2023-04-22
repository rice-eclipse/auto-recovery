from gpiozero import Servo
from time import sleep
import gpiozero

pin_factory = gpiozero.pins.pigpio.PiGPIOFactory()
servo_l = Servo(24, pin_factory=pin_factory)
servo_r = Servo(23, pin_factory=pin_factory)

try:
	while True:
		servo_l.min(); servo_r.min();
		print("min")
		sleep(8)
		servo_l.mid(); servo_r.mid();
		print("mid")
		sleep(8)
		servo_l.max(); servo_r.max();
		print("max")
		sleep(8)
except KeyboardInterrupt:
	print("Program stopped")
