from gpiozero import Servo
from time import sleep

servo_l = Servo(23)
servo_r = Servo(24)
print("got here")

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
