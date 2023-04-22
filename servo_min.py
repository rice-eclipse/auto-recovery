from gpiozero import Servo
import time

print("left")
servo_l = Servo(24)
time.sleep(5)
print("right")
servo_r = Servo(23)
print("retracting both")
servo_l.min()
servo_r.min()
time.sleep(8)
