import time
import pigpio


SERVO = 18
pi = pigpio.pi()
try:
    while True:
        pi.set_servo_pulsewidth(SERVO,1500)
        time.sleep(0)
        pi.set_servo_pulsewidth(SERVO,0)
        time.sleep(0)
except KeyboardInterrupt:
    pi.stop()
    exit()
