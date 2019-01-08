import webiopi
from time import sleep
GPIO = webiopi.GPIO

L1 = 6

GPIO.output(L1, GPIO.LOW)

sleep(10)
GPIO.cleanup()

