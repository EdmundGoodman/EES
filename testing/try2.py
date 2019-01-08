from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
L1 = 6
GPIO.setup(L1, GPIO.OUT)
GPIO.output(L1, GPIO.LOW)



sleep(10)
GPIO.cleanup()
