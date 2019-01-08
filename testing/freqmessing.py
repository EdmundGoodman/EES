import RPi.GPIO as GPIO
import time
from time import sleep
import RPi.GPIO as GPIO
import time

PIN = 18
PWMA1 = 6
PWMA2 = 13
PWMB1 = 20
PWMB2 = 21

D1 = 12
D2 = 26


#PWM = 50

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(PIN,GPIO.IN,GPIO.PUD_UP)
GPIO.setup(PWMA1,GPIO.OUT)
GPIO.setup(PWMA2,GPIO.OUT)
GPIO.setup(PWMB1,GPIO.OUT)
GPIO.setup(PWMB2,GPIO.OUT)
GPIO.setup(D1,GPIO.OUT)
GPIO.setup(D2,GPIO.OUT)
freq = int(input("Enter Frequency: "))

p1 = GPIO.PWM(D1,freq)
p2 = GPIO.PWM(D2,freq)
duty = int(input("Enter duty percent: "))

p1.start(duty)
p2.start(duty)
try:
	sleep(10000000)
finally:
	GPIO.cleanup()
