import RPi.GPIO as GPIO
GPIO.setwarnings(False)
motor1 = 6
motor2 = 13

GPIO.setmode(GPIO.BCM)

GPIO.setup(motor1,GPIO.OUT)
GPIO.setup(motor2,GPIO.OUT)

try:
	GPIO.output(motor1, GPIO.HIGH)
	GPIO.output(motor2, GPIO.HIGH)
finally:
	GPIO.cleanup()
