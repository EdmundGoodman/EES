from pynput.keyboard import Key, Listener
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
for i in range(14,20):
	GPIO.setup(i,GPIO.OUT,initial = 1)
GPIO.setup(8,GPIO.OUT,initial = 1)
GPIO.setup(11, GPIO.OUT, initial = 1)

def on_press(key):
	if key == Key.down:
		GPIO.output(16, GPIO.HIGH)
		GPIO.output(19, GPIO.HIGH)
		GPIO.output(14, GPIO.HIGH)
		GPIO.output(15, GPIO.HIGH)
		GPIO.output(8, GPIO.HIGH)
		GPIO.output(11, GPIO.HIGH)
		GPIO.output(16, GPIO.LOW)
		GPIO.output(19, GPIO.LOW)
		
	if key == Key.up:
		GPIO.output(16, GPIO.HIGH)
		GPIO.output(19,GPIO.HIGH)
		GPIO.output(14, GPIO.LOW)
		GPIO.output(15,GPIO.LOW)
		GPIO.output(8, GPIO.LOW)
		GPIO.output(11, GPIO.LOW)
		GPIO.output(16, GPIO.LOW)
		GPIO.output(19,GPIO.LOW)
	if key == Key.right:
		GPIO.output(16,GPIO.HIGH)
		GPIO.output(19,GPIO.HIGH)
		GPIO.output(8, GPIO.LOW)#LOW to HIGH 14 TO 17
		GPIO.output(11,GPIO.LOW)# LOW to HIGH 15 TO 18
		GPIO.output(14, GPIO.HIGH)
		GPIO.output(15, GPIO.HIGH)
		GPIO.output(16, GPIO.LOW)
		GPIO.output(19, GPIO.LOW)
	if key == Key.left:
		GPIO.output(16,GPIO.HIGH)
		GPIO.output(19,GPIO.HIGH)
		GPIO.output(8, GPIO.HIGH)
		GPIO.output(11,GPIO.HIGH)
		GPIO.output(14, GPIO.LOW)
		GPIO.output(15, GPIO.LOW)
		GPIO.output(16, GPIO.LOW)
		GPIO.output(19, GPIO.LOW)

def on_release(key):
	for i in range(14,20):
		GPIO.output(i,GPIO.HIGH)
	GPIO.output(8, GPIO.HIGH)
	GPIO.output(11, GPIO.HIGH)
	if key == Key.esc:
		# Stop listener
 		return False

# Collect events until released
try:
	with Listener(
		on_press=on_press,
		on_release=on_release) as listener:
			listener.join()
finally:
	GPIO.cleanup()
