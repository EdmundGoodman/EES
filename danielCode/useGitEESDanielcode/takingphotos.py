import picamera
import io
import time
from PIL import Image
cam = picamera.PiCamera()
time.sleep(0.2)

count = 0
def outputs(numPhotos):
	global count
	stream = io.BytesIO()
	for i in range(numPhotos):
		yield stream
		stream.seek(0)
		img = Image.open(stream)
		img.save("{}.jpeg".format(count))
		count += 1
		stream.seek(0)
		stream.truncate(0)
	
cam.capture_sequence(outputs(50), "jpeg", use_video_port=True)
