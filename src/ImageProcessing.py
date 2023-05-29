import math
import picamera
import picamera.array
import time
from sklearn.cluster import KMeans

class ImageProcessing:

	right_lines = []
	left_lines = []
	averaged_left = None
	averaged_right = None
	imageToShow = None

	image = None
	image_gauss = None
	image_hsv = None
	image_canny = None
	image_hough = None

	PDError = 0

	staticMiddlePoint = None
	intersection = None
	staticBottomPoint = None

	camera = None



	def __init__(self, imagePath):

		self.camera = picamera.PiCamera()
		self.stream = picamera.array.PiRGBArray(self.camera)
		self.camera.resolution = (320,240)
		self.camera.brightness = 65
		self.camera.framerate=32
		#self.camera.capture("/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/deneme.jpg")




	def CaptureImage(self):

		self.camera.capture(self.stream, 'bgr', use_video_port = True)
		self.image = self.stream.array
		self.imageToShow = self.image
		self.stream.seek(0)
		self.stream.truncate()

		self.SaveImage(self.image, "image")


	def gaussFilter(self):

		self.image_gauss = cv.blur(self.image_right, (5,5))
		self.saveImage(self.image_gauss, "image_gauss")

	def HSVFilter(self):
		lower = np.array([0,0,150])
		upper = np.array([180,30,255])

		hsv = cv.cvtColor(self.image_gauss, cv.COLOR_BGR2HSV)
		self.image_hsv = cv.inRange(hsv, lower, upper)

		self.saveImage(self.image_hsv, "image_hsv")


	def cannyEdgeDetection(self, threshold1 = 60, threshold2 = 150):
		self.image_canny = cv.Canny(self.image_hsv,threshold1, threshold2)
		self.saveImage(self.image_canny, "image_canny")




	def saveImage(self, image, imageName):
		cv.imwrite(f"/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/Images/{imageName}.jpg", image)

