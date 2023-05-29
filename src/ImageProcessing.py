import cv2 as cv
import numpy as np
import math
import picamera
import picamera.array
import time
from sklearn.cluster import KMeans

class ImageProcessing:

	lines = []
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



	def __init__(self):

		self.camera = picamera.PiCamera()
		self.stream = picamera.array.PiRGBArray(self.camera)
		self.camera.resolution = (320,240)
		self.camera.brightness = 50
		self.camera.framerate=32
		self.camera.capture("/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/deneme.jpg")




	def CaptureImage(self):

		self.camera.capture(self.stream, 'bgr', use_video_port = True)
		self.image = self.stream.array
		self.imageToShow = self.image
		self.stream.seek(0)
		self.stream.truncate()

		self.SaveImage(self.image, "image")


	def GaussFilter(self):

		self.image_gauss = cv.blur(self.image, (5,5))
		self.SaveImage(self.image_gauss, "image_gauss")

	def HSVFilter(self):
		lower = np.array([0,0,150])
		upper = np.array([200,30,255])

		hsv = cv.cvtColor(self.image_gauss, cv.COLOR_BGR2HSV)
		self.image_hsv = cv.inRange(hsv, lower, upper)

		self.SaveImage(self.image_hsv, "image_hsv")


	def CannyEdgeDetection(self, threshold1 = 60, threshold2 = 150):
		self.image_canny = cv.Canny(self.image_hsv,threshold1, threshold2)
		self.SaveImage(self.image_canny, "image_canny")

	def LaneDetection(self, threshold=50):
		lines = cv.HoughLines(self.image_canny, 1, np.pi / 180, threshold)

		self.lines = lines

		features = []
		counter = 0
		unclustered_index=[]
		for line in lines:
			rho, theta = line[0]

			if theta <1.20 or theta>1.99: 
				if theta<-0.1 or theta>0.1:
					features.append([rho, theta])
				else:
					unclustered_index.append(counter)
					print("unclustered -> rho => ",rho, " theta = > ", theta, " counter = ", counter)
			else:
				unclustered_index.append(counter)
				print("unclustered -> rho => ",rho, " theta = > ", theta, "counter = ", counter)
			counter+=1

		lines = np.delete(lines, unclustered_index, 0)

		features = np.array(features)
		features[:, 0] = features[:, 0] / np.max(features[:, 0])
		features[:, 1] = features[:, 1] / np.max(features[:, 1])
		
		kmeans = KMeans(n_clusters=2)
		kmeans.fit(features)
		
		labels = kmeans.labels_
		
		right_side_cluster = []
		left_side_cluster = []
		
		for i, line in enumerate(lines):
			rho, theta = line[0]

			if labels[i]==0:
				self.DrawHoughLines(rho,theta,(0,255,0))
				right_side_cluster.append((rho, theta))
			else:
				self.DrawHoughLines(rho,theta,(0,0,255))
				left_side_cluster.append((rho, theta))


		self.averaged_right = np.average(right_side_cluster, axis=0)
		self.averaged_left = np.average(left_side_cluster, axis=0)
		self.total = [self.averaged_left,self.averaged_right]
		self.DrawHoughLines(self.averaged_right[0],self.averaged_right[1],(255,0,255))
		self.DrawHoughLines(self.averaged_left[0],self.averaged_left[1],(0,255,255))
		self.SaveImage(self.imageToShow, "image_to_show")






	def DrawHoughLines(self,rho,theta, color):
		a = math.cos(theta)
		b = math.sin(theta)
		x0 = a * rho
		y0 = b * rho
		pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
		pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
		cv.line(self.imageToShow, pt1, pt2, color, 3, cv.LINE_AA,)



	def SaveImage(self, image, imageName):
		cv.imwrite(f"/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/Images/{imageName}.jpg", image)

