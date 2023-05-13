import cv2 as cv
import numpy as np
import math
import picamera
import time
from sklearn.cluster import KMeans

class ImageProcessing:
	pastRhoAndThetaValues = [(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]
	right_lines = []
	left_lines = []
	averaged_left = None
	averaged_right = None
	imageToShow = None
	staticMiddlePoint = None
	intersection = None
	staticBottomPoint = None
	distanceBetweenStaticMiddlePointAndTrackLine = None
	camera = None
	kmeans_lines = None
	lines = None
	total = None

	def __init__(self, imagePath):
		self.imagePath = imagePath
		self.image = cv.imread(cv.samples.findFile(imagePath))
		self.camera = picamera.PiCamera()
		self.camera.resolution = (400,300)
		self.camera.exposure_mode="auto"
		self.camera.capture(self.imagePath)



	def captureImage(self):
		self.camera.capture(self.imagePath, use_video_port=True)
		self.image = cv.imread(cv.samples.findFile(self.imagePath))

	def saveShowImage(self):
		cv.imwrite('/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/showImage.jpg', self.imageToShow)

	def BGRtoGrayScale(self):
		self.image = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

	def ROI(self, x1, y1, x2, y2):
		self.image = self.image[x1:y1,x2:y2]

	def prepareDisplayImage(self):
		self.imageToShow = cv.cvtColor(self.image, cv.COLOR_GRAY2BGR)
		cv.circle(self.imageToShow,self.staticMiddlePoint,5,(0,255,0),-1)
		#cv.line(self.imageToShow, self.intersectionCoordinatesOfHoughLines, self.staticBottomPoint, (0, 255, 0), 2)
		for i in self.lines:
			self.drawHoughLinesRaw(i[0][0],i[0][1])
		self.drawHoughLines(self.averaged_right[0],self.averaged_right[1])
		self.drawHoughLines(self.averaged_left[0],self.averaged_left[1])
		self.saveShowImage()



	def houghLineTransform(self):
		lines = cv.HoughLines(self.image, 1, np.pi / 180, 50)
		self.lines = lines
	
		features = []
		for line in lines:
			rho, theta = line[0]
			features.append([rho, theta])

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
				right_side_cluster.append((rho,theta))
			else:
				left_side_cluster.append((rho,theta))

		print(right_side_cluster,"****", left_side_cluster)
		self.averaged_right = np.average(right_side_cluster, axis=0)
		self.averaged_left = np.average(left_side_cluster, axis=0)
		self.total = [self.averaged_left,self.averaged_right]





	def findLineToTrack(self):
		if self.averaged_left[0]>0 and self.averaged_right[0]>0 or self.averaged_left[0]<0 and self.averaged_right[0]<0:
			self.kmeans_lines = np.average(self.total, axis=0)
			self.findDistanceWithRhoThetaPoint(self.kmeans_lines[0], self.kmeans_lines[1])
		else:
			self.findIntersection()
			cv.line(self.imageToShow, self.intersection, (200,300), (0,255,255), 3, cv.LINE_AA,)
			self.findDistanceWithPointPointPoint()


	def findIntersection(self):
		theta1 = self.averaged_right[1]
		theta2 = self.averaged_left[1]
		rho1 = self.averaged_right[0]
		rho2 = self.averaged_left[0]
		a = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
		b = np.array([[rho1], [rho2]])
		intersection = np.linalg.solve(a, b)
		x, y = map(int, intersection)
		self.intersection = (x,y)

	def drawHoughLines(self,rho,theta):
		a = math.cos(theta)
		b = math.sin(theta)
		x0 = a * rho
		y0 = b * rho
		pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
		pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
		cv.line(self.imageToShow, pt1, pt2, (255,0,0), 3, cv.LINE_AA,)

	def drawHoughLinesRaw(self,rho,theta):
		a = math.cos(theta)
		b = math.sin(theta)
		x0 = a * rho
		y0 = b * rho
		pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
		pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
		cv.line(self.imageToShow, pt1, pt2, (0,0,255), 3, cv.LINE_AA,)

	def showImage(self):
		cv.imshow("deneme", self.imageToShow)

	def cannyEdgeDetection(self, threshold1, threshold2):
		self.image = cv.Canny(self.image, threshold1, threshold2, None, 3)

	def setStaticMiddlePoint(self, x, y):
		self.staticMiddlePoint = (x,y)

	def findDistanceWithRhoThetaPoint(self, rho, theta):
		x = self.staticMiddlePoint[0]
		y = self.staticMiddlePoint[1]
		self.distanceBetweenStaticMiddlePointAndTrackLine = abs((x - rho * math.cos(theta)) * math.sin(theta) - (y - rho * math.sin(theta)) * math.cos(theta))
		print("distance between static point and track -> ", self.distanceBetweenStaticMiddlePointAndTrackLine)

	def findDistanceWithPointPointPoint(self):
		p1 = np.array([self.intersection[0], self.intersection[1]])
		p2 = np.array([self.staticBottomPoint[0], self.staticBottomPoint[1]])
		p3 = np.array([self.staticMiddlePoint[0],self.staticMiddlePoint[1]])
		d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
		print("distance between static point and track -> ", d)
		self.distanceBetweenStaticMiddlePointAndTrackLine = d
