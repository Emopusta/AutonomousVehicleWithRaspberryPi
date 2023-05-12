import cv2 as cv
import numpy as np
import math
import picamera
import time

class ImageProcessing:
	pastRhoAndThetaValues = [(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]
	right_lines = []
	left_lines = []
	averaged_left = None
	averaged_right = None
	imageToShow = None
	staticMiddlePoint = None
	intersectionCoordinatesOfHoughLines = None
	staticBottomPoint = None
	distanceBetweenStaticMiddlePointAndTrackLine = None
	camera = None
	lines = None
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
		cv.line(self.imageToShow, self.intersectionCoordinatesOfHoughLines, self.staticBottomPoint, (0, 255, 0), 2)
		for i in self.lines:
			self.drawHoughLinesRaw(i[0][0],i[0][1])
		self.drawHoughLines(self.averaged_right[0],self.averaged_right[1])
		self.drawHoughLines(self.averaged_left[0],self.averaged_left[1])
		self.saveShowImage()



	def houghLineTransform(self):
		lines = cv.HoughLines(self.image, 1, np.pi / 180, 50)
		self.lines = lines
		right_flag=False
		left_flag=False
		
		for i in lines:
			slope = i[0][0]
			intercept = i[0][1]
			
			if intercept < 1.39 or intercept > 1.74:
				if intercept<-0.05 or intercept>0.05:
					if slope<200:
						right_flag = True
						self.right_lines.append((slope,intercept))
					else:
						left_flag = True
						self.left_lines.append((slope,intercept))
			else :
				print("yatay")
		if right_flag and left_flag:
			self.averaged_right = np.average(self.right_lines, axis=0)
			self.averaged_left = np.average(self.left_lines, axis=0)
			self.drawHoughLines(self.averaged_right[0],self.averaged_right[1])
			self.drawHoughLines(self.averaged_left[0],self.averaged_left[1])

		elif left_flag:
			self.averaged_left =np.average(self.left_lines, axis=0)
			self.drawHoughLines(self.averaged_left[0],self.averaged_left[1])
			self.averaged_right = [400,0]
			self.drawHoughLines(self.averaged_right[0],self.averaged_right[1])

		elif right_flag:
			self.averaged_right = np.average(self.right_lines, axis=0)
			self.drawHoughLines(self.averaged_right[0],self.averaged_right[1])
			self.averaged_left = [0,0]
			self.drawHoughLines(self.averaged_left[0],self.averaged_left[1])
		else:
			print("naneyi yedin")
		
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

	def findLineToTrack(self):
		theta1 = self.averaged_right[1]
		theta2 = self.averaged_left[1]
		rho1 = self.averaged_right[0]
		rho2 = self.averaged_left[0]
		a = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
		b = np.array([[rho1], [rho2]])
		intersection = np.linalg.solve(a, b)
		x, y = map(int, intersection)
		self.intersectionCoordinatesOfHoughLines = (x,y)
		self.staticBottomPoint = (200,440)

	def findDistanceBetweenStaticMiddlePointAndTrackLine(self):
		p1 = np.array([self.intersectionCoordinatesOfHoughLines[0], self.intersectionCoordinatesOfHoughLines[1]])
		p2 = np.array([self.staticBottomPoint[0], self.staticBottomPoint[1]])
		p3 = np.array([self.staticMiddlePoint[0],self.staticMiddlePoint[1]])
		d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
		print("distance between static point and track -> ", d)
		self.distanceBetweenStaticMiddlePointAndTrackLine = d
