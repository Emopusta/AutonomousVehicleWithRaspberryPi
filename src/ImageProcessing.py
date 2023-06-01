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

	imageToShow = None
	image = None
	image_left = None
	image_right = None
	image_left_gauss = None
	image_right_gauss = None
	image_left_hsv = None
	image_right_hsv = None
	image_left_canny = None
	image_right_canny = None
	image_left_hough = None
	image_right_hough = None

	left_point_upper = None
	left_point_lower = None
	right_point_upper = None
	right_point_lower = None

	left_point_upper_intersection = None
	right_point_upper_intersection = None
	left_point_lower_intersection = None
	right_point_lower_intersection = None

	mid_point_upper = None
	mid_point_lower = None


	PDError = 0

	staticMiddlePoint = None
	intersection = None
	staticBottomPoint = None

	camera = None



	def __init__(self):

		self.camera = picamera.PiCamera()
		self.stream = picamera.array.PiRGBArray(self.camera)
		self.camera.resolution = (320,240)
		self.camera.brightness = 72
		self.camera.framerate=32
		self.camera.capture("/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/deneme.jpg")




	def CaptureImage(self):
		startTime = time.time()

		self.camera.capture(self.stream, 'bgr', use_video_port = True)
		self.image = self.stream.array
		self.imageToShow = self.image
		self.stream.seek(0)
		self.stream.truncate()

		self.camera.capture(self.stream, 'bgr', use_video_port = True)
		self.image_left = self.stream.array
		cv.circle(self.image_left, (320,120), 160, (0,0,255), -1)
		cv.circle(self.image_left, (160,0), 125, (0,0,255), -1)
		cv.circle(self.image_left, (0,0), 120, (0,0,225), -1)
		cv.line(self.image_left, (0,0),(0,240), (0,0,255), 5)
		self.stream.seek(0)
		self.stream.truncate()

		self.camera.capture(self.stream, 'bgr', use_video_port = True)
		self.image_right = self.stream.array
		cv.circle(self.image_right, (0,120), 160, (0,0,255), -1)
		cv.circle(self.image_right, (160,0), 125, (0,0,255), -1)
		cv.circle(self.image_right, (320,0), 120, (0,0,225), -1)
		cv.line(self.image_right, (320,0),(320,240), (0,0,255), 5)
		self.stream.seek(0)
		self.stream.truncate()

		endTime = time.time()
		print("Image capture cycle time = ", endTime - startTime)

		self.SaveImage(self.image, "image")
		self.SaveImage(self.image_left, "image_left")
		self.SaveImage(self.image_right, "image_right")



	def GaussFilter(self):

		self.image_left_gauss = cv.blur(self.image_left, (5,5))
		self.image_right_gauss = cv.blur(self.image_right, (5,5))

		self.SaveImage(self.image_left_gauss, "image_left_gauss")
		self.SaveImage(self.image_right_gauss, "image_right_gauss")


	def HSVFilter(self):
		lower = np.array([0,0,150])
		upper = np.array([170,30,255])

		hsv_left = cv.cvtColor(self.image_left_gauss, cv.COLOR_BGR2HSV)
		self.image_left_hsv = cv.inRange(hsv_left, lower, upper)

		hsv_right = cv.cvtColor(self.image_right_gauss, cv.COLOR_BGR2HSV)
		self.image_right_hsv = cv.inRange(hsv_right, lower, upper)

		self.SaveImage(self.image_left_hsv, "image_left_hsv")
		self.SaveImage(self.image_right_hsv, "image_right_hsv")



	def CannyEdgeDetection(self, threshold1 = 60, threshold2 = 150):

		self.image_left_canny = cv.Canny(self.image_left_hsv,threshold1, threshold2)
		self.image_right_canny = cv.Canny(self.image_right_hsv,threshold1, threshold2)

		self.SaveImage(self.image_left_canny, "image_left_canny")
		self.SaveImage(self.image_right_canny, "image_right_canny")


	def LaneDetection(self, threshold=50):

		self.left_lines =  cv.HoughLines(self.image_left_canny, 1, np.pi/180,threshold)
		self.right_lines =  cv.HoughLines(self.image_right_canny, 1, np.pi/180,threshold)

		print(type(self.left_lines), "*-*-*-*-*", type(self.right_lines))
		
		
		if self.left_lines is not  None and self.right_lines is not None:
			print("problem yok")
			pass


		elif self.left_lines is None and self.right_lines is None:
			self.left_lines= np.array([[[1,np.pi*2+np.pi*2/27]]])
			self.right_lines= np.array([[[319,np.pi*2-np.pi*2/27]]])

		elif self.left_lines is None:
			print("sol taraf yok")
			x = np.mean(self.right_lines, axis = 0)
			r,t = x[0]
			self.left_lines = np.array([[[r,t]]])

		elif self.right_lines is None:
			print("sag taraf yok")
			x = np.mean(self.left_lines, axis = 0)
			r,t = x[0]
			self.right_lines = np.array([[[r,t]]])
		else :
			print("screwed")


		left_features = []
		left_unchosen_index = []
		left_counter = 0

		right_features= []
		right_unchosen_index = []
		right_counter = 0

		print("left => ")
		for i in self.left_lines:
			rho,theta = i[0]
			temp = float(theta)%np.pi
			if -0.1<temp<0.1 or temp>3.01:
				left_unchosen_index.append(left_counter)
				print("not drawn : rho : ", rho , ", theta :", theta)
			else:
				left_features.append([[rho, theta]])
				self.DrawHoughLines(i[0][0],i[0][1], (255,0,0))
			print(i)
			left_counter+=1

		if len(left_features)!=0:
			self.left_lines = np.array(left_features)
		else:
			self.left_lines= np.array([[[1,np.pi*2+np.pi*2/27]]])

		print("right=>")
		for i in self.right_lines:
			rho,theta = i[0]
			temp = float(theta)%np.pi
			if -0.1<temp<0.1 or temp>3.01:
				right_unchosen_index.append(right_counter)
				print("not drawn : rho : ", rho , ", theta :", theta)
			else:
				right_features.append([[rho, theta]])
				self.DrawHoughLines(i[0][0],i[0][1], (255,200,200))
			print(i)
			right_counter+=1

		if len(right_features)!=0:
			self.right_lines = np.array(right_features)
		else:
			self.right_lines= np.array([[[1,np.pi*2+np.pi*2/27]]])


		print("--------------------------> left ", self.left_lines)
		print("--------------------------> ", self.right_lines)
		self.averaged_left = np.mean(self.left_lines, axis = 0)
		self.averaged_right = np.mean(self.right_lines, axis = 0)
		print("************************", self.averaged_left, self.averaged_right)
		self.DrawHoughLines(self.averaged_left[0][0],self.averaged_left[0][1],(255,0,0))
		self.DrawHoughLines(self.averaged_right[0][0],self.averaged_right[0][1],(0,255,0))

		cv.line(self.image, (-500,180), (500,180),(60,20,100),7)
		cv.line(self.image, (-500,50), (500,50),(60,20,100),7)
		print("--------",self.averaged_left, "-----", self.averaged_right)
		self.SetLeftLinePoints(self.averaged_left[0][0],self.averaged_left[0][1])
		self.SetRightLinePoints(self.averaged_right[0][0],self.averaged_right[0][1])

		print("asdasd ",self.left_point_upper,self.left_point_lower)

		self.FindLeftIntersectionPoint(self.left_point_upper,self.left_point_lower,180,50)
		self.FindRightIntersectionPoint(self.right_point_upper,self.right_point_lower,180,50)
		print(self.left_point_upper_intersection , self.right_point_upper_intersection,	self.left_point_lower_intersection, self.right_point_lower_intersection)
		
		self.SetMidPoints()
		print("upper mid point: ", self.mid_point_upper)
		print("lower mid point: ", self.mid_point_lower)
		#self.PDError = (self.mid_point_upper[1]-self.mid_point_lower[1])/(self.mid_point_upper[0]-self.mid_point_lower[0])
		self.PDError = (self.mid_point_upper[0]-self.mid_point_lower[0])/(self.mid_point_upper[1]-self.mid_point_lower[1])
		print(self.PDError)

		self.SaveImage(self.imageToShow, "imageToShow")


	def FindLeftIntersectionPoint(self, point1, point2, y1, y2):
		m = (point1[1]-point2[1])/(point1[0]-point2[0])
		print("point1 = ", point1)
		print("point2 = ", point2)
		if m == 0:
			m = 1
		n = point1[1]-(m*point1[0])
		x1 = int((y1-n)/m)
		x2 = int((y2-n)/m)
		cv.circle(self.imageToShow, (x1,y1) , 3, (0,0, 255),-1)
		cv.circle(self.imageToShow, (x2,y2) , 3, (0,0, 255),-1)
		
		self.left_point_upper_intersection = (x1,y1)
		self.left_point_lower_intersection = (x2,y2)

	def FindRightIntersectionPoint(self, point1, point2, y1, y2):
		m = (point1[1]-point2[1])/(point1[0]-point2[0])
		print("point1 = ", point1)
		print("point2 = ", point2)
		if m == 0:
			m = 1
		n = point1[1]-(m*point1[0])
		print("x1 = ", round((y1-n)/m))
		x1 = int((y1-n)/m)
		x2 = int((y2-n)/m)
		print("x1 , x2 = ", x1, x2)
		cv.circle(self.imageToShow, (x1,y1) , 3, (0,0, 255),-1)
		cv.circle(self.imageToShow, (x2,y2) , 3, (0,0, 255),-1)
		
		self.right_point_upper_intersection = (x1,y1)
		self.right_point_lower_intersection = (x2,y2)

	def SetLeftLinePoints(self, rho, theta):
		a = math.cos(theta)
		b = math.sin(theta)
		x0 = a * rho
		y0 = b * rho
		self.left_point_upper = np.array([int(x0 + 1000*(-b)), int(y0 + 1000*(a))])
		self.left_point_lower = np.array([int(x0 - 1000*(-b)), int(y0 - 1000*(a))])
		#cv.circle(self.imageToShow, self.left_point_upper, 3, (0,0, 255),-1)
		#cv.circle(self.imageToShow, self.right_point_upper, 3, (0,0, 255),-1)

	def SetRightLinePoints(self, rho, theta):
		a = math.cos(theta)
		b = math.sin(theta)
		x0 = a * rho
		y0 = b * rho
		self.right_point_upper = np.array([int(x0 + 1000*(-b)), int(y0 + 1000*(a))])
		self.right_point_lower = np.array([int(x0 - 1000*(-b)), int(y0 - 1000*(a))])
		#cv.circle(self.imageToShow, self.left_point_upper, 3, (0,0, 255),-1)
		#cv.circle(self.imageToShow, self.right_point_upper, 3, (0,0, 255),-1)


	def SetMidPoints(self):
		self.mid_point_upper = (int((self.left_point_upper_intersection[0] + self.right_point_upper_intersection[0])/2),int((self.left_point_upper_intersection[1] + self.right_point_upper_intersection[1])/2))
		self.mid_point_lower = ( int( (self.left_point_lower_intersection[0] + self.right_point_lower_intersection[0])/2),int((self.left_point_lower_intersection[1] + self.right_point_lower_intersection[1])/2))

		print("---------> ", self.mid_point_upper, self.mid_point_lower)
		cv.circle(self.imageToShow, self.mid_point_upper, 3, (0,0, 255),-1)
		cv.circle(self.imageToShow, self.mid_point_lower, 3, (0,0, 255),-1)


	def DrawHoughLines(self,rho,theta, color):
		a = math.cos(theta)
		b = math.sin(theta)
		x0 = a * rho
		y0 = b * rho
		pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
		pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
		cv.line(self.imageToShow, pt1, pt2, color, 3, cv.LINE_AA,)


	def ROI(self, x1, y1, x2, y2):
		self.image = self.image[x1:y1,x2:y2]
		self.image_left = self.image_left[x1:y1,x2:y2]
		self.image_right = self.image_right[x1:y1,x2:y2]
		

	def SaveImage(self, image, imageName):
		cv.imwrite(f"/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/Images/{imageName}.jpg", image)

