from pinConfiguration.MotorDriverPinConfiguration import  MotorDriverPinConfiguration

from ImageProcessing import ImageProcessing
from pinConfiguration.UltrasonicSensorPinConfiguration import UltrasonicSensorPinConfiguration as Upin
from UltrasonicSensor import UltrasonicSensor
from PIDController import PIDController
import sys
import cv2 as cv
import RPi.GPIO as GPIO
import time

def main(argv):
	GPIO.setmode(GPIO.BOARD)
	
	imageProcessing = ImageProcessing("/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/deneme.jpg")
	ultrasonicSensor = UltrasonicSensor()
	pidController = PIDController(0, 20.0)

	imageProcessing.setStaticMiddlePoint(200,150)

	while True:
		try:
			startTime = time.time()

			if ultrasonicSensor.calculateDistance() <10:
				break
			endUltrasonic = time.time()

			startCaptureTime = time.time()
			imageProcessing.captureImage()
			endCaptureTime = time.time()

			#imageProcessing.ROI(100,500,0,500)
			
			startConvertTime = time.time()
			imageProcessing.BGRtoGrayScale()
			endConvertTime = time.time()

			startCannyTime = time.time()
			imageProcessing.cannyEdgeDetection(150,150)
			endCannyTime = time.time()
			
			startHoughTime = time.time()
			imageProcessing.houghLineTransform(threshold=50)
			endHoughTime = time.time()

			startTrackTime = time.time()
			imageProcessing.findLineToTrack()
			endTrackTime = time.time()

			startPointToTrackTime = time.time()
			imageProcessing.findDistanceBetweenStaticMiddlePointAndTrackLine()
			endPointToTrackTime = time.time()

			imageProcessing.prepareDisplayImage()


			pidController.getMotorParametersWithError(imageProcessing.distanceBetweenStaticMiddlePointAndTrackLine)
			endTime = time.time()
			
			print("ultrasonic = ", endUltrasonic- startTime)
			print("image capture time = ", endCaptureTime-startCaptureTime)
			print("image convert grayscale time = ", endConvertTime - startConvertTime)
			print("canny edge time = " , endCannyTime - startCannyTime)
			print("hough lines time = ", endHoughTime - startHoughTime)
			print("Track time = ", endTrackTime - startTrackTime)
			print("Error find time = ", endPointToTrackTime - startPointToTrackTime)
			print("cycle time = ", endTime-startTime)
			print("*******************************")

		except KeyboardInterrupt:
			GPIO.cleanup()
			return 0
		#except:
		#	print("cizgi yok")
	print("program sonlandi")
	cv.waitKey()
	return 0













if __name__ == "__main__":
	main(sys.argv[1:])
