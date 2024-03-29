from pinConfiguration.MotorDriverPinConfiguration import  MotorDriverPinConfiguration

from ImageProcessing import ImageProcessing
from pinConfiguration.UltrasonicSensorPinConfiguration import UltrasonicSensorPinConfiguration as Upin
from UltrasonicSensor import UltrasonicSensor
from PDController import PDController
import sys
import cv2 as cv
import RPi.GPIO as GPIO
import time

def main(argv):
	GPIO.setmode(GPIO.BOARD)

	imageProcessing = ImageProcessing()
	ultrasonicSensor = UltrasonicSensor()
	pdController = PDController(0, 20.0)

	while True:
		try:
			startTime = time.time()

			"""
			if ultrasonicSensor.calculateDistance() <10:
				break
			"""

			imageProcessing.CaptureImage()

			#imageProcessing.ROI(0,240,0,315)

			imageProcessing.GaussFilter()

			imageProcessing.HSVFilter()

			imageProcessing.CannyEdgeDetection(60,150)

			imageProcessing.LaneDetection(35)

			pdController.GetMotorParametersWithError(imageProcessing.PDError)
			
			endTime = time.time()


			print("cycle time = ", endTime-startTime)

		except KeyboardInterrupt:
			GPIO.cleanup()
			return 0

		except:
			print("Something happening!")

	print("Exiting Program...")
	cv.waitKey()
	return 0

if __name__ == "__main__":
	main(sys.argv[1:])
