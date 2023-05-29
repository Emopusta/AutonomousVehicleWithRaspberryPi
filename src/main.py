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

	#imageProcessing.setStaticMiddlePoint(200,100)
	#imageProcessing.setStaticBottomPoint(200,300)
	while True:
		try:
			startTime = time.time()

			"""
			if ultrasonicSensor.calculateDistance() <10:
				break
			"""
			imageProcessing.CaptureImage()

			imageProcessing.GaussFilter()

			imageProcessing.HSVFilter()

			imageProcessing.CannyEdgeDetection(60,150)

			imageProcessing.LaneDetection(50)


			endTime = time.time()


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
