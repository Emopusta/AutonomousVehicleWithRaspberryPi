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
	pidController = PIDController(10, 20.0)

	imageProcessing.setStaticMiddlePoint(200,150)

	while True:
		try:
			ultrasonicSensor.calculateDistance()
			if ultrasonicSensor.distance <= 5:
				break

			imageProcessing.captureImage()
			imageProcessing.BGRtoGrayScale()
			imageProcessing.cannyEdgeDetection(150,150)
			imageProcessing.houghLineTransform(threshold=50)
			imageProcessing.findLineToTrack()
			imageProcessing.findDistanceBetweenStaticMiddlePointAndTrackLine()
			imageProcessing.prepareDisplayImage()

			pidController.getMotorParametersWithError(imageProcessing.distanceBetweenStaticMiddlePointAndTrackLine)
		except KeyboardInterrupt:
			GPIO.cleanup()
			return 0
		except:
			print("cizgi yok")
	print("program sonlandi")
	cv.waitKey()
	return 0













if __name__ == "__main__":
	main(sys.argv[1:])
