from MotorDriverPinConfiguration import  MotorDriverPinConfiguration
from ImageProcessing import ImageProcessing
from UltrasonicSensorPinConfiguration import UltrasonicSensorPinConfiguration as Upin
from UltrasonicSensor import UltrasonicSensor
from PIDController import PIDController
import sys
import cv2 as cv
import RPi.GPIO as GPIO

def main(argv):
	GPIO.cleanup()
	GPIO.setmode(GPIO.BOARD)

	imageProcessing = ImageProcessing("/home/emopusta/Emre/AutonomousVehicleWithRaspberryPi/deneme.jpg")
	ultrasonicSensor = UltrasonicSensor()
	pidController = PIDController(10, 20)

	imageProcessing.setStaticMiddlePoint(250,250)

	while True:

		ultrasonicSensor.calculateDistance()
		if ultrasonicSensor.distance <= 5:
			break

		imageProcessing.captureImage()
		imageProcessing.BGRtoGrayScale()
		imageProcessing.cannyEdgeDetection(150,150)
		imageProcessing.grayScaleToBGRShowImage()
		imageProcessing.houghLineTransform(threshold=50)
		imageProcessing.findLineToTrack()
		imageProcessing.findDistanceBetweenStaticMiddlePointAndTrackLine()


	cv.waitKey()
	return 0













if __name__ == "__main__":
	main(sys.argv[1:])
