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
	
	imageProcessing.setStaticMiddlePoint(200,100)
	imageProcessing.setStaticBottomPoint(200,300)
	while True:
		try:
			startTime = time.time()

			if ultrasonicSensor.calculateDistance() <10:
				break

			imageProcessing.captureImage()

			#imageProcessing.ROI(50,210,0,400)

			imageProcessing.BGRtoGrayScale()

			imageProcessing.cannyEdgeDetection(150,150)
			


			imageProcessing.houghLineTransform()

			imageProcessing.findLineToTrack()


			pidController.getMotorParametersWithError(imageProcessing.distanceBetweenStaticMiddlePointAndTrackLine)

			imageProcessing.prepareDisplayImage()

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
