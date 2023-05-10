from pinConfiguration.UltrasonicSensorPinConfiguration import UltrasonicSensorPinConfiguration as pin
import RPi.GPIO as GPIO
import time

class UltrasonicSensor:

	def __init__(self):
		GPIO.setup(pin.echoPin, GPIO.IN)
		GPIO.setup(pin.triggerPin, GPIO.OUT)


	def calculateDistance(self):

		GPIO.output(pin.triggerPin, 0)
		time.sleep(.1)
		GPIO.output(pin.triggerPin, 1)
		time.sleep(.1)
		GPIO.output(pin.triggerPin, 0)
		while GPIO.input(pin.echoPin) == 0:
			pass
		echoStartTime = time.time()
		while GPIO.input(pin.echoPin) == 1:
			pass
		echoStopTime = time.time()
		ptt = echoStopTime - echoStartTime
		distance = 1234.367 * ptt * 5280 * 12 / 3600
		self.distance = distance
		print("Ultrasonic distance -> ", distance)
		return distance
