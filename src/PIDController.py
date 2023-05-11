import RPi.GPIO as GPIO
from pinConfiguration.MotorDriverPinConfiguration import MotorDriverPinConfiguration as pin
import time
from simple_pid import PID

class PIDController:
	error = 0
	integral = 0
	last_error = 0
	kp = 0.5
	ki = 0.1
	kd = 0.2
	pid = PID(0.5, 0.1, 0.05, setpoint=0)


	def __init__(self, error, dutyCycle):
		self.error = error
		GPIO.setup(pin.en1, GPIO.OUT)
		GPIO.setup(pin.in1, GPIO.OUT)
		GPIO.setup(pin.in2, GPIO.OUT)
		GPIO.setup(pin.en2, GPIO.OUT)
		GPIO.setup(pin.in3, GPIO.OUT)
		GPIO.setup(pin.in4, GPIO.OUT)

		GPIO.output(pin.in1, GPIO.LOW)
		GPIO.output(pin.in2, GPIO.LOW)
		GPIO.output(pin.in3, GPIO.LOW)
		GPIO.output(pin.in4, GPIO.LOW)
		self.duty = dutyCycle

		self.p1 = GPIO.PWM(pin.en1, 1000)
		self.p2 = GPIO.PWM(pin.en2, 1000)

		self.p1.start(self.duty)
		self.p2.start(self.duty)



	def controlLeftSide(self,speed):
		print(speed)
		if speed > 0:
			GPIO.output(pin.in1, GPIO.LOW)
			GPIO.output(pin.in2, GPIO.HIGH)
		elif speed < 0:
			GPIO.output(pin.in1, GPIO.HIGH)
			GPIO.output(pin.in2, GPIO.LOW)
		else:
			GPIO.output(pin.in1, GPIO.LOW)
			GPIO.output(pin.in2, GPIO.LOW)

		speed = abs(speed)
		if speed > 99:
			speed = 99.0
		

		print("left_side -> ", speed)
		self.p1.ChangeDutyCycle(abs(speed))

	def controlRightSide(self,speed):
		if speed > 0:
			GPIO.output(pin.in3, GPIO.HIGH)
			GPIO.output(pin.in4, GPIO.LOW)
		elif speed < 0:
			GPIO.output(pin.in3, GPIO.LOW)
			GPIO.output(pin.in4, GPIO.HIGH)
		else:
			GPIO.output(pin.in3, GPIO.LOW)
			GPIO.output(pin.in4, GPIO.LOW)
		speed = abs(speed)
		if speed > 99:
			speed = 99.0
		

		print("right side -> ", abs(speed))
		self.p2.ChangeDutyCycle(abs(speed))


	def getMotorParametersWithError(self, newError):


		error = newError # replace with actual error calculation

		self.integral += error
		derivative = error - self.last_error
		self.last_error = error

		            # calculate output values for motor control
		output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
		left_speed = 0.5 + output
		right_speed = 0.5 - output

		            # limit speed between 20 and 40 in a proportional way
		if left_speed < 0:
			left_speed = 0
		if right_speed < 0:
			right_speed = 0
		if left_speed > 1:
			left_speed = 1
		if right_speed > 1:
			right_speed = 1
		left_speed = (left_speed * 20) + 7.5
		right_speed = (right_speed * 20) + 7.5


		self.controlLeftSide(speed=left_speed) 
		self.controlRightSide(speed=right_speed)
		


		"""
		#self.integral += newError
		newError /= 5
		derivative = newError - self.error
		self.error = newError
		output = (self.kp * newError) + (self.ki * self.integral) + (self.kd * derivative)

		print("integral -> ", self.integral)
		print("output of the PID -> ", output)
		
		left_speed =  0.5 + output
		right_speed =  0.5 - output
		left_speed = (left_speed/10) + 15
		right_speed = (right_speed/10)+ 15
		print(left_speed, right_speed)
		self.controlLeftSide(speed=left_speed)
		self.controlRightSide(speed=right_speed)
		time.sleep(0.1)
		"""
	


"""

# Main loop
while True:
    # Read sensor values and calculate error
    # (replace this with your sensor code)
    error = 0

    # Use the PID controller to calculate the correction value
    correction = pid(error)

    # Adjust the motor speeds based on the correction value
    left_speed = 50 + correction
    right_speed = 50 - correction
    left_motor(left_speed)
    right_motor(right_speed)

    # Wait for a short time before reading sensor values again
    sleep(0.1)
"""
