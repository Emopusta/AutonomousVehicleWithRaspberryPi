import time

class PIDController:
    integral = 0
    kp = 0.5
    ki = 0.2
    kd = 0.1

    def __init__(self, error) -> None:
        self.error = error

    def GetMotorParametersWithError(self, newError):

        self.integral += newError
        derivative = newError - self.error
        self.error = newError

        output = (self.kp * newError) + (self.ki * self.integral) + (self.kd * derivative)


        left_speed = 0.5 + output
        right_speed = 0.5 - output
        # left_motor.forward(speed=left_speed)
        # right_motor.forward(speed=right_speed)

      
        time.sleep(0.1)

"""
import RPi.GPIO as GPIO
from time import sleep
from simple_pid import PID

# Define the pins for the left motor
left_forward_pin = 17
left_backward_pin = 18

# Define the pins for the right motor
right_forward_pin = 27
right_backward_pin = 22

# Set up the GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_forward_pin, GPIO.OUT)
GPIO.setup(left_backward_pin, GPIO.OUT)
GPIO.setup(right_forward_pin, GPIO.OUT)
GPIO.setup(right_backward_pin, GPIO.OUT)

# Create a PID controller object
pid = PID(1, 0.1, 0.05, setpoint=0)

# Define a function to control the left motor
def left_motor(speed):
    if speed > 0:
        GPIO.output(left_forward_pin, GPIO.HIGH)
        GPIO.output(left_backward_pin, GPIO.LOW)
    elif speed < 0:
        GPIO.output(left_forward_pin, GPIO.LOW)
        GPIO.output(left_backward_pin, GPIO.HIGH)
    else:
        GPIO.output(left_forward_pin, GPIO.LOW)
        GPIO.output(left_backward_pin, GPIO.LOW)
    speed = abs(speed)
    if speed > 100:
        speed = 100
    pwm = GPIO.PWM(left_forward_pin, 100)
    pwm.start(speed)

# Define a function to control the right motor
def right_motor(speed):
    if speed > 0:
        GPIO.output(right_forward_pin, GPIO.HIGH)
        GPIO.output(right_backward_pin, GPIO.LOW)
    elif speed < 0:
        GPIO.output(right_forward_pin, GPIO.LOW)
        GPIO.output(right_backward_pin, GPIO.HIGH)
    else:
        GPIO.output(right_forward_pin, GPIO.LOW)
        GPIO.output(right_backward_pin, GPIO.LOW)
    speed = abs(speed)
    if speed > 100:
        speed = 100
    pwm = GPIO.PWM(right_forward_pin, 100)
    pwm.start(speed)

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