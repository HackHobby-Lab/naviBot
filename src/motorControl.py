import RPi.GPIO as GPIO
import time

# GPIO mode
GPIO.setmode(GPIO.BCM)

# Pin definitions for 4 motors
# Motor 1
M1_IN1 = 2
M1_IN2 = 3

# Motor 2
M2_IN1 = 26
M2_IN2 = 27

# Motor 3
M3_IN1 = 10
M3_IN2 = 9

# Motor 4
M4_IN1 = 20
M4_IN2 = 21

# Set up pins as output
motor_pins = [
    M1_IN1, M1_IN2,
    M2_IN1, M2_IN2,
    M3_IN1, M3_IN2,
    M4_IN1, M4_IN2
]

for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

def motor_control(in1, in2, direction):
    """
    Control a single motor.
    :param in1: GPIO pin for IN1
    :param in2: GPIO pin for IN2
    :param direction: 'forward', 'backward', or 'stop'
    """
    if direction == 'forward':
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:  # Stop the motor
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

try:
    while True:
        # Motor 1: Forward
        motor_control(M1_IN1, M1_IN2, 'forward')
        # Motor 2: Backward
        motor_control(M2_IN1, M2_IN2, 'forward')
        # Motor 3: Forward
        motor_control(M3_IN1, M3_IN2, 'forward')
        # Motor 4: Backward
        motor_control(M4_IN1, M4_IN2, 'forward')

        print("Motors running...")
        time.sleep(5)

        # Stop all motors
        motor_control(M1_IN1, M1_IN2, 'stop')
        motor_control(M2_IN1, M2_IN2, 'stop')
        motor_control(M3_IN1, M3_IN2, 'stop')
        motor_control(M4_IN1, M4_IN2, 'stop')

        print("Motors stopped.")
        time.sleep(2)

except KeyboardInterrupt:
    print("Program stopped by User")
    GPIO.cleanup()
