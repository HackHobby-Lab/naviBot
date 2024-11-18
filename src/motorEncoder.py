import RPi.GPIO as GPIO
import time
from threading import Thread

# GPIO mode
GPIO.setmode(GPIO.BCM)

# Pin definitions for motors
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

# Pin definitions for encoders
ENC1_PIN = 11  # Motor 1 encoder
ENC2_PIN = 19  # Motor 2 encoder
ENC3_PIN = 16  # Motor 3 encoder
ENC4_PIN = 13  # Motor 4 encoder

# Pulses per revolution (depends on encoder)
PULSES_PER_REV = 20

# RPM counters
pulse_counts = [0, 0, 0, 0]
rpms = [0, 0, 0, 0]

# Motor and encoder pins setup
motor_pins = [M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2, M4_IN1, M4_IN2]
encoder_pins = [ENC1_PIN, ENC2_PIN, ENC3_PIN, ENC4_PIN]

for pin in motor_pins + encoder_pins:
    GPIO.setup(pin, GPIO.OUT if pin in motor_pins else GPIO.IN)

# Pulse counting callback
def pulse_callback(channel):
    motor_index = encoder_pins.index(channel)
    pulse_counts[motor_index] += 1

# Attach interrupts to encoder pins
for pin in encoder_pins:
    GPIO.add_event_detect(pin, GPIO.RISING, callback=pulse_callback)

# RPM calculation function
def calculate_rpm():
    global pulse_counts, rpms
    while True:
        # Copy the pulse counts and reset
        current_counts = pulse_counts[:]
        pulse_counts = [0, 0, 0, 0]

        # Calculate RPM for each motor
        for i in range(4):
            rpms[i] = (current_counts[i] * 60) / PULSES_PER_REV

        # Print RPMs for debugging
        print(f"RPMs: Motor 1: {rpms[0]}, Motor 2: {rpms[1]}, Motor 3: {rpms[2]}, Motor 4: {rpms[3]}")
        time.sleep(1)

# Start RPM calculation in a separate thread
Thread(target=calculate_rpm, daemon=True).start()

# Motor control function
def motor_control(in1, in2, direction):
    if direction == 'forward':
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else:  # Stop
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

# Main program
try:
    while True:
        # Motor 1: Forward
        motor_control(M1_IN1, M1_IN2, 'forward')
        # Motor 2: Backward
        motor_control(M2_IN1, M2_IN2, 'backward')
        # Motor 3: Forward
        motor_control(M3_IN1, M3_IN2, 'forward')
        # Motor 4: Backward
        motor_control(M4_IN1, M4_IN2, 'backward')

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
