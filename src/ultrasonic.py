import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM (Broadcom)
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the ultrasonic sensors
TRIG_PINS = [17, 22, 24, 5]
ECHO_PINS = [18, 23, 25, 6]

# Set up the GPIO pins
for pin in TRIG_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

for pin in ECHO_PINS:
    GPIO.setup(pin, GPIO.IN)

def get_distance(trig_pin, echo_pin):
    # Send a pulse to the trigger pin to start the measurement
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microseconds pulse
    GPIO.output(trig_pin, GPIO.LOW)

    # Wait for the echo pin to go HIGH (signal received)
    pulse_start = time.time()
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()
        # Timeout condition (no pulse received in 1 second)
        if pulse_start - time.time() > 1:
            return None  # Return None if no pulse received in 1 second

    # Wait for the echo pin to go LOW (pulse ended)
    pulse_end = time.time()
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate the distance based on the pulse duration
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # 17150 is the speed of sound in cm/s

    return round(distance, 2)

try:
    while True:
        for i in range(4):
            print(f"Sensor {i + 1} measuring distance:")
            distance = get_distance(TRIG_PINS[i], ECHO_PINS[i])
            if distance is not None:
                print(f"Distance: {distance} cm\n")
            else:
                print("No object detected, or timeout occurred.\n")
        
        time.sleep(1)  # Wait before measuring again

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
