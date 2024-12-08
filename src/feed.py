import RPi.GPIO as GPIO
import socket
import json
import time
import math

# Ultrasonic sensor pins
TRIG_PINS = [17, 22, 24, 5]
ECHO_PINS = [18, 23, 25, 6]
SENSOR_ANGLES = [0, 90, 180, 270]  # Sensor angles

# Setup GPIO
GPIO.setmode(GPIO.BCM)
for trig in TRIG_PINS:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.output(trig, GPIO.LOW)
for echo in ECHO_PINS:
    GPIO.setup(echo, GPIO.IN)

# Socket setup
HOST = "192.168.100.151"  # Host machine's IP address
PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Function to get distance
def get_distance(trig_pin, echo_pin):
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microseconds pulse
    GPIO.output(trig_pin, GPIO.LOW)

    pulse_start = time.time()
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()
        if time.time() - pulse_start > 1:
            return None

    pulse_end = time.time()
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

try:
    while True:
        wall_points = []

        for i in range(len(TRIG_PINS)):
            distance = get_distance(TRIG_PINS[i], ECHO_PINS[i])
            if distance and 5 < distance <= 400:  # Valid range
                wall_points.append({
                    "distance": distance,
                    "angle": SENSOR_ANGLES[i]
                })

        # Send wall points to the host
        message = {
            "wall_points": wall_points,
            "status": "alive"
        }
        sock.sendto(json.dumps(message).encode(), (HOST, PORT))
        time.sleep(0.2)  # Adjust frequency as needed

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    GPIO.cleanup()
    sock.close()