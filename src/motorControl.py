import RPi.GPIO as GPIO
import socket
import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from picamera2 import Picamera2

# GPIO mode
GPIO.setmode(GPIO.BCM)

# Pin definitions for motors
M1_IN1 = 10
M1_IN2 = 9
M2_IN1 = 20
M2_IN2 = 21
M3_IN1 = 26
M3_IN2 = 27
M4_IN1 = 2
M4_IN2 = 3

# Set up pins as output
motor_pins = [M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2, M4_IN1, M4_IN2]
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

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

# Robot movement function
def move_robot(command):
    # print(f"Executing command: {command}")  # Debug log
    if command == 'forward' or command == 'FORWARD' :
        motor_control(M1_IN1, M1_IN2, 'forward')
        motor_control(M2_IN1, M2_IN2, 'forward')
        motor_control(M3_IN1, M3_IN2, 'backward')
        motor_control(M4_IN1, M4_IN2, 'forward')
    elif command == 'backward' or command == 'BACKWARD':
        motor_control(M1_IN1, M1_IN2, 'backward')
        motor_control(M2_IN1, M2_IN2, 'backward')
        motor_control(M3_IN1, M3_IN2, 'forward')
        motor_control(M4_IN1, M4_IN2, 'backward')
    elif command == 'left' or command == 'LEFT':
        motor_control(M1_IN1, M1_IN2, 'backward')
        motor_control(M2_IN1, M2_IN2, 'forward') 
        motor_control(M3_IN1, M3_IN2, 'forward')
        motor_control(M4_IN1, M4_IN2, 'forward')
    elif command == 'right' or command == 'RIGHT':
        motor_control(M1_IN1, M1_IN2, 'forward')
        motor_control(M2_IN1, M2_IN2, 'backward')
        motor_control(M3_IN1, M3_IN2, 'backward')
        motor_control(M4_IN1, M4_IN2, 'backward')
    else:  # Stop
        for pin in motor_pins:
            GPIO.output(pin, GPIO.LOW)

# Initialize camera
picam2 = Picamera2()
picam2.start()
cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

# Socket setup
HOST = '0.0.0.0'
PORT = 65432
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print(f"Listening for connections on {HOST}:{PORT}...")


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


def avoid_obstacles():
    """Check distances from all sensors and steer accordingly."""
    distances = []
    for trig_pin, echo_pin in zip(TRIG_PINS, ECHO_PINS):
        distance = get_distance(trig_pin, echo_pin)
        distances.append(distance if distance is not None else float('inf'))

    # Determine the direction with the farthest distance (preferable direction)
    directions = ['forward', 'right', 'backward', 'left']
    max_distance = max(distances)
    closest_distance = min(distances)
    safe_distance = 30  # Threshold distance in cm to detect obstacles

    if closest_distance < safe_distance:
        closest_index = distances.index(closest_distance)
        if closest_index == 0:  # Obstacle in front
            return 'left'  # Prefer left turn
        elif closest_index == 1:  # Obstacle on the right
            return 'backward'
        elif closest_index == 2:  # Obstacle at the back
            return 'forward'
        elif closest_index == 3:  # Obstacle on the left
            return 'right'
    return 'stop' if max_distance < safe_distance else 'forward'


# Main program loop
try:
    client_socket, addr = server_socket.accept()
    print(f"Connection established with {addr}")
    socket_command = None

    # Variable to store the last valid command
    last_command = "stop"

    while True:
        # Capture a frame from the camera
        frame = picam2.capture_array()

        # Decode QR codes in the frame
        qr_command = None
        decoded_objects = decode(frame)
        for obj in decoded_objects:
            qr_command = obj.data.decode("utf-8").strip()  # Clean the QR data
            print(f"Detected QR Code command: {qr_command}")
            last_command = qr_command  # Update the last command with QR code

        # Socket communication
        socket_command = None
        try:
            client_socket.settimeout(0.1)  # Non-blocking socket read
            socket_command = client_socket.recv(1024).decode('utf-8').strip()
            if socket_command:
                print(f"Received socket command: {socket_command}")
                last_command = socket_command  # Update the last command with socket command
        except socket.timeout:
            pass

        # Check for obstacles and override last command if necessary
        obstacle_avoidance_command = avoid_obstacles()
        if obstacle_avoidance_command != 'stop':
            last_command = obstacle_avoidance_command

            
        # Use the last valid command if no new command is detected
        print(f"Executing last command: {last_command}")
        move_robot(last_command)

        # Display camera feed
        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break



except KeyboardInterrupt:
    print("Program stopped by User")
finally:
    # Cleanup
    server_socket.close()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
