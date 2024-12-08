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
    if command == 'forward' or command == 'FORWARD' or command == 'DOCK':
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
