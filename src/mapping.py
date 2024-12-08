import RPi.GPIO as GPIO
import time
import math
import tkinter as tk

# Set the GPIO mode to BCM (Broadcom)
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the ultrasonic sensors
TRIG_PINS = [17, 22, 24, 5]
ECHO_PINS = [18, 23, 25, 6]

# Sensor angles relative to the robot
SENSOR_ANGLES = [0, 90, 180, 270]

# Set up the GPIO pins
for pin in TRIG_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

for pin in ECHO_PINS:
    GPIO.setup(pin, GPIO.IN)

def get_distance(trig_pin, echo_pin):
    """Measure distance using an ultrasonic sensor."""
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microseconds pulse
    GPIO.output(trig_pin, GPIO.LOW)

    pulse_start = time.time()
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()
        if time.time() - pulse_start > 1:
            return None  # Timeout

    pulse_end = time.time()
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # 17150 cm/s speed of sound

    return round(distance, 2)

def polar_to_cartesian(distance, angle):
    """Convert polar coordinates to Cartesian coordinates."""
    x = distance * math.cos(math.radians(angle))
    y = distance * math.sin(math.radians(angle))
    return x, y

# Initialize Tkinter GUI
root = tk.Tk()
root.title("Real-Time Room Mapping")
canvas_size = 500
canvas = tk.Canvas(root, width=canvas_size, height=canvas_size, bg="white")
canvas.pack()

# Robot position in the center of the canvas
robot_x, robot_y = canvas_size // 2, canvas_size // 2
robot_size = 10

# Draw the robot
robot_id = canvas.create_rectangle(
    robot_x - robot_size, robot_y - robot_size,
    robot_x + robot_size, robot_y + robot_size,
    fill="green", outline="black"
)

# Store boundary points
boundary_points = []

def update_map():
    """Update the boundary map in real time."""
    global boundary_points

    new_boundary_points = []

    for i in range(4):
        distance = get_distance(TRIG_PINS[i], ECHO_PINS[i])
        if distance is not None and 5 < distance <= 400:  # Valid range
            x, y = polar_to_cartesian(distance, SENSOR_ANGLES[i])

            # Scale and shift to fit canvas
            scaled_x = robot_x + (x * 2)
            scaled_y = robot_y - (y * 2)  # Invert y-axis for correct orientation

            # Approximate boundary at 15 cm
            if distance > 15:
                scale_factor = 15 / distance
                scaled_x = robot_x + (x * scale_factor * 2)
                scaled_y = robot_y - (y * scale_factor * 2)

            new_boundary_points.append((scaled_x, scaled_y))

    # Update the boundary points
    if new_boundary_points:
        boundary_points += new_boundary_points

        # Clear old map and redraw robot
        canvas.delete("boundary")
        for i in range(len(boundary_points)):
            if i > 0:
                canvas.create_line(
                    boundary_points[i - 1][0], boundary_points[i - 1][1],
                    boundary_points[i][0], boundary_points[i][1],
                    fill="blue", tags="boundary"
                )

    # Schedule the next update
    root.after(50, update_map)  # 50 ms for real-time updates

try:
    update_map()
    root.mainloop()

except KeyboardInterrupt:
    print("Mapping stopped by User")
    GPIO.cleanup()
    root.destroy()
