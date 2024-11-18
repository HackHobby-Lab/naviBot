import cv2
import numpy as np
from pyzbar.pyzbar import decode
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()

# Start the camera preview
picam2.start()

# Set up a window for displaying the feed
cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

while True:
    # Capture a frame from the camera
    frame = picam2.capture_array()

    # Decode QR codes in the frame
    decoded_objects = decode(frame)

    # Loop through all decoded objects and display the result
    for obj in decoded_objects:
        # Get the data from the QR code
        qr_data = obj.data.decode("utf-8")

        # Print the QR code data to the terminal
        print(f"QR Code Data: {qr_data}")

        # Get the points for drawing a rectangle around the QR code
        points = obj.polygon
        if len(points) == 4:
            pts = points
        else:
            pts = cv2.convexHull(np.array([point for point in points], dtype=np.float32))

        # Draw a bounding box around the QR code
        cv2.polylines(frame, [np.int32(pts)], isClosed=True, color=(0, 255, 0), thickness=5)

        # Display the QR code data on the screen
        cv2.putText(frame, qr_data, (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # Display the frame with the detected QR codes
    cv2.imshow("Camera Feed", frame)

    # Wait for 1 ms and exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
picam2.stop()
cv2.destroyAllWindows()
