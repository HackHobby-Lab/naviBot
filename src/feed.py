import cv2
from flask import Flask, Response
from picamera2 import Picamera2

# Initialize the Flask app
app = Flask(__name__)

# Initialize the camera
picam2 = Picamera2()

# Start the camera preview
picam2.start()

def gen():
    """Generate frames for streaming"""
    while True:
        # Capture a frame from the camera
        frame = picam2.capture_array()

        # Encode the frame in JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if ret:
            # Convert the frame to bytes and yield it
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    """Serve the video feed"""
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
