import socket
import keyboard  # Install with `pip install keyboard`

# Set up connection
HOST = '192.168.100.151'  # Replace with your Raspberry Pi's IP
PORT = 65432

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

print("Connected to the Raspberry Pi. Use arrow keys to control the robot (Esc to quit).")

# Store the current state of movement to avoid redundant commands
current_command = None

try:
    while True:
        # Detect key presses
        if keyboard.is_pressed('up') and current_command != 'forward':
            client_socket.sendall(b'forward\n')
            current_command = 'forward'
        elif keyboard.is_pressed('down') and current_command != 'backward':
            client_socket.sendall(b'backward\n')
            current_command = 'backward'
        elif keyboard.is_pressed('left') and current_command != 'left':
            client_socket.sendall(b'left\n')
            current_command = 'left'
        elif keyboard.is_pressed('right') and current_command != 'right':
            client_socket.sendall(b'right\n')
            current_command = 'right'
        # Detect key releases (no key pressed)
        elif not keyboard.is_pressed('up') and not keyboard.is_pressed('down') \
                and not keyboard.is_pressed('left') and not keyboard.is_pressed('right') \
                and current_command != 'stop':
            client_socket.sendall(b'stop\n')
            current_command = 'stop'
        
        # Exit if Esc key is pressed
        if keyboard.is_pressed('esc'):
            print("Exiting...")
            break

except KeyboardInterrupt:
    print("Program stopped by User")
finally:
    client_socket.sendall(b'stop\n')  # Ensure the robot stops when exiting
    client_socket.close()
