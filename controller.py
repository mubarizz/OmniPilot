import pygame
import socket
import struct
import time

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller connected: {joystick.get_name()}")
else:
    print("No controller found!")
    exit()

# Setup UDP Socket
sock_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
MATLAB_ADDRESS = ("127.0.0.1", 5006)

# Initial positions
target_x, target_y, target_z, target_psi = 0.0, 0.0, 1.5, 0.0

# Used to control how fast the loop runs (Frames Per Second)
clock = pygame.time.Clock()
running = True

print("Ready to fly! Move the sticks.")

while running:
    # 1. Keep pygame pump alive (needed so it doesn't freeze)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False


    # 2. Get CONTINUOUS stick states
    def deadzone(val):
        return val if abs(val) > 0.1 else 0.0


    roll_input = deadzone(joystick.get_axis(0))  # Left Stick X
    pitch_input = deadzone(joystick.get_axis(1))  # Left Stick Y
    yaw_input = deadzone(joystick.get_axis(3))  # Right Stick X

    # Note: Depending on your controller, Right Stick Y could be axis 3, 4, or 5.
    # Adjust this index if your altitude doesn't respond!
    throttle_input = deadzone(joystick.get_axis(3))

    # 3. Update targets continuously (Speed multiplier)
    speed = 0.05

    # Mapping inputs to XYZ movements
    target_y -= roll_input * speed
    target_x -= pitch_input * speed  # Pushing stick UP (negative Y) moves X forward
    target_psi += yaw_input * speed
    target_z -= throttle_input * speed  # Pushing stick UP (negative Y) moves Z higher

    # Prevent the target from going underground
    if target_z < 0:
        target_z = 0.0

    # 4. Pack and Send to MATLAB constantly
    data = struct.pack('<4f', target_x, target_y, target_z, target_psi)
    sock_out.sendto(data, MATLAB_ADDRESS)

    # Print to console so you can PROVE the numbers are changing
    print(f"Live Target -> X:{target_x:.2f} Y:{target_y:.2f} Z:{target_z:.2f}", end='\r')

    # 5. Lock the loop to 20 updates per second
    clock.tick(20)