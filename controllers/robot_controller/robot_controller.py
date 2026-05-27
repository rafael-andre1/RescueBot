"""
scout — drives forward, avoids walls with the LiDAR,
        keeps the camera streaming, and prints any ping it receives.
"""
from controller import Robot

TIME_STEP      = 32
CRUISE_SPEED   = 3.0    # rad/s
TURN_SPEED     = 2.0    # rad/s
SAFE_DISTANCE  = 0.4    # metres; spin away if anything in the front arc is closer
FRONT_ARC_DEG  = 60     # total forward arc width (±30°)

robot = Robot()

# --- Devices ---
left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()                # nice to have for the 3D view

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)


def front_min_distance(ranges):
    """Min distance across the forward ±(FRONT_ARC_DEG/2) arc.
    LiDAR ray 0 points along the robot's +X (forward); rays go counter-clockwise."""
    n = len(ranges)
    half = int((FRONT_ARC_DEG / 2.0) * n / 360.0)
    forward = list(ranges[:half]) + list(ranges[-half:])
    forward = [r for r in forward if r > 0.0]  # filter "no return" values
    return min(forward) if forward else float("inf")


def drain_pings():
    """Print every packet waiting in the receiver queue."""
    while receiver.getQueueLength() > 0:
        try:
            msg = receiver.getString()           # works for text packets
        except Exception:
            msg = receiver.getBytes()
        strength  = receiver.getSignalStrength() # 1/dist^2
        direction = receiver.getEmitterDirection()  # unit vec in robot frame
        print(f"[scout] PING '{msg}'  "
              f"strength={strength:.4f}  "
              f"dir=({direction[0]:+.2f}, {direction[1]:+.2f}, {direction[2]:+.2f})")
        receiver.nextPacket()


# --- Main loop ---
while robot.step(TIME_STEP) != -1:
    drain_pings()

    ranges = lidar.getRangeImage()
    if not ranges:
        continue

    if front_min_distance(ranges) < SAFE_DISTANCE:
        # Obstacle ahead → spin left in place
        left_motor.setVelocity(-TURN_SPEED)
        right_motor.setVelocity(+TURN_SPEED)
    else:
        left_motor.setVelocity(CRUISE_SPEED)
        right_motor.setVelocity(CRUISE_SPEED)