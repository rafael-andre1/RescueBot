"""
pinger — sits in the corner and broadcasts 'HELP' on channel 1
         every PING_INTERVAL seconds.
"""
from controller import Robot

TIME_STEP     = 32
PING_INTERVAL = 3.0   # seconds

robot   = Robot()
emitter = robot.getDevice("emitter")

elapsed = 0.0
while robot.step(TIME_STEP) != -1:
    elapsed += TIME_STEP / 1000.0
    if elapsed >= PING_INTERVAL:
        elapsed = 0.0
        emitter.send("HELP".encode("utf-8"))
        print("[pinger] sent HELP")