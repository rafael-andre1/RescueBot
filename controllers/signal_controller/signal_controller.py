"""
pinger — sits in the corner and broadcasts a distress ping on channel 1
         every PING_INTERVAL seconds. The ping carries the source's own
         position: "SOS <x> <y>", so the scout homes on the received
         coordinates rather than reading them from the map.
"""
from controller import Robot

TIME_STEP     = 32
PING_INTERVAL = 3.0   # seconds

robot   = Robot()
emitter = robot.getDevice("emitter")
gps     = robot.getDevice("gps")
gps.enable(TIME_STEP)

elapsed = 0.0
while robot.step(TIME_STEP) != -1:
    elapsed += TIME_STEP / 1000.0
    if elapsed >= PING_INTERVAL:
        elapsed = 0.0
        x, y, _ = gps.getValues()
        msg = "SOS %.3f %.3f" % (x, y)
        emitter.send(msg.encode("utf-8"))
        print("[pinger] sent " + msg)
