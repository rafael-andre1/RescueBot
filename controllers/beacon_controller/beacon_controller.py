from controller import Robot

TIME_STEP = 32

robot = Robot()
try:
    BEACON_ID = int(robot.getName().split("_")[-1])
except ValueError:
    BEACON_ID = 0

emitter = robot.getDevice("emitter")
msg = ("SOS %d" % BEACON_ID).encode("utf-8")

while robot.step(TIME_STEP) != -1:
    emitter.send(msg)
