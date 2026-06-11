"""
beacon_controller — one distress beacon.

Spawned dynamically by the signal manager as "beacon_<id>". Broadcasts
"SOS <id>" on channel 1 every step (32 ms) so scouts get a continuous,
fresh bearing. It never silences itself — the manager removes the whole
node once the assigned scout physically reaches it.
"""
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
