"""
signal_controller_swarm — distress beacon for swarm arena.
    Each pinger Robot in arena_swarm.wbt configures its Emitter on a
    distinct channel (1=red, 2=yellow, 3=green). The matching scout
    listens on that channel only, so colour, channel and beacon are
    bound together by the world file alone — this controller has no
    per-pinger config and stays identical for all three.

    Sends "SOS" every step (32 ms) so each scout gets a continuous,
    fresh bearing rather than a stale one.
"""
from controller import Robot

TIME_STEP = 32

robot   = Robot()
emitter = robot.getDevice("emitter")

# Channel is fixed on the Emitter node in the world file; we just shout SOS.
while robot.step(TIME_STEP) != -1:
    emitter.send(b"SOS")
