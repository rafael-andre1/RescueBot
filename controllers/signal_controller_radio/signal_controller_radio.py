"""
signal_controller_radio — broadcasts a distress signal on channel 1 every step.
         Sends identity only ("SOS"). No position — the scout locates the
         beacon from signal strength and bearing alone.
         Broadcasts every step (32 ms) so the scout gets a continuous,
         fresh bearing rather than a stale 3-second-old one.
"""
from controller import Robot

TIME_STEP = 32

robot   = Robot()
emitter = robot.getDevice("emitter")

while robot.step(TIME_STEP) != -1:
    emitter.send(b"SOS")
