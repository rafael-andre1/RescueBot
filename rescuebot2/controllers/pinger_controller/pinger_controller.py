"""
pinger — a distress beacon. Several of these are scattered around the arena.

Iteration-2 change (NO POSITIONAL CHEATING):
  The beacon NO LONGER broadcasts its own coordinates. It only sends its
  identity: "SOS <name>". The scout must work out *where* a beacon is purely
  from the radio it can sense — signal strength (≈ 1/r²) and the bearing from
  the receiver's getEmitterDirection(). The beacon therefore has no GPS.

It emits every step so the scout gets an almost-continuous bearing to home on.
The console print is throttled so the log stays readable.
"""
from controller import Robot

TIME_STEP   = 32
PRINT_EVERY = 3.0   # seconds between console prints (sending still happens every step)

robot   = Robot()
emitter = robot.getDevice("emitter")
name    = robot.getName()          # e.g. "pinger_1" — this is the only thing we leak
msg     = ("SOS " + name).encode("utf-8")

elapsed = 0.0
while robot.step(TIME_STEP) != -1:
    emitter.send(msg)              # broadcast identity only — never a position
    elapsed += TIME_STEP / 1000.0
    if elapsed >= PRINT_EVERY:
        elapsed = 0.0
        print("[%s] broadcasting distress (id only, no position)" % name)
