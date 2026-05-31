"""
signal_controller — Supervisor running ONE distress beacon through
NUM_BEACONS random positions on the map.

At startup, picks NUM_BEACONS random world positions and teleports itself
to the first one. Broadcasts "SOS" every step (so the scout's radio sees
a fresh bearing each tick). When the scout reaches the current beacon
(within RESCUE_RADIUS) the controller advances to the next position and
teleports there. The beacon NEVER disappears on a timer — it only moves
once the scout has physically arrived.

World requirements (already set in arena.wbt):
  • This Robot has  supervisor TRUE  and one Emitter named "emitter" on
    channel 1.
  • The scout's Robot is named "scout".
"""
import random
from controller import Supervisor

TIME_STEP     = 32
NUM_BEACONS   = 10
RESCUE_RADIUS = 0.40       # scout within this distance counts as a rescue (m)
BEACON_Z      = 0.05
SCOUT_NAME    = "scout"

# Random positions inside the scout's map (with a small margin from edges)
X_RANGE = (-3.5, 3.5)
Y_RANGE = (-2.5, 2.5)

robot   = Supervisor()
emitter = robot.getDevice("emitter")
translation_field = robot.getSelf().getField("translation")

# Find the scout Robot by name (no DEF needed)
scout_node = None
root_children = robot.getRoot().getField("children")
for i in range(root_children.getCount()):
    node = root_children.getMFNode(i)
    name_f = node.getField("name")
    if name_f is not None and name_f.getSFString() == SCOUT_NAME:
        scout_node = node
        break
if scout_node is None:
    print("[signal] FATAL: no Robot named '%s' found" % SCOUT_NAME)
    while robot.step(TIME_STEP) != -1:
        pass

# Pre-generate the random distress positions
positions = [(random.uniform(*X_RANGE), random.uniform(*Y_RANGE))
             for _ in range(NUM_BEACONS)]
print("[signal] %d distress positions:" % NUM_BEACONS)
for i, p in enumerate(positions):
    print("  #%2d: (%+.2f, %+.2f)" % (i + 1, p[0], p[1]))


def teleport_to(i):
    x, y = positions[i]
    translation_field.setSFVec3f([x, y, BEACON_Z])
    print("[signal] beacon now at #%d: (%+.2f, %+.2f)" % (i + 1, x, y))


idx = 0
teleport_to(idx)

while robot.step(TIME_STEP) != -1:
    # Continuous broadcast → fresh bearing for the radio scout every tick
    emitter.send(b"SOS")

    # Rescue check: scout within RESCUE_RADIUS of the active beacon?
    sx, sy, _ = scout_node.getPosition()
    dx = positions[idx][0] - sx
    dy = positions[idx][1] - sy
    if dx * dx + dy * dy < RESCUE_RADIUS * RESCUE_RADIUS:
        print("[signal] beacon #%d RESCUED (t=%.1f s)" % (idx + 1, robot.getTime()))
        idx += 1
        if idx >= NUM_BEACONS:
            print("[signal] all %d beacons rescued — done" % NUM_BEACONS)
            break
        teleport_to(idx)
