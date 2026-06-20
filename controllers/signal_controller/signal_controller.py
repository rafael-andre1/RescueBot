"""
signal_controller — Supervisor: spawner + auctioneer for the multi-robot
rescue mission.

At startup it spawns NUM_ROBOTS scouts clustered around the arena centre,
then activates NUM_GOALS distress beacons one every BEACON_INTERVAL
seconds at random positions. Beacons accumulate: one NEVER disappears
until its assigned scout physically reaches it.

Auction (sealed-bid, blocking):
  • Each beacon broadcasts "SOS <id>" on channel 1 (its own emitter).
  • Idle scouts hear it, estimate the goal from bearing+strength, and bid
    their theoretical path cost over channel 2: "BID <beacon> <robot> <cost>".
    (Cost = known-map Dijkstra to the known cell closest to the estimate,
    plus 1.5× straight-line for the unknown remainder — computed scout-side.)
  • After AUCTION_WINDOW seconds of bidding the manager awards the beacon
    to the LOWEST fresh bid from a free robot: "AWARD <beacon> <robot>".
  • Awards are BLOCKING — the beacon belongs to that robot until rescued.
    No exchange, no re-auction. A busy robot never receives a second award.
  • Rescue = assigned robot within RESCUE_RADIUS of the beacon (measured
    with supervisor ground truth). The beacon node is then removed and the
    robot freed for future auctions.

When all NUM_GOALS beacons have been spawned and rescued, the manager
broadcasts "DONE" for a couple of seconds (so every scout saves its map
and exits) and stops.
"""
import math
import random
from controller import Supervisor

# ─── Mission parameters (the knobs you asked to be global) ──────────
NUM_ROBOTS      = 3          # scouts spawned at startup
NUM_GOALS       = 5          # distress beacons over the whole mission
BEACON_INTERVAL = 20.0       # seconds between beacon activations
AUCTION_WINDOW  = 2.0        # bid-collection time before awarding (s)
RESCUE_RADIUS   = 0.40       # assigned robot within this → rescued (m)
BID_FRESHNESS   = 3.0        # ignore bids older than this (s)
CLUSTER_RADIUS  = 0.35       # scouts drop in a ring of this radius (m)

TIME_STEP = 32

# Beacon positions are drawn inside the scouts' map bounds (with margin)
X_RANGE = (-3.5, 3.5)
Y_RANGE = (-2.5, 2.5)

robot = Supervisor()
auction_tx = robot.getDevice("auction_emitter")
auction_rx = robot.getDevice("auction_receiver")
auction_rx.enable(TIME_STEP)

root_children = robot.getRoot().getField("children")

# ─── Spawn the scouts, clustered around the centre ──────────────────
SCOUT_TEMPLATE = (
    'DEF SCOUT_%d E-puck { '
    'translation %.3f %.3f 0 rotation 0 0 1 %.3f '
    'name "scout_%d" controller "robot_controller" '
    'emitter_channel 2 receiver_channel 1 '
    'turretSlot [ '
    'Lidar { translation 0 0 0.02 horizontalResolution 360 '
    'fieldOfView 6.283185 numberOfLayers 1 maxRange 5 } '
    'Display { name "map_display" width 200 height 200 } '
    'Compass { name "compass" } '
    'Receiver { name "auction_receiver" channel 2 } '
    '] }'
)

scout_nodes = []
scout_start = []
for i in range(NUM_ROBOTS):
    if NUM_ROBOTS == 1:
        sx, sy, ang = 0.0, 0.0, 0.0
    else:
        ang = 2.0 * math.pi * i / NUM_ROBOTS
        sx = CLUSTER_RADIUS * math.cos(ang)
        sy = CLUSTER_RADIUS * math.sin(ang)
    root_children.importMFNodeFromString(
        -1, SCOUT_TEMPLATE % (i, sx, sy, ang, i))
    scout_nodes.append(robot.getFromDef("SCOUT_%d" % i))
    scout_start.append((sx, sy))
print("[manager] spawned %d scouts clustered at centre (r=%.2f m)"
      % (NUM_ROBOTS, CLUSTER_RADIUS))

# ─── Beacon spawning ─────────────────────────────────────────────────
# Colour options for beacons — picked at random on spawn
BEACON_COLOURS = {
    "red":    (1.0, 0.0, 0.0),
    "yellow": (1.0, 1.0, 0.0),
    "green":  (0.0, 1.0, 0.0),
}
COLOUR_NAMES = list(BEACON_COLOURS.keys())

BEACON_TEMPLATE = (
    'DEF BEACON_%d Robot { '
    'translation %.3f %.3f 0.05 '
    'name "beacon_%d" controller "beacon_controller" '
    'children [ '
    'Shape { appearance PBRAppearance { baseColor %.1f %.1f %.1f emissiveColor %.1f %.1f %.1f '
    'roughness 0.2 metalness 0 } geometry Box { size 0.15 0.15 0.1 } } '
    'Emitter { channel 1 } '
    '] }'
)

beacon_pos     = {}           # id → (x, y)
beacon_node    = {}           # id → supervisor node ref
beacon_spawn_t = {}           # id → sim time it appeared
beacon_colour  = {}           # id → colour name ("red", "yellow", "green")
assigned       = {}           # beacon id → robot id (locked, never changes)
rescued        = set()        # beacon ids rescued
busy_robots    = set()        # robot ids currently locked to a beacon
bids           = {}           # (beacon, robot) → (cost, t_received)
# Wall rectangles from arena.wbt: (centre_x, centre_y, half_w, half_h)
# with a safety margin so beacons don't spawn too close to walls.
WALL_MARGIN = 0.30   # extra clearance around each wall (m)
WALLS = [
    # wall_1: vertical at (1.5, 0), Box 0.05×2
    (1.5,   0.0,   0.05/2 + WALL_MARGIN, 2.0/2 + WALL_MARGIN),
    # wall_2: horizontal at (-1.5, -1), Box 2×0.05
    (-1.5, -1.0,   2.0/2 + WALL_MARGIN,  0.05/2 + WALL_MARGIN),
    # wall_3: horizontal at (0, 2), Box 1.5×0.05
    (0.0,   2.0,   1.5/2 + WALL_MARGIN,  0.05/2 + WALL_MARGIN),
    # wall_4: vertical at (-2, 0.5), Box 0.05×2
    (-2.0,  0.5,   0.05/2 + WALL_MARGIN, 2.0/2 + WALL_MARGIN),
    # wall_5: horizontal at (2, -1.5), Box 2×0.05
    (2.0,  -1.5,   2.0/2 + WALL_MARGIN,  0.05/2 + WALL_MARGIN),
    # wall_6: vertical at (2.5, 1.25), Box 0.05×2
    (2.5,   1.25,  0.05/2 + WALL_MARGIN, 2.0/2 + WALL_MARGIN),
    # wall_7: horizontal at (-1, 1), Box 1×0.05
    (-1.0,  1.0,   1.0/2 + WALL_MARGIN,  0.05/2 + WALL_MARGIN),
]


def _position_clear(x, y):
    """True if (x, y) doesn't overlap any wall rectangle (with margin)."""
    for wx, wy, hw, hh in WALLS:
        if abs(x - wx) < hw and abs(y - wy) < hh:
            return False
    return True


def _safe_position():
    """Generate a random position that doesn't overlap with any wall."""
    for _ in range(200):
        x = random.uniform(*X_RANGE)
        y = random.uniform(*Y_RANGE)
        if _position_clear(x, y):
            return x, y
    return random.uniform(*X_RANGE), random.uniform(*Y_RANGE)  # fallback


positions = [_safe_position() for _ in range(NUM_GOALS)]

# Pre-build a balanced colour pool so every colour appears roughly equally
# e.g. 5 beacons → at least 1 of each + 2 random; 7 → at least 2 of each + 1
_base   = COLOUR_NAMES * (NUM_GOALS // len(COLOUR_NAMES))
_extra  = random.sample(COLOUR_NAMES, NUM_GOALS % len(COLOUR_NAMES))
colours_pool = _base + _extra
random.shuffle(colours_pool)

print("[manager] %d distress positions:" % NUM_GOALS)
for i, p in enumerate(positions):
    print("   beacon %d: (%+.2f, %+.2f) colour=%s" % (i, p[0], p[1], colours_pool[i]))

spawned   = 0
done_sent_until = None

while robot.step(TIME_STEP) != -1:
    t = robot.getTime()

    # ── Activate the next beacon on schedule (they accumulate) ──────
    if spawned < NUM_GOALS and t >= 1.0 + spawned * BEACON_INTERVAL:
        bx, by = positions[spawned]
        col_name = colours_pool[spawned]
        cr, cg, cb = BEACON_COLOURS[col_name]
        root_children.importMFNodeFromString(
            -1, BEACON_TEMPLATE % (spawned, bx, by, spawned,
                                   cr, cg, cb, cr, cg, cb))
        beacon_pos[spawned]     = (bx, by)
        beacon_node[spawned]    = robot.getFromDef("BEACON_%d" % spawned)
        beacon_spawn_t[spawned] = t
        beacon_colour[spawned]  = col_name
        print("[manager] beacon %d ACTIVE at (%+.2f, %+.2f) colour=%s  t=%.1f s"
              % (spawned, bx, by, col_name, t))
        spawned += 1

    # ── Collect bids ─────────────────────────────────────────────────
    while auction_rx.getQueueLength() > 0:
        try:
            parts = auction_rx.getString().split()
            if len(parts) == 4 and parts[0] == "BID":
                b, r, c = int(parts[1]), int(parts[2]), float(parts[3])
                bids[(b, r)] = (c, t)
        except (ValueError, UnicodeDecodeError):
            pass
        auction_rx.nextPacket()

    # ── Award auctions (lowest fresh bid from a free robot wins) ────
    for b in list(beacon_pos):
        if b in assigned or b in rescued:
            continue
        if t - beacon_spawn_t[b] < AUCTION_WINDOW:
            continue                      # still collecting bids
        best_r, best_c = None, float("inf")
        for r in range(NUM_ROBOTS):
            if r in busy_robots:
                continue
            entry = bids.get((b, r))
            if entry is None or (t - entry[1]) > BID_FRESHNESS:
                continue
            if entry[0] < best_c:
                best_c, best_r = entry[0], r
        if best_r is not None:
            assigned[b] = best_r
            busy_robots.add(best_r)
            col = beacon_colour.get(b, "red")
            auction_tx.send(("AWARD %d %d %s" % (b, best_r, col)).encode("utf-8"))
            print("[manager] beacon %d (%s) AWARDED to scout %d (bid %.1f)"
                  % (b, col, best_r, best_c))

    # ── Rescue check: only the ASSIGNED robot can rescue its beacon ──
    for b, r in list(assigned.items()):
        if b in rescued:
            continue
        node = scout_nodes[r]
        if node is None:
            continue
        sx, sy, _ = node.getPosition()
        bx, by = beacon_pos[b]
        if (sx - bx) ** 2 + (sy - by) ** 2 < RESCUE_RADIUS ** 2:
            rescued.add(b)
            busy_robots.discard(r)
            if beacon_node[b] is not None:
                beacon_node[b].remove()
            print("[manager] beacon %d RESCUED by scout %d  t=%.1f s"
                  % (b, r, t))

    # ── Mission end: everything spawned and rescued ─────────────────
    if spawned >= NUM_GOALS and len(rescued) >= NUM_GOALS:
        if done_sent_until is None:
            done_sent_until = t + 2.0
            print("[manager] all %d beacons rescued — broadcasting DONE" % NUM_GOALS)
        if t < done_sent_until:
            auction_tx.send(b"DONE")
        else:
            break
