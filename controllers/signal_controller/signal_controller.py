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
NUM_ROBOTS      = 5        # scouts spawned at startup
NUM_GOALS       = 0          # distress beacons over the mission (0 = spawn forever)
BEACON_INTERVAL = 5.0        # seconds between beacon activations
AUCTION_WINDOW  = 2.0        # bid-collection time before the FIRST assignment (s)
RESCUE_RADIUS   = 0.40       # assigned robot within this → rescued (m)

# Scouts resolve a ping within RADIO_STOP_DIST once their camera confirms its
# colour. Beacons are spawned at least 2x that apart so a scout can never have
# two pings inside its resolution radius at once — the beacon it sees is always
# unambiguous, which is what makes opportunistic (resolve-on-sight) safe.
RADIO_STOP_DIST = 0.80
MIN_BEACON_SEP  = 2.0 * RADIO_STOP_DIST   # 1.6 m centre-to-centre
BID_FRESHNESS   = 3.0        # ignore bids older than this (s)
CLUSTER_RADIUS  = 0.35       # scouts drop in a ring of this radius (m)

# Dynamic reassignment (continuous market) — no change cap
COMMIT_COST     = 15.0       # an assignee whose bid (≈ remaining cost) is below this
                             # is "committed" — never reassigned, so a robot is never
                             # yanked off a goal it's about to reach
INF = float("inf")

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
    'controllerArgs [ "%.3f" "%.3f" ] '          # world start → shared global map frame
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
        -1, SCOUT_TEMPLATE % (i, sx, sy, ang, i, sx, sy))
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

# Same little 0.15×0.15×0.1 box as the original beacon, but FLOATING at
# MARKER_Z — above the LiDAR's single horizontal scan layer (~6 cm) — so
# it looks the same yet is never detected as a wall. No phantom wall.
MARKER_Z = 0.22                      # height above floor (LiDAR plane ≈ 0.06)
BEACON_TEMPLATE = (
    'DEF BEACON_%d Robot { '
    'translation %.3f %.3f 0.05 '
    'name "beacon_%d" controller "beacon_controller" '
    'children [ '
    'Emitter { channel 1 } '
    'Solid { translation 0 0 %.3f children [ '
    'Shape { appearance PBRAppearance { baseColor %.1f %.1f %.1f '
    'emissiveColor %.1f %.1f %.1f roughness 0.3 metalness 0 } '
    'geometry Box { size 0.15 0.15 0.1 } } ] } '
    '] }'
)

beacon_pos     = {}           # id → (x, y)
beacon_node    = {}           # id → supervisor node ref
beacon_spawn_t = {}           # id → sim time it appeared
beacon_colour  = {}           # id → colour name ("red", "yellow", "green")
rescued        = set()        # beacon ids rescued
bids           = {}           # (beacon, robot) → (cost, t_received)

# Live assignment (can change — dynamic market)
assignee       = {}           # beacon id → robot id (or absent = unassigned)
task           = {r: None for r in range(NUM_ROBOTS)}   # robot → beacon (or None)


def fresh_bid(b, r, t):
    """Robot r's most recent bid on beacon b, if fresh; else None."""
    e = bids.get((b, r))
    if e is None or (t - e[1]) > BID_FRESHNESS:
        return None
    return e[0]
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


def _safe_position(active_positions):
    """Random position clear of walls AND at least MIN_BEACON_SEP from every
    currently-active beacon. Returns None if no such spot is found (arena too
    full right now) — the caller defers the spawn rather than placing one too
    close, which would break the resolve-on-sight guarantee."""
    sep2 = MIN_BEACON_SEP ** 2
    for _ in range(200):
        x = random.uniform(*X_RANGE)
        y = random.uniform(*Y_RANGE)
        if not _position_clear(x, y):
            continue
        if all((x - ax) ** 2 + (y - ay) ** 2 >= sep2
               for (ax, ay) in active_positions):
            return x, y
    return None


print("[manager] spawning a beacon every %.0f s%s"
      % (BEACON_INTERVAL,
         " (forever)" if NUM_GOALS == 0 else " up to %d" % NUM_GOALS))

spawned   = 0
done_sent_until = None

while robot.step(TIME_STEP) != -1:
    t = robot.getTime()

    # ── Activate the next beacon on schedule (they accumulate) ──────
    #    NUM_GOALS == 0 → spawn forever; position + colour chosen per spawn.
    if ((NUM_GOALS == 0 or spawned < NUM_GOALS)
            and t >= 1.0 + spawned * BEACON_INTERVAL):
        # Keep every new beacon >= MIN_BEACON_SEP from all CURRENTLY-active ones
        # (rescued beacons are gone, so they free up space). If the arena is too
        # crowded right now, defer: leave `spawned` unchanged so we retry next
        # step and place it the moment a rescue opens room.
        active_positions = [beacon_pos[bb] for bb in beacon_pos if bb not in rescued]
        spot = _safe_position(active_positions)
        if spot is not None:
            bx, by = spot
            col_name = random.choice(COLOUR_NAMES)
            cr, cg, cb = BEACON_COLOURS[col_name]
            root_children.importMFNodeFromString(
                -1, BEACON_TEMPLATE % (spawned, bx, by, spawned, MARKER_Z,
                                       cr, cg, cb, cr, cg, cb))
            beacon_pos[spawned]      = (bx, by)
            beacon_node[spawned]     = robot.getFromDef("BEACON_%d" % spawned)
            beacon_spawn_t[spawned]  = t
            beacon_colour[spawned]   = col_name
            print("[manager] beacon %d ACTIVE at (%+.2f, %+.2f) colour=%s  t=%.1f s"
                  % (spawned, bx, by, col_name, t))
            spawned += 1

    # ── Collect bids + rescue declarations ──────────────────────────
    #   A robot declares "RESOLVED <robot> <beacon>" once it is within range
    #   AND its camera has confirmed the beacon's colour (same rule as the
    #   swarm) — we honour that rather than waiting for it to physically touch.
    resolved_decls = set()
    while auction_rx.getQueueLength() > 0:
        try:
            parts = auction_rx.getString().split()
            if len(parts) == 4 and parts[0] == "BID":
                b, r, c = int(parts[1]), int(parts[2]), float(parts[3])
                bids[(b, r)] = (c, t)
            elif len(parts) == 3 and parts[0] == "RESOLVED":
                resolved_decls.add((int(parts[1]), int(parts[2])))
        except (ValueError, UnicodeDecodeError):
            pass
        auction_rx.nextPacket()

    active = [b for b in beacon_pos if b not in rescued]

    # ── (1) Initial assignment: give each unassigned beacon to the
    #        lowest-bidding FREE robot (after a short bid-collection window) ──
    for b in active:
        if assignee.get(b) is not None:
            continue
        if t - beacon_spawn_t[b] < AUCTION_WINDOW:
            continue
        best_r, best_c = None, INF
        for r in range(NUM_ROBOTS):
            if task[r] is not None:
                continue
            c = fresh_bid(b, r, t)
            if c is not None and c < best_c:
                best_c, best_r = c, r
        if best_r is not None:
            assignee[b] = best_r
            task[best_r] = b
            print("[manager] beacon %d (%s) → scout %d (bid %.1f)"
                  % (b, beacon_colour[b], best_r, best_c))

    # ── (2) One reassignment / swap improvement per step ────────────
    #   • Free closer robot B: switch b to B if B's bid < current assignee's.
    #   • B busy on g2: swap only if the TOTAL cost drops.
    #   No change cap; committed assignees (almost there) are untouchable.
    for b in active:
        a = assignee.get(b)
        if a is None:
            continue
        ca = fresh_bid(b, a, t)                 # current assignee's cost on b
        if ca is None or ca < COMMIT_COST:
            continue                            # a is committed / no fresh bid

        cand, cand_c = None, INF                # best OTHER robot for b
        for r in range(NUM_ROBOTS):
            if r == a:
                continue
            c = fresh_bid(b, r, t)
            if c is not None and c < cand_c:
                cand_c, cand = c, r
        if cand is None:
            continue

        g2 = task[cand]
        if g2 is None:
            # Candidate is free → switch if strictly closer
            if cand_c < ca:
                task[a] = None
                assignee[b] = cand
                task[cand] = b
                print("[manager] beacon %d REASSIGNED scout %d → %d (%.1f < %.1f)"
                      % (b, a, cand, cand_c, ca))
                break
        elif g2 != b:
            # Candidate busy on g2 → swap only if total cost improves
            cb_g2 = fresh_bid(g2, cand, t)      # candidate's cost on its goal
            ca_g2 = fresh_bid(g2, a, t)         # assignee's cost on candidate's goal
            if cb_g2 is None or ca_g2 is None or cb_g2 < COMMIT_COST:
                continue
            cur_sum = ca + cb_g2
            new_sum = cand_c + ca_g2
            if new_sum < cur_sum:
                assignee[b]  = cand; task[cand] = b
                assignee[g2] = a;    task[a]    = g2
                print("[manager] SWAP beacons %d↔%d between scouts %d,%d "
                      "(sum %.1f < %.1f)" % (b, g2, a, cand, new_sum, cur_sum))
                break

    # ── (3) Broadcast each robot's current task (authoritative) ─────
    for r in range(NUM_ROBOTS):
        b = task[r]
        if b is None:
            auction_tx.send(("TASK %d -1 none" % r).encode("utf-8"))
        else:
            auction_tx.send(("TASK %d %d %s" % (r, b, beacon_colour[b])).encode("utf-8"))

    # ── (4) Rescue: a beacon is resolved when ANY scout DECLARES it (in range
    #        + camera-confirmed colour) — opportunistic, so a passing scout can
    #        resolve a beacon that isn't its assignment, or one that's still
    #        unassigned. A ground-truth proximity check on the assignee is kept
    #        only as a safety fallback so the mission can never stall. ────────
    for b in list(active):
        a = assignee.get(b)                       # current assignee (may be None)
        # Resolver = the declaring scout if any, else the assignee (fallback).
        resolver = next((rr for (rr, bb) in resolved_decls if bb == b), None)
        if resolver is None and a is not None:
            node = scout_nodes[a]
            if node is not None:
                sx, sy, _ = node.getPosition()
                bx, by = beacon_pos[b]
                if (sx - bx) ** 2 + (sy - by) ** 2 < RESCUE_RADIUS ** 2:
                    resolver = a
        if resolver is not None:
            rescued.add(b)
            assignee[b] = None
            if a is not None:
                task[a] = None                    # free the assignee (if any)
            if beacon_node[b] is not None:
                beacon_node[b].remove()
            # Credit the scout that actually resolved it.
            auction_tx.send(("RESCUED %d %d" % (resolver, b)).encode("utf-8"))
            extra = "" if resolver == a else " (opportunistic; was %s)" % (
                "scout %d" % a if a is not None else "unassigned")
            print("[manager] beacon %d RESCUED by scout %d  t=%.1f s%s"
                  % (b, resolver, t, extra))

    # ── Mission end: only in finite mode (NUM_GOALS > 0). Infinite mode
    #    runs until you stop the sim — scouts never receive DONE. ──────
    if NUM_GOALS > 0 and spawned >= NUM_GOALS and len(rescued) >= NUM_GOALS:
        if done_sent_until is None:
            done_sent_until = t + 2.0
            print("[manager] all %d beacons rescued — broadcasting DONE" % NUM_GOALS)
        if t < done_sent_until:
            auction_tx.send(b"DONE")
        else:
            break
