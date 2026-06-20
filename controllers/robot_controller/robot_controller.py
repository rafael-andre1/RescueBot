"""
scout — distress-ping rescue robot (multi-robot auction variant).

  • No GPS. Pose is dead-reckoned from wheel encoders, with compass for heading.
  • Goals are unknown — each beacon pings "SOS <id>" with a robot-relative
    bearing + signal strength. Strength → estimated range (~1/sqrt(s));
    bearing + heading → world direction → projected goal estimate.
  • AUCTION: while idle, the scout bids on every unawarded beacon it hears.
    Bid = Dijkstra cost over the KNOWN map to the known cell closest to the
    goal estimate, plus 1.5× straight-line for the unknown remainder.
    Bids go to the signal manager on channel 2; the manager awards each
    beacon to the lowest bidder. Awards are BLOCKING — once this scout wins
    a beacon it is locked to it until physically rescued. No exchange.
  • Path-finding (Dijkstra over the live occupancy grid) routes around walls
    discovered en route. NO reactive obstacle avoidance, NO spiral search.

Lifecycle: idle (sit, scan, bid) → win a beacon → home in on its pings via
the cost gradient → beacon silenced by the manager when reached → back to
idle. On "DONE" from the manager (or prolonged radio silence) the map is
saved as slam_map_<id>.png and the controller exits.
"""
from controller import Robot
import math
import heapq

#  Tuning knobs
TIME_STEP        = 32
CRUISE_SPEED     = 3.0
MAX_SPEED        = 6.28
SAFE_DISTANCE    = 0.12         # reactive override kicks in if lidar in the
                                # intended direction reads closer than this (m)

#  E-puck odometry (replaces GPS)
WHEEL_R          = 0.0205       # m, wheel radius
WHEEL_BASE       = 0.052        # m, distance between wheels (unused — heading from compass)

#  Map / grid parameters
MAP_SIZE         = 200
MAP_CENTRE       = MAP_SIZE // 2
WORLD_X_MAX      = 4.0
WORLD_Y_MAX      = 3.0
OBSTACLE_INFLATE      = 4    # cells of buffer the planner adds around walls
                             # (e-puck radius ≈ 3.5 cm, cell ≈ 3-4 cm; 4 cells
                             # = 12-16 cm — robot physically fits any planned cell)
ROBOT_FOOTPRINT_CELLS = 2    # half-width of robot's body footprint, in cells
                             # (used by footprint_clear, currently dormant while
                             # the reactive override is off)

#  Occupancy thresholds
WALL_CERTAINTY   = 0.30
IMPASSABLE       = 0.90
IMPASSABLE_COST  = 1000

#  Radio homing
GOAL_MOVE_THRESH = 0.20         # re-plan when bearing-derived goal moves > this (m)

#  Auction (channel 2; the signal manager is the auctioneer)
REBID_PERIOD       = 1.0      # while idle, refresh bids this often (s)
BID_UNKNOWN_FACTOR = 1.5      # cost multiplier for the unknown remainder of a bid
BEACON_SILENT_T    = 1.0      # my beacon silent this long → it was rescued (the
                              # manager removes the beacon node at RESCUE_RADIUS,
                              # so silence == arrival; no strength threshold needed)
SILENCE_TIMEOUT    = 8.0      # idle + no SOS at all for this long after the first
                              # ping ever → assume mission over, save map and exit

#  Robot-robot avoidance (peer heartbeats shared on channel 2)
ROBOT_FLAG         = 2        # value stamped into robot_occ for peer-occupied cells
ROBOT_AVOID_DIST   = 0.35     # peer closer than this, moving, higher priority → I stop
ROBOT_RELEASE_DIST = 0.55     # resume once that peer is farther than this (hysteresis)
ROBOT_MARK_DIST    = 0.70     # peers closer than this get stamped into the grid
ROBOT_MARK_CELLS   = 4        # half-width of the block stamped around a peer (cells)
PEER_FRESH         = 0.5      # peer data older than this is ignored (s)
AVOID_REPLAN_MIN   = 0.5      # min seconds between mark-triggered replans

#  Display
DRAW_INTERVAL    = 16

#  Neighbour offsets
STRAIGHT = 1.0
DIAG     = 1.5
INF      = float("inf")
NEIGHBOURS = [(-1, -1, DIAG), (-1, 0, STRAIGHT), (-1, 1, DIAG),
              ( 0, -1, STRAIGHT),                  ( 0, 1, STRAIGHT),
              ( 1, -1, DIAG),  ( 1, 0, STRAIGHT),  ( 1, 1, DIAG)]

# ═══════════════════════════════════════════════════════════════════
#  Webots device setup
# ═══════════════════════════════════════════════════════════════════
robot = Robot()

# Identity: spawned by the manager as "scout_<id>"
ROBOT_NAME = robot.getName()
try:
    ROBOT_ID = int(ROBOT_NAME.split("_")[-1])
except ValueError:
    ROBOT_ID = 0

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Wheel encoders for dead-reckoning (replaces GPS)
left_enc  = robot.getDevice("left wheel sensor")
right_enc = robot.getDevice("right wheel sensor")
left_enc.enable(TIME_STEP)
right_enc.enable(TIME_STEP)

lidar    = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
LIDAR_FOV = lidar.getFov()   # Webots scanline: index 0 at +FOV/2, angle decreases (CW) with index

camera   = robot.getDevice("camera")
camera.enable(TIME_STEP)

receiver = robot.getDevice("receiver")          # channel 1 — beacon SOS pings
receiver.enable(TIME_STEP)

# Auction bus (channel 2): built-in emitter sends bids, extra receiver
# hears AWARD/DONE from the manager (and other scouts' bids, ignored).
auction_tx = robot.getDevice("emitter")
auction_rx = robot.getDevice("auction_receiver")
auction_rx.enable(TIME_STEP)

compass  = robot.getDevice("compass")
compass.enable(TIME_STEP)

display  = robot.getDevice("map_display")


# ═══════════════════════════════════════════════════════════════════
#  Coordinate helpers (map origin = robot start position)
# ═══════════════════════════════════════════════════════════════════
def world_to_pix(wx, wy):
    px = int(MAP_CENTRE + (wx / WORLD_X_MAX) * MAP_CENTRE)
    py = int(MAP_CENTRE - (wy / WORLD_Y_MAX) * MAP_CENTRE)
    return max(0, min(MAP_SIZE - 1, px)), max(0, min(MAP_SIZE - 1, py))


def pix_to_world(px, py):
    wx = (px - MAP_CENTRE) / MAP_CENTRE * WORLD_X_MAX
    wy = (MAP_CENTRE - py) / MAP_CENTRE * WORLD_Y_MAX
    return wx, wy


def beam_angle(i, n):
    return (LIDAR_FOV / 2.0) - (LIDAR_FOV * i / n)


# ═══════════════════════════════════════════════════════════════════
#  MATRIX 1 — occupancy grid (hits / visits → probability)
# ═══════════════════════════════════════════════════════════════════

'''
The ocupancy grid is where we store where walls are the map is divided into 200x200 matrix
and each point either is free or ocupied but since lidar isnt perfect we have a ocupancy certeny
that tells how sure we are that that cell has a wall this is filed up as the robot move towards the goal
'''
hits   = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]
visits = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]


def get_occupancy(x, y):
    v = visits[y][x]
    if v == 0:
        return 0.0
    return hits[y][x] / v


def is_wall(x, y):
    return get_occupancy(x, y) > WALL_CERTAINTY


def get_cell_cost(x, y):
    occ = get_occupancy(x, y)
    if occ > IMPASSABLE:
        return IMPASSABLE_COST
    return 1 + int(occ / IMPASSABLE * 8)


# ═══════════════════════════════════════════════════════════════════
#  Bresenham line — trace a ray through the grid
# ═══════════════════════════════════════════════════════════════════
def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0); dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy: err -= dy; x0 += sx
        if e2 < dx:  err += dx; y0 += sy


# ═══════════════════════════════════════════════════════════════════
#  LiDAR scan → update Matrix 1.  Returns True if any cell changed STATE.
# ═══════════════════════════════════════════════════════════════════
def scan_lidar(robot_x, robot_y, heading, ranges):
    state_changed = False
    n = len(ranges)
    max_range = lidar.getMaxRange() - 0.05
    rpx, rpy = world_to_pix(robot_x, robot_y)

    for i, r in enumerate(ranges):
        if r <= 0.0 or math.isinf(r) or math.isnan(r):
            continue
        angle  = heading + beam_angle(i, n)
        is_hit = r <= max_range
        ray_len = r if is_hit else max_range
        hx = robot_x + ray_len * math.cos(angle)
        hy = robot_y + ray_len * math.sin(angle)
        epx, epy = world_to_pix(hx, hy)

        prev_wall = is_wall(epx, epy) if is_hit else False
        for cx, cy in bresenham(rpx, rpy, epx, epy):
            if not (0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE):
                continue
            if cx == epx and cy == epy and is_hit:
                hits[cy][cx] += 1
                visits[cy][cx] += 1
            else:
                visits[cy][cx] += 1

        if is_hit and is_wall(epx, epy) != prev_wall:
            state_changed = True

    return state_changed


# ═══════════════════════════════════════════════════════════════════
#  Inflation — rebuilt whenever the cost matrix is recomputed
# ═══════════════════════════════════════════════════════════════════
"""
We also have a cost matrix that originates from the goal but since we dont know where the goal is
we use the radio signal strengh to roughly deetermine the distance of the goal and compute from there
we use 1 for straight lines 1.5 for diagonals and we go towards the cell with the smallest value in the 8
that are in vecinity of the robot and each time a wall that we didnt know existem we recompute the cost
for the cell that just became a wall so the cost to goal going up automaticaly makes the robot re-rote aroud the wall
we also set cells close to walls as likely walls for safety due to lidar imperfections 
"""
inflated = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]


def inflate_obstacles():
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            inflated[y][x] = 0
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            if is_wall(x, y):
                for dy in range(-OBSTACLE_INFLATE, OBSTACLE_INFLATE + 1):
                    for dx in range(-OBSTACLE_INFLATE, OBSTACLE_INFLATE + 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < MAP_SIZE and 0 <= nx < MAP_SIZE:
                            inflated[ny][nx] = 1


# ═══════════════════════════════════════════════════════════════════
#  Robot flag layer — peers stamped into the grid as ROBOT_FLAG (2)
#  Separate from walls: transient, cleared/re-stamped as peers move.
#  The planner treats flagged cells as blocked, so the path naturally
#  goes AROUND a stopped peer robot.
# ═══════════════════════════════════════════════════════════════════
robot_occ   = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]
robot_marks = set()


def stamp_robot_marks(positions):
    """Stamp a block of ROBOT_FLAG cells around each peer position.
    positions: list of (wx, wy). Returns True if the marked set changed."""
    global robot_marks
    new_marks = set()
    for (wx, wy) in positions:
        cpx, cpy = world_to_pix(wx, wy)
        for dy in range(-ROBOT_MARK_CELLS, ROBOT_MARK_CELLS + 1):
            for dx in range(-ROBOT_MARK_CELLS, ROBOT_MARK_CELLS + 1):
                nx, ny = cpx + dx, cpy + dy
                if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                    new_marks.add((nx, ny))
    if new_marks == robot_marks:
        return False
    for (x, y) in robot_marks:
        robot_occ[y][x] = 0
    for (x, y) in new_marks:
        robot_occ[y][x] = ROBOT_FLAG
    robot_marks = new_marks
    return True


# ═══════════════════════════════════════════════════════════════════
#  MATRIX 2 — bidirectional Dijkstra path from robot to goal
#  The cost matrix is no longer a full flood-fill; BD explores roughly
#  half the area of unidirectional Dijkstra, then we write the resulting
#  path into the cost matrix (cost decreases along the path toward goal)
#  so the existing follow_gradient walks it.
# ═══════════════════════════════════════════════════════════════════
cost = [[INF] * MAP_SIZE for _ in range(MAP_SIZE)]


def _snap_to_free(px, py, radius):
    """If (px,py) is blocked, return the nearest free cell within radius, else (px,py)."""
    if (inflated[py][px] == 0 and robot_occ[py][px] == 0
            and get_cell_cost(px, py) < IMPASSABLE_COST):
        return px, py
    best, best_d = None, INF
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx, ny = px + dx, py + dy
            if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                if (inflated[ny][nx] == 0 and robot_occ[ny][nx] == 0
                        and get_cell_cost(nx, ny) < IMPASSABLE_COST):
                    d = abs(dx) + abs(dy)
                    if d < best_d:
                        best_d = d
                        best = (nx, ny)
    return best  # may be None


def bidirectional_dijkstra(start, goal):
    """Shortest path on the inflated grid from start to goal (cells).
    Edge u→v weight = move_dist(u,v) * cell_cost(v) (cost-of-destination model).
    Returns [start, ..., goal] or None if unreachable."""
    if start == goal:
        return [start]

    sx, sy = start
    gx, gy = goal
    dist_f   = {start: 0.0}
    dist_b   = {goal:  0.0}
    parent_f = {}
    parent_b = {}
    pq_f = [(0.0, sx, sy)]
    pq_b = [(0.0, gx, gy)]
    settled_f, settled_b = set(), set()
    best_total = INF
    meeting    = None

    while pq_f and pq_b:
        # Termination: any unexplored path costs ≥ min(pq_f) + min(pq_b)
        if pq_f[0][0] + pq_b[0][0] >= best_total:
            break

        if pq_f[0][0] <= pq_b[0][0]:
            d, ux, uy = heapq.heappop(pq_f)
            if (ux, uy) in settled_f:
                continue
            settled_f.add((ux, uy))
            if (ux, uy) in dist_b:
                tot = d + dist_b[(ux, uy)]
                if tot < best_total:
                    best_total = tot
                    meeting = (ux, uy)
            for dx, dy, md in NEIGHBOURS:
                nx, ny = ux + dx, uy + dy
                if not (0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE):
                    continue
                if inflated[ny][nx] == 1 or robot_occ[ny][nx] != 0:
                    continue
                cc = get_cell_cost(nx, ny)
                if cc >= IMPASSABLE_COST:
                    continue
                nd = d + md * cc       # forward edge: weight = md * cost(destination)
                if nd < dist_f.get((nx, ny), INF):
                    dist_f[(nx, ny)]   = nd
                    parent_f[(nx, ny)] = (ux, uy)
                    heapq.heappush(pq_f, (nd, nx, ny))
        else:
            d, ux, uy = heapq.heappop(pq_b)
            if (ux, uy) in settled_b:
                continue
            settled_b.add((ux, uy))
            if (ux, uy) in dist_f:
                tot = dist_f[(ux, uy)] + d
                if tot < best_total:
                    best_total = tot
                    meeting = (ux, uy)
            cc_u = get_cell_cost(ux, uy)
            for dx, dy, md in NEIGHBOURS:
                nx, ny = ux + dx, uy + dy
                if not (0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE):
                    continue
                if inflated[ny][nx] == 1 or robot_occ[ny][nx] != 0:
                    continue
                if get_cell_cost(nx, ny) >= IMPASSABLE_COST:
                    continue
                # Reverse edge n→u in original has weight md * cost(u)
                nd = d + md * cc_u
                if nd < dist_b.get((nx, ny), INF):
                    dist_b[(nx, ny)]   = nd
                    parent_b[(nx, ny)] = (ux, uy)
                    heapq.heappush(pq_b, (nd, nx, ny))

    if meeting is None:
        return None

    # Reconstruct: forward parents from meeting back to start
    path_f = [meeting]
    cur = meeting
    while cur != start:
        cur = parent_f.get(cur)
        if cur is None:
            return None
        path_f.append(cur)
    path_f.reverse()

    # Then backward parents from meeting forward to goal
    path_b = []
    cur = meeting
    while cur != goal:
        cur = parent_b.get(cur)
        if cur is None:
            return None
        path_b.append(cur)

    return path_f + path_b


def compute_cost_matrix():
    """BD path from robot to goal → written into cost[][] so follow_gradient walks it.
    Cells off the path stay INF (they're not relevant for this single trajectory)."""
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            cost[y][x] = INF

    start = _snap_to_free(*world_to_pix(robot_x, robot_y), radius=5)
    goal  = _snap_to_free(*world_to_pix(goal_x,  goal_y),  radius=15)
    if start is None or goal is None:
        return

    path = bidirectional_dijkstra(start, goal)
    if path is None:
        return

    # Cost decreases monotonically from start (high) to goal (0) so follow_gradient
    # walks downhill along the path.
    last = len(path) - 1
    for i, (px, py) in enumerate(path):
        cost[py][px] = float(last - i)


def replan():
    inflate_obstacles()
    compute_cost_matrix()


# ═══════════════════════════════════════════════════════════════════
#  Radio homing — turn the strongest "HELP" ping into a world-frame goal
# ═══════════════════════════════════════════════════════════════════
def read_sos_packets():
    """Drain the SOS receiver (channel 1). Returns {beacon_id: (strength,
    bearing_rel)} keeping the strongest packet per beacon this step."""
    packets = {}
    while receiver.getQueueLength() > 0:
        try:
            msg = receiver.getString()
            parts = msg.split()
            if parts and parts[0] == "SOS":
                bid_id = int(parts[1]) if len(parts) > 1 else 0
                s = receiver.getSignalStrength()
                d = receiver.getEmitterDirection()
                b = math.atan2(d[1], d[0])       # robot-relative bearing
                if bid_id not in packets or s > packets[bid_id][0]:
                    packets[bid_id] = (s, b)
        except (ValueError, UnicodeDecodeError):
            pass
        receiver.nextPacket()
    return packets


def project_goal(robot_x, robot_y, heading, strength, bearing_rel):
    """Strength + bearing → estimated goal position in this scout's frame."""
    rng = 1.0 / math.sqrt(strength)              # Webots default ~1/d² emission
    world_b = heading + bearing_rel
    return (robot_x + rng * math.cos(world_b),
            robot_y + rng * math.sin(world_b))


# ═══════════════════════════════════════════════════════════════════
#  Auction bid — theoretical cost to a goal estimate
#  Dijkstra over the KNOWN map only (cells we've actually seen), then
#  from the known cell closest to the goal, infer the unknown remainder
#  as straight-line distance at BID_UNKNOWN_FACTOR (1.5×) cost.
# ═══════════════════════════════════════════════════════════════════
def known_flood(rpx, rpy):
    """Dijkstra from the robot's cell over KNOWN free cells (visits > 0,
    not wall, not inflated). Returns {(x, y): path_cost}."""
    dist = {(rpx, rpy): 0.0}
    pq = [(0.0, rpx, rpy)]
    while pq:
        c, cx, cy = heapq.heappop(pq)
        if c > dist.get((cx, cy), INF):
            continue
        for dx, dy, md in NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE):
                continue
            if visits[ny][nx] == 0:              # unknown — not part of known map
                continue
            if is_wall(nx, ny) or inflated[ny][nx] == 1 or robot_occ[ny][nx] != 0:
                continue
            cc = get_cell_cost(nx, ny)
            if cc >= IMPASSABLE_COST:
                continue
            nc = c + md * cc
            if nc < dist.get((nx, ny), INF):
                dist[(nx, ny)] = nc
                heapq.heappush(pq, (nc, nx, ny))
    return dist


def bid_for_goal(flood, gwx, gwy):
    """Bid = min over known cells of (path cost there + 1.5 × straight-line
    remainder to the goal estimate), in cell units. The robot's own cell is
    in the flood with cost 0, so 'no known path helps' is covered too."""
    gpx, gpy = world_to_pix(gwx, gwy)
    best = INF
    for (cx, cy), c in flood.items():
        rem = math.hypot(cx - gpx, cy - gpy)
        tot = c + BID_UNKNOWN_FACTOR * rem
        if tot < best:
            best = tot
    return best


# ═══════════════════════════════════════════════════════════════════
#  Line-of-Sight & Lookahead Target (Path Smoothing)
# ═══════════════════════════════════════════════════════════════════
def has_line_of_sight(x0, y0, x1, y1):
    """
    Check if there is a clear line of sight between two points
    (no obstacles or unknown areas).
    """
    for cx, cy in bresenham(x0, y0, x1, y1):
        if not (0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE):
            return False
        # If unexplored, wall, inflated obstacle, or another robot
        if visits[cy][cx] == 0 or is_wall(cx, cy) or inflated[cy][cx] == 1 or robot_occ[cy][cx] != 0:
            return False
    return True


def get_lookahead_target(rpx, rpy, heading):
    """
    Finds the furthest point on the path (following the lowest cost)
    that still has a direct line of sight from the robot.
    """
    target_px, target_py = follow_gradient(rpx, rpy, heading)
    if (target_px, target_py) == (rpx, rpy):
        return rpx, rpy

    current_px, current_py = target_px, target_py
    MAX_LOOKAHEAD = 20  # Maximum number of cells to predict ahead

    for _ in range(MAX_LOOKAHEAD):
        min_c = INF
        next_px, next_py = current_px, current_py

        # Look for the neighbor with the lowest cost
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = current_px + dx, current_py + dy
                if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                    c = cost[ny][nx]
                    if c < min_c:
                        min_c = c
                        next_px, next_py = nx, ny

        # Stop if we hit a local minimum or the goal
        if min_c >= cost[current_py][current_px]:
            break

        current_px, current_py = next_px, next_py

        # Check if we still have a straight line of sight to this new point
        if has_line_of_sight(rpx, rpy, current_px, current_py):
            target_px, target_py = current_px, current_py
        else:
            # Obstacle broke the line of sight; use the last valid target
            break

    return target_px, target_py


# ═══════════════════════════════════════════════════════════════════
#  Gradient follower — trace several cells ahead
# ═══════════════════════════════════════════════════════════════════
def follow_gradient(rpx, rpy, heading):
    """Pick the next target cell from the 16-cell area around the robot.
    The robot's footprint is ~2×2 cells, so the candidate area is 4×4 = 16
    cells (1-cell margin around the footprint in every direction).
    Picks the cell with the lowest cost; on ties, picks the one requiring
    the least heading change — so a straight valley in the cost field
    produces straight-line motion."""
    candidates = []
    for dy in (-1, 0, 1, 2):
        for dx in (-1, 0, 1, 2):
            nx, ny = rpx + dx, rpy + dy
            if not (0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE):
                continue
            candidates.append((cost[ny][nx], dx, dy, nx, ny))

    if not candidates:
        return rpx, rpy

    min_c = min(c for (c, _, _, _, _) in candidates)
    if min_c == INF:
        return rpx, rpy              # nothing reachable, hold position

    tied = [t for t in candidates if t[0] == min_c]
    if len(tied) == 1:
        return tied[0][3], tied[0][4]

    # Tie-break: least heading change (max cos(bearing − heading)).
    # Skip the "stay" candidate (dx=dy=0) if there's any tied move.
    rwx, rwy = pix_to_world(rpx, rpy)
    best = (rpx, rpy)
    best_align = -INF
    for _, dx, dy, nx, ny in tied:
        if dx == 0 and dy == 0:
            continue
        nwx, nwy = pix_to_world(nx, ny)
        bearing = math.atan2(nwy - rwy, nwx - rwx)
        align = math.cos(bearing - heading)
        if align > best_align:
            best_align = align
            best = (nx, ny)
    return best


# ═══════════════════════════════════════════════════════════════════
#  Reactive safety override
#  If the gradient's intended direction would push the robot below
#  SAFE_DISTANCE of a wall (per current LiDAR), override the target by
#  picking the lowest-cost neighbour cell that DOES have safe clearance.
#  The override is naturally transient — it's re-evaluated every step,
#  so the moment the intended direction is safe again the planner takes
#  over without state.
# ═══════════════════════════════════════════════════════════════════
def footprint_clear(cx, cy):
    """True if the robot's body footprint around (cx, cy) is free of walls and
    inflated cells. The footprint is (2N+1)×(2N+1) cells with N=ROBOT_FOOTPRINT_CELLS,
    so it captures the fact that the robot is bigger than one cell."""
    for dy in range(-ROBOT_FOOTPRINT_CELLS, ROBOT_FOOTPRINT_CELLS + 1):
        for dx in range(-ROBOT_FOOTPRINT_CELLS, ROBOT_FOOTPRINT_CELLS + 1):
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE):
                return False
            if is_wall(nx, ny) or inflated[ny][nx] == 1:
                return False
    return True


def lidar_at_bearing(ranges, bearing_rel, half_window=3):
    """Min LiDAR range in a small arc around the given robot-relative bearing."""
    if not ranges:
        return INF
    n = len(ranges)
    # beam_angle(i, n) = FOV/2 - FOV*i/n  ⇒  i = (FOV/2 - bearing) * n/FOV
    center = int(round((LIDAR_FOV / 2.0 - bearing_rel) * n / LIDAR_FOV)) % n
    min_r = INF
    for offset in range(-half_window, half_window + 1):
        r = ranges[(center + offset) % n]
        if r > 0.0 and not math.isinf(r) and not math.isnan(r) and r < min_r:
            min_r = r
    return min_r


def safe_target_override(rpx, rpy, robot_x, robot_y, heading, ranges, intended):
    """If the intended direction is unsafe, return the lowest-cost neighbour
    cell that has clearance > SAFE_DISTANCE. Otherwise return `intended`.

    Returns (target_cell, was_overridden)."""
    # 1) Check the intended direction first.
    iwx, iwy = pix_to_world(*intended)
    bearing_world = math.atan2(iwy - robot_y, iwx - robot_x)
    bearing_rel   = bearing_world - heading
    while bearing_rel >  math.pi: bearing_rel -= 2.0 * math.pi
    while bearing_rel < -math.pi: bearing_rel += 2.0 * math.pi
    intended_clear = (lidar_at_bearing(ranges, bearing_rel) >= SAFE_DISTANCE
                      and footprint_clear(intended[0], intended[1]))
    if intended_clear:
        return intended, False

    # 2) Intended is unsafe — pick the lowest-cost neighbour that satisfies
    #    BOTH the LiDAR clearance check AND the footprint check (so the
    #    robot's body actually fits there, even though it spans several cells).
    best_c, best = INF, None
    for dx, dy, _ in NEIGHBOURS:
        nx, ny = rpx + dx, rpy + dy
        if not (0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE):
            continue
        c = cost[ny][nx]
        if c == INF:
            continue
        if not footprint_clear(nx, ny):
            continue
        nwx, nwy = pix_to_world(nx, ny)
        b_world = math.atan2(nwy - robot_y, nwx - robot_x)
        b_rel   = b_world - heading
        while b_rel >  math.pi: b_rel -= 2.0 * math.pi
        while b_rel < -math.pi: b_rel += 2.0 * math.pi
        if lidar_at_bearing(ranges, b_rel) < SAFE_DISTANCE:
            continue
        if c < best_c:
            best_c, best = c, (nx, ny)
    if best is None:
        return intended, False   # nothing safe — fall back to plan, scout will likely stop
    return best, True


# ═══════════════════════════════════════════════════════════════════
#  Differential-drive steering (proportional)
# ═══════════════════════════════════════════════════════════════════
def steer_to(robot_x, robot_y, heading, target_wx, target_wy):
    dx = target_wx - robot_x
    dy = target_wy - robot_y
    dist = math.hypot(dx, dy)
    if dist < 0.01:
        return 0.0, 0.0
    desired = math.atan2(dy, dx)
    error = desired - heading
    while error >  math.pi: error -= 2.0 * math.pi
    while error < -math.pi: error += 2.0 * math.pi
    K_TURN  = 4.0
    omega   = K_TURN * error
    forward = CRUISE_SPEED * max(0.0, 1.0 - 2.0 * abs(error) / math.pi)
    lv = max(-MAX_SPEED, min(MAX_SPEED, forward - omega))
    rv = max(-MAX_SPEED, min(MAX_SPEED, forward + omega))
    return lv, rv


# ═══════════════════════════════════════════════════════════════════
#  Display
# ═══════════════════════════════════════════════════════════════════
def save_map_image(robot_maps, filename="slam_map.png"):
    """Render a polished SLAM map (matplotlib). Mirrors the export from
    robot_controller_radio.py — multi-robot-ready, but currently called with
    a single-element list.

    robot_maps: list[dict] with keys:
        hits, visits     – MAP_SIZE × MAP_SIZE 2-D lists
        origin           – (wx, wy) of this robot's starting position
        trajectory       – list of (wx, wy)
        pinger_positions – list of (wx, wy)  (places this robot rescued)
        label            – display name
    """
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.lines import Line2D
    except ImportError:
        print("[scout] install matplotlib + numpy:  pip install matplotlib numpy")
        return

    # ── Merge all robots into one global occupancy grid ──────────────
    g_hits   = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
    g_visits = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
    for rm in robot_maps:
        ox, oy = rm["origin"]
        off_px =  round(ox / WORLD_X_MAX * MAP_CENTRE)
        off_py = -round(oy / WORLD_Y_MAX * MAP_CENTRE)
        h = np.array(rm["hits"],   dtype=np.int32)
        v = np.array(rm["visits"], dtype=np.int32)
        sx0 = max(0, -off_px); sx1 = min(MAP_SIZE, MAP_SIZE - off_px)
        dx0 = max(0,  off_px); dx1 = min(MAP_SIZE, MAP_SIZE + off_px)
        sy0 = max(0, -off_py); sy1 = min(MAP_SIZE, MAP_SIZE - off_py)
        dy0 = max(0,  off_py); dy1 = min(MAP_SIZE, MAP_SIZE + off_py)
        if sx0 < sx1 and sy0 < sy1:
            g_hits  [dy0:dy1, dx0:dx1] += h[sy0:sy1, sx0:sx1]
            g_visits[dy0:dy1, dx0:dx1] += v[sy0:sy1, sx0:sx1]

    with np.errstate(divide="ignore", invalid="ignore"):
        occ = np.where(g_visits > 0, g_hits / g_visits, np.nan)

    # Light-grey unexplored, white explored-free, dark-grey-ish walls.
    img = np.full((MAP_SIZE, MAP_SIZE, 4), [0.93, 0.93, 0.93, 1.0], dtype=np.float32)
    free_mask = (~np.isnan(occ)) & (occ <= WALL_CERTAINTY)
    img[free_mask] = [1.0, 1.0, 1.0, 1.0]
    wall_mask = (~np.isnan(occ)) & (occ > WALL_CERTAINTY)
    t_wall = np.clip(
        (occ[wall_mask] - WALL_CERTAINTY) / (IMPASSABLE - WALL_CERTAINTY), 0, 1)
    shade = 0.35 * (1.0 - t_wall)
    img[wall_mask, 0] = shade
    img[wall_mask, 1] = shade
    img[wall_mask, 2] = shade

    fig, ax = plt.subplots(figsize=(8, 7), dpi=150)
    ax.imshow(img, origin="upper",
              extent=[-WORLD_X_MAX, WORLD_X_MAX, -WORLD_Y_MAX, WORLD_Y_MAX],
              aspect="equal", interpolation="nearest")
    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Y (m)", fontsize=9)
    n = len(robot_maps)
    ax.set_title("SLAM Map — %d robot%s" % (n, "s" if n != 1 else ""), fontsize=11)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.4, color="#888888")

    legend_handles = [
        mpatches.Patch(facecolor=(0.20, 0.20, 0.20), label="Wall"),
        mpatches.Patch(facecolor=(1.00, 1.00, 1.00),
                       edgecolor="gray", linewidth=0.5, label="Free (explored)"),
        mpatches.Patch(facecolor=(0.93, 0.93, 0.93),
                       edgecolor="gray", linewidth=0.5, label="Unexplored"),
    ]

    colors = plt.cm.tab10.colors
    for i, rm in enumerate(robot_maps):
        c     = colors[i % len(colors)]
        lbl   = rm.get("label", "Robot %d" % (i + 1))
        ox, oy = rm["origin"]
        traj  = rm.get("trajectory", [])
        ppos  = rm.get("pinger_positions", [])

        if len(traj) > 1:
            tx, ty = zip(*traj)
            ax.plot(tx, ty, color=c, linewidth=1.1, alpha=0.85, zorder=3)
            legend_handles.append(
                Line2D([0], [0], color=c, linewidth=1.5,
                       label="%s trajectory" % lbl))

        ax.plot(ox, oy, marker="P", color=c, markersize=9,
                markeredgecolor="white", markeredgewidth=0.7, zorder=5)
        legend_handles.append(
            Line2D([0], [0], marker="P", color="w", markerfacecolor=c,
                   markersize=8, label="%s start" % lbl))

        if ppos:
            px = [p[0] for p in ppos]
            py = [p[1] for p in ppos]
            ax.plot(px, py, linestyle="none", marker="*", color="crimson",
                    markersize=14, zorder=6,
                    markeredgecolor="darkred", markeredgewidth=0.5)
            legend_handles.append(
                Line2D([0], [0], marker="*", color="w",
                       markerfacecolor="crimson", markersize=10,
                       label="%s rescues" % lbl))

    ax.legend(handles=legend_handles, loc="upper left",
              fontsize=7, framealpha=0.9, edgecolor="gray")
    fig.tight_layout()
    fig.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[scout] map saved → " + filename)


def draw_map(robot_px, robot_py, goal_px, goal_py,
             target_px=None, target_py=None):
    display.setColor(0xDDDDDD)
    display.fillRectangle(0, 0, MAP_SIZE, MAP_SIZE)
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            occ = get_occupancy(x, y)
            if occ > IMPASSABLE:
                display.setColor(0x000000)
            elif occ > WALL_CERTAINTY:
                t = (occ - WALL_CERTAINTY) / (IMPASSABLE - WALL_CERTAINTY)
                display.setColor((int(180 + 75 * t) << 16) | (int(120 * (1.0 - t)) << 8))
            elif robot_occ[y][x] != 0:
                display.setColor(0xCC44CC)       # peer robot flag (transient)
            elif inflated[y][x] == 1:
                display.setColor(0x444444)
            elif visits[y][x] > 0:
                g = int(180 + 60 * (1.0 - occ / WALL_CERTAINTY))
                display.setColor((40 << 16) | (g << 8) | 40)
            else:
                continue
            display.drawPixel(x, y)
    if target_px is not None and target_py is not None:
        display.setColor(0x00FFFF)
        display.drawLine(robot_px, robot_py, target_px, target_py)
        display.setColor(0xFFFF00)
        display.fillOval(target_px, target_py, 2, 2)
    display.setColor(0xFF0000)
    display.fillOval(goal_px, goal_py, 3, 3)
    display.setColor(0x0000FF)
    display.fillOval(robot_px, robot_py, 3, 3)


# ═══════════════════════════════════════════════════════════════════
#  Main loop
# ═══════════════════════════════════════════════════════════════════
"""
So the full logic is we get one or multiple help requests and we go towards the one with the strongest signal (closer),
we estimate the distance from the signal and update it once it changes significanty due to noise in the signal then you
calculate the cost matrix and the ocupancy matrix and the robot moves to the cell of the 8 that are adjent to the robot
one its close to the goal mesured from the signal strengh he arrived
"""
prev_cell     = (-1, -1)
step_count    = 0
goal_x, goal_y = 0.0, 0.0          # placeholder; set when my beacon is heard
robot_x, robot_y = 0.0, 0.0        # dead-reckoned pose, map origin = start
prev_left_enc  = None              # initialised on first iteration
prev_right_enc = None
rescues_completed  = 0             # how many beacons this scout rescued
origin_x, origin_y = 0.0, 0.0      # dead-reckoned frame origin (always 0,0 here)
trajectory         = []            # list of (wx, wy) — robot path for the map
rescued_positions  = []            # list of (wx, wy) — where each rescue happened
last_ping_time     = 0.0           # sim_time of the most recent SOS packet (any beacon)
any_ping_ever      = False         # have we heard at least one SOS yet?

# Auction state
assigned_id   = None               # beacon id this scout is locked to (None = idle)
have_goal     = False              # first estimate of my beacon received?
taken         = set()              # beacon ids awarded to anyone (incl. me)
last_rebid    = -1e9               # sim_time of last bid broadcast
my_last_ping  = 0.0                # sim_time my assigned beacon was last heard
done_received = False              # manager said the mission is over

# Peer-avoidance state
peers             = {}             # id → {pos, range, state, t}
yielding_to       = None           # peer id I'm stopped for (None = driving freely)
last_marks_replan = -1e9           # throttle mark-triggered replans

print("[scout %d] online — idle, listening for SOS to bid on" % ROBOT_ID)

while robot.step(TIME_STEP) != -1:
    step_count += 1
    sim_time    = step_count * TIME_STEP / 1000.0
    map_changed = False

    # ── Heading from compass (unchanged) ────────────────────────────
    compass_vals = compass.getValues()
    heading      = math.atan2(compass_vals[0], compass_vals[1])

    # ── Pose: dead-reckon from wheel encoders ───────────────────────
    le = left_enc.getValue()
    re = right_enc.getValue()
    if prev_left_enc is None:                # first step → just remember
        prev_left_enc, prev_right_enc = le, re
    else:
        dleft  = (le - prev_left_enc)  * WHEEL_R
        dright = (re - prev_right_enc) * WHEEL_R
        linear = (dleft + dright) / 2.0
        robot_x += linear * math.cos(heading)
        robot_y += linear * math.sin(heading)
        prev_left_enc, prev_right_enc = le, re

    robot_px, robot_py = world_to_pix(robot_x, robot_y)
    goal_px, goal_py   = world_to_pix(goal_x, goal_y)
    current_cell = (robot_px, robot_py)
    ranges       = lidar.getRangeImage()

    # ── SLAM: update map and replan whenever a cell changes state ───
    if current_cell != prev_cell:
        prev_cell = current_cell
        trajectory.append((robot_x, robot_y))     # for the saved map
        state_changed = False
        if ranges:
            state_changed = scan_lidar(robot_x, robot_y, heading, ranges)
        # Replan if (a) a cell flipped state, or (b) we've drifted off the BD path
        if have_goal and (state_changed or cost[robot_py][robot_px] == INF):
            replan()
            map_changed = True

    # ── Radio: per-beacon SOS packets (channel 1) ────────────────────
    packets = read_sos_packets()
    if packets:
        any_ping_ever  = True
        last_ping_time = sim_time

    # ── Heartbeat: tell the other scouts where I am and if I'm moving ─
    my_state = 1 if (assigned_id is not None and have_goal
                     and yielding_to is None) else 0
    auction_tx.send(("POS %d %d" % (ROBOT_ID, my_state)).encode("utf-8"))

    # ── Auction bus (channel 2): AWARD / DONE / peer POS heartbeats ──
    while auction_rx.getQueueLength() > 0:
        try:
            parts = auction_rx.getString().split()
            if parts and parts[0] == "AWARD" and len(parts) == 3:
                b, r = int(parts[1]), int(parts[2])
                taken.add(b)
                if r == ROBOT_ID:
                    assigned_id  = b
                    have_goal    = False
                    my_last_ping = sim_time
                    print("[scout %d] WON beacon %d — locked until rescued"
                          % (ROBOT_ID, b))
            elif parts and parts[0] == "POS" and len(parts) == 3:
                pid, pstate = int(parts[1]), int(parts[2])
                if pid != ROBOT_ID:
                    s = auction_rx.getSignalStrength()
                    d = auction_rx.getEmitterDirection()
                    if s > 0:
                        rng  = 1.0 / math.sqrt(s)
                        brg  = math.atan2(d[1], d[0])
                        wb   = heading + brg
                        peers[pid] = {
                            "pos":   (robot_x + rng * math.cos(wb),
                                      robot_y + rng * math.sin(wb)),
                            "range": rng,
                            "state": pstate,
                            "t":     sim_time,
                        }
            elif parts and parts[0] == "DONE":
                done_received = True
        except (ValueError, UnicodeDecodeError):
            pass
        auction_rx.nextPacket()

    # ── Peer avoidance ───────────────────────────────────────────────
    # Rule: in a close encounter only ONE robot moves. Priority = lower id
    # (an idle/stopped peer never has priority — it's already standing still).
    # The yielding robot freezes; the moving robot stamps the frozen peer
    # into the grid (robot_occ = 2) and its planner routes AROUND it.
    if assigned_id is not None and have_goal:
        if yielding_to is None:
            for pid, p in peers.items():
                if (sim_time - p["t"] < PEER_FRESH and p["state"] == 1
                        and pid < ROBOT_ID and p["range"] < ROBOT_AVOID_DIST):
                    yielding_to = pid
                    print("[scout %d] yielding to scout %d (%.2f m)"
                          % (ROBOT_ID, pid, p["range"]))
                    break
        else:
            p = peers.get(yielding_to)
            if (p is None or sim_time - p["t"] > 2.0 * PEER_FRESH
                    or p["state"] != 1 or p["range"] > ROBOT_RELEASE_DIST):
                print("[scout %d] resuming (scout %s cleared)"
                      % (ROBOT_ID, yielding_to))
                yielding_to = None
    else:
        yielding_to = None

    # Stamp nearby peers I must route around: stopped ones, or lower-priority
    # movers (they will yield to me). Never stamp a peer I'm yielding to —
    # I'm standing still anyway and it would poison my map while it passes.
    mark_positions = [p["pos"] for pid, p in peers.items()
                      if sim_time - p["t"] < PEER_FRESH
                      and pid != yielding_to
                      and p["range"] < ROBOT_MARK_DIST
                      and (pid > ROBOT_ID or p["state"] == 0)]
    if stamp_robot_marks(mark_positions):
        if have_goal and sim_time - last_marks_replan > AVOID_REPLAN_MIN:
            replan()
            map_changed = True
            last_marks_replan = sim_time

    # ── End-of-mission: manager said DONE, or radio went dead ───────
    if done_received or (any_ping_ever and assigned_id is None
                         and (sim_time - last_ping_time) > SILENCE_TIMEOUT):
        print("[scout %d] mission complete — %d rescue%s"
              % (ROBOT_ID, rescues_completed,
                 "s" if rescues_completed != 1 else ""))
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        draw_map(robot_px, robot_py, goal_px, goal_py)
        save_map_image([{
            "hits": hits, "visits": visits,
            "origin": (origin_x, origin_y),
            "trajectory": trajectory,
            "pinger_positions": rescued_positions,
            "label": "Scout %d" % ROBOT_ID,
        }], filename="slam_map_%d.png" % ROBOT_ID)
        break

    # ── ASSIGNED: home in on my beacon only ─────────────────────────
    if assigned_id is not None:
        if assigned_id in packets:
            my_last_ping = sim_time
            s, b = packets[assigned_id]
            gx, gy = project_goal(robot_x, robot_y, heading, s, b)
            if (not have_goal
                    or math.hypot(gx - goal_x, gy - goal_y) > GOAL_MOVE_THRESH):
                goal_x, goal_y = gx, gy
                have_goal = True
                replan()
                map_changed = True
                goal_px, goal_py = world_to_pix(goal_x, goal_y)

        elif sim_time - my_last_ping > BEACON_SILENT_T:
            # My beacon went silent → the manager removed it → rescued.
            rescues_completed += 1
            rescued_positions.append((robot_x, robot_y))
            print("[scout %d] beacon %d rescued (#%d for me, t=%.1f s) — back to idle"
                  % (ROBOT_ID, assigned_id, rescues_completed, sim_time))
            assigned_id = None
            have_goal   = False
            goal_x, goal_y = 0.0, 0.0
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

        if assigned_id is not None and have_goal and yielding_to is None:
            # Drive: if the goal is directly visible, go straight there;
            # otherwise use gradient descent with Line-of-Sight lookahead.
            if has_line_of_sight(robot_px, robot_py, goal_px, goal_py):
                lookahead_px, lookahead_py = goal_px, goal_py
            else:
                lookahead_px, lookahead_py = get_lookahead_target(robot_px, robot_py, heading)
            lookahead_wx, lookahead_wy = pix_to_world(lookahead_px, lookahead_py)
            lv, rv = steer_to(robot_x, robot_y, heading, lookahead_wx, lookahead_wy)
            left_motor.setVelocity(lv)
            right_motor.setVelocity(rv)
            if map_changed or step_count % DRAW_INTERVAL == 0:
                draw_map(robot_px, robot_py, goal_px, goal_py,
                         lookahead_px, lookahead_py)
        else:
            # Yielding to a higher-priority scout, or no packet from my
            # beacon yet → hold position
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
        continue

    # ── IDLE: sit still, scan, and bid on unawarded beacons ─────────
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    candidates = [b for b in packets if b not in taken]
    if candidates and sim_time - last_rebid >= REBID_PERIOD:
        last_rebid = sim_time
        flood = known_flood(robot_px, robot_py)   # one flood serves all bids
        for b in candidates:
            s, brg = packets[b]
            gx, gy = project_goal(robot_x, robot_y, heading, s, brg)
            cost_bid = bid_for_goal(flood, gx, gy)
            if cost_bid < INF:
                auction_tx.send(("BID %d %d %.3f"
                                 % (b, ROBOT_ID, cost_bid)).encode("utf-8"))

    if map_changed or step_count % DRAW_INTERVAL == 0:
        draw_map(robot_px, robot_py, goal_px, goal_py)
