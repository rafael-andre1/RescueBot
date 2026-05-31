"""
scout — distress-ping rescue robot (radio-homing variant).

  • No GPS. Pose is dead-reckoned from wheel encoders, with compass for heading.
  • Goal is unknown — each ping gives a robot-relative bearing + signal strength.
    Strength → estimated range (~1/sqrt(s)); bearing + heading → world direction.
    Together they project an estimated goal in the dead-reckoned frame.
  • Path-finding (Dijkstra over the live occupancy grid) routes around walls
    discovered en route. NO reactive obstacle avoidance, NO spiral search —
    the planner handles dead-ends because the cost field reflects what we've
    mapped so far.

Lifecycle: sit still and scan until the first ping; then follow the cost
gradient toward the estimated goal. Replan when (a) a cell flips state, or
(b) the bearing-derived goal moves > GOAL_MOVE_THRESH. Arrival is declared
when signal strength exceeds S_ARRIVE. Map is saved as slam_map.png on
arrival or expiry.
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
S_ARRIVE         = 6.25         # signal strength threshold → declare arrival.
                                # Webots emitter: strength = 1/d². So:
                                #   d = 1/sqrt(s)   →   s = 1/d²
                                #   d = 0.40 m → s = 6.25
                                #   d = 0.30 m → s = 11.1
                                #   d = 0.20 m → s = 25.0
                                # Keep this aligned with signal_controller's
                                # RESCUE_RADIUS so scout + signal trigger at the
                                # same distance.

#  Continuous rescue
MAX_RESCUES        = 10         # stop after this many help requests handled

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

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

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
#  MATRIX 2 — bidirectional Dijkstra path from robot to goal
#  The cost matrix is no longer a full flood-fill; BD explores roughly
#  half the area of unidirectional Dijkstra, then we write the resulting
#  path into the cost matrix (cost decreases along the path toward goal)
#  so the existing follow_gradient walks it.
# ═══════════════════════════════════════════════════════════════════
cost = [[INF] * MAP_SIZE for _ in range(MAP_SIZE)]


def _snap_to_free(px, py, radius):
    """If (px,py) is blocked, return the nearest free cell within radius, else (px,py)."""
    if inflated[py][px] == 0 and get_cell_cost(px, py) < IMPASSABLE_COST:
        return px, py
    best, best_d = None, INF
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx, ny = px + dx, py + dy
            if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                if inflated[ny][nx] == 0 and get_cell_cost(nx, ny) < IMPASSABLE_COST:
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
                if inflated[ny][nx] == 1:
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
                if inflated[ny][nx] == 1:
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
def estimate_goal_from_radio(robot_x, robot_y, heading):
    """Drain the receiver queue, return (gx, gy, strength) for the strongest
    HELP packet, or None if nothing was heard this step."""
    best_s, best_b = -1.0, 0.0
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        if "SOS" in msg or "HELP" in msg:        # accept either keyword
            s = receiver.getSignalStrength()
            d = receiver.getEmitterDirection()
            b = math.atan2(d[1], d[0])           # robot-relative bearing
            if s > best_s:
                best_s, best_b = s, b
        receiver.nextPacket()
    if best_s <= 0:
        return None
    rng = 1.0 / math.sqrt(best_s)                # Webots default ~1/d² emission
    world_b = heading + best_b
    return (robot_x + rng * math.cos(world_b),
            robot_y + rng * math.sin(world_b),
            best_s)


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
def save_map_image(filename="slam_map.png"):
    try:
        from PIL import Image
        img = Image.new("RGB", (MAP_SIZE, MAP_SIZE), (221, 221, 221))
        pix = img.load()
        for y in range(MAP_SIZE):
            for x in range(MAP_SIZE):
                occ = get_occupancy(x, y)
                if occ > IMPASSABLE:
                    pix[x, y] = (0, 0, 0)
                elif occ > WALL_CERTAINTY:
                    t = (occ - WALL_CERTAINTY) / (IMPASSABLE - WALL_CERTAINTY)
                    pix[x, y] = (int(180 + 75 * t), int(120 * (1.0 - t)), 0)
                elif inflated[y][x] == 1:
                    pix[x, y] = (68, 68, 68)
                elif visits[y][x] > 0:
                    g = int(180 + 60 * (1.0 - occ / WALL_CERTAINTY))
                    pix[x, y] = (40, g, 40)
        img.save(filename)
        print("[scout] map saved → " + filename)
    except ImportError:
        print("[scout] install Pillow to enable image save:  pip install Pillow")


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
ping_received = False
goal_x, goal_y = 0.0, 0.0          # placeholder; set on first ping
robot_x, robot_y = 0.0, 0.0        # dead-reckoned pose, map origin = start
prev_left_enc  = None              # initialised on first iteration
prev_right_enc = None
rescues_completed = 0              # how many help requests handled so far

print("[scout] online — sitting still until first HELP ping")

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
        state_changed = False
        if ranges:
            state_changed = scan_lidar(robot_x, robot_y, heading, ranges)
        # Replan if (a) a cell flipped state, or (b) we've drifted off the BD path
        if ping_received and (state_changed or cost[robot_py][robot_px] == INF):
            replan()
            map_changed = True

    # ── Radio: derive an estimated goal in world frame ──────────────
    # Every rescue runs the same loop: wait for a HELP ping → home in on
    # the strongest signal → arrive → reset to wait. (estimate_goal_from_radio
    # already picks the strongest packet, so a future multi-pinger world
    # works without changes here.)
    est = estimate_goal_from_radio(robot_x, robot_y, heading)
    if est is not None:
        gx, gy, strength = est

        # Arrival: same logic for every rescue
        if ping_received and strength > S_ARRIVE:
            rescues_completed += 1
            print("[scout] rescue #%d reached (strength=%.4f, t=%.1f s)"
                  % (rescues_completed, strength, sim_time))
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            if rescues_completed >= MAX_RESCUES:
                draw_map(robot_px, robot_py, goal_px, goal_py)
                save_map_image()
                break
            # Reset to wait-for-ping. Occupancy survives; cost matrix will
            # be regenerated by replan() when the next HELP arrives.
            ping_received = False
            goal_x, goal_y = 0.0, 0.0
            print("[scout] waiting for next HELP request ...")

        # Update real-radio goal on first ping or significant move
        elif (not ping_received
                or math.hypot(gx - goal_x, gy - goal_y) > GOAL_MOVE_THRESH):
            goal_x, goal_y = gx, gy
            if not ping_received:
                ping_received = True
                print("[scout] HELP received — engaging gradient navigation")
            replan()
            map_changed = True
            goal_px, goal_py = world_to_pix(goal_x, goal_y)

    # ── Phase 1: no ping yet → sit still and scan ───────────────────
    if not ping_received:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if map_changed or step_count % DRAW_INTERVAL == 0:
            draw_map(robot_px, robot_py, goal_px, goal_py)
        continue

    # ── Phase 2: pure gradient descent (reactive override disabled) ─
    lookahead_px, lookahead_py = follow_gradient(robot_px, robot_py, heading)
    lookahead_wx, lookahead_wy = pix_to_world(lookahead_px, lookahead_py)
    lv, rv = steer_to(robot_x, robot_y, heading, lookahead_wx, lookahead_wy)
    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)

    if map_changed or step_count % DRAW_INTERVAL == 0:
        draw_map(robot_px, robot_py, goal_px, goal_py, lookahead_px, lookahead_py)
