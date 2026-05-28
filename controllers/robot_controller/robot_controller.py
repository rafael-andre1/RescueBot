"""
scout — Wavefront gradient planner with probabilistic occupancy grid.

1. Occupancy grid: each cell tracks hits/visits → occupancy %.
2. Cell cost: 1–9 for 0–90% occupancy, 1000 for >90%.
3. Inflation: only from cells with >30% certainty.
4. Cost matrix: Dijkstra flood-fill from goal using cell costs.
5. Robot follows the gradient (direction vector to target cell).
"""
from controller import Robot
import math
import heapq

# ─── Tuning knobs ──────────────────────────────────────────────────
TIME_STEP       = 32
CRUISE_SPEED    = 3.0
TURN_SPEED      = 2.0
MAX_SPEED       = 6.28
SAFE_DISTANCE   = 0.15
FRONT_ARC_DEG   = 60

# ─── Map / grid parameters ─────────────────────────────────────────
MAP_SIZE        = 200
MAP_CENTRE      = MAP_SIZE // 2
WORLD_X_MAX     = 4.0
WORLD_Y_MAX     = 3.0
OBSTACLE_INFLATE = 2        # pixels of inflation around certain walls

# ─── Occupancy thresholds ──────────────────────────────────────────
WALL_CERTAINTY  = 0.30      # only inflate cells above this occupancy
IMPASSABLE      = 0.90      # above this → cost = 1000
IMPASSABLE_COST = 1000

# ─── Goal ──────────────────────────────────────────────────────────
PINGER_X        =  3.0
PINGER_Y        = -2.0
GOAL_TOL        = 0.10

# ─── Neighbor definitions ─────────────────────────────────────────
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

lidar    = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

camera   = robot.getDevice("camera")
camera.enable(TIME_STEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

gps      = robot.getDevice("gps")
gps.enable(TIME_STEP)

compass  = robot.getDevice("compass")
compass.enable(TIME_STEP)

display  = robot.getDevice("map_display")


# ═══════════════════════════════════════════════════════════════════
#  Coordinate helpers
# ═══════════════════════════════════════════════════════════════════
def world_to_pix(wx, wy):
    px = int(MAP_CENTRE + (wx / WORLD_X_MAX) * MAP_CENTRE)
    py = int(MAP_CENTRE - (wy / WORLD_Y_MAX) * MAP_CENTRE)
    return max(0, min(MAP_SIZE - 1, px)), max(0, min(MAP_SIZE - 1, py))


def pix_to_world(px, py):
    wx = (px - MAP_CENTRE) / MAP_CENTRE * WORLD_X_MAX
    wy = (MAP_CENTRE - py) / MAP_CENTRE * WORLD_Y_MAX
    return wx, wy


# ═══════════════════════════════════════════════════════════════════
#  Occupancy grid — hits / visits → probability
# ═══════════════════════════════════════════════════════════════════
hits   = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]
visits = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]


def get_occupancy(x, y):
    """Return occupancy probability 0.0–1.0 for cell (x, y)."""
    v = visits[y][x]
    if v == 0:
        return 0.0
    return hits[y][x] / v


def get_cell_cost(x, y):
    """Occupancy → traversal cost: 1–9 for 0–90%, 1000 for >90%."""
    occ = get_occupancy(x, y)
    if occ > IMPASSABLE:
        return IMPASSABLE_COST
    # Linear: 0% → 1, 90% → 9
    return 1 + int(occ / IMPASSABLE * 8)


# ═══════════════════════════════════════════════════════════════════
#  Bresenham line — trace a ray through the grid
# ═══════════════════════════════════════════════════════════════════
def bresenham(x0, y0, x1, y1):
    """Yield all (x, y) cells along the line from (x0,y0) to (x1,y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


# ═══════════════════════════════════════════════════════════════════
#  LiDAR scan → update occupancy grid
# ═══════════════════════════════════════════════════════════════════
def scan_lidar(robot_x, robot_y, heading, ranges):
    """Ray-trace each LiDAR beam. Free cells get a visit, hit cells get a hit+visit.
    Returns True if any cell changed significantly."""
    changed = False
    n = len(ranges)
    max_range = lidar.getMaxRange() - 0.05
    rpx, rpy = world_to_pix(robot_x, robot_y)

    for i, r in enumerate(ranges):
        if r <= 0.0 or math.isinf(r) or math.isnan(r):
            continue

        angle = heading + (2.0 * math.pi * i / n)
        is_hit = r <= max_range

        # End-point of the ray (clip to max range for free rays)
        ray_len = r if is_hit else max_range
        hx = robot_x + ray_len * math.cos(angle)
        hy = robot_y + ray_len * math.sin(angle)
        epx, epy = world_to_pix(hx, hy)

        # Trace the ray
        prev_occ = get_occupancy(epx, epy) if is_hit else -1.0
        for cx, cy in bresenham(rpx, rpy, epx, epy):
            if not (0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE):
                continue
            if cx == epx and cy == epy and is_hit:
                # Hit cell — mark as occupied
                hits[cy][cx] += 1
                visits[cy][cx] += 1
            else:
                # Free cell — ray passed through
                visits[cy][cx] += 1

        # Check if the hit cell changed occupancy bracket significantly
        if is_hit:
            new_occ = get_occupancy(epx, epy)
            if (prev_occ <= WALL_CERTAINTY) != (new_occ <= WALL_CERTAINTY):
                changed = True
            elif (prev_occ <= IMPASSABLE) != (new_occ <= IMPASSABLE):
                changed = True

    return changed


# ═══════════════════════════════════════════════════════════════════
#  Inflation — only from cells with >30% certainty
# ═══════════════════════════════════════════════════════════════════
inflated = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]


def inflate_obstacles():
    """Mark inflated zone around cells with occupancy > WALL_CERTAINTY."""
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            inflated[y][x] = 0
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            if get_occupancy(x, y) > WALL_CERTAINTY:
                for dy in range(-OBSTACLE_INFLATE, OBSTACLE_INFLATE + 1):
                    for dx in range(-OBSTACLE_INFLATE, OBSTACLE_INFLATE + 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < MAP_SIZE and 0 <= nx < MAP_SIZE:
                            inflated[ny][nx] = 1


# ═══════════════════════════════════════════════════════════════════
#  Cost matrix — Dijkstra flood-fill from goal using cell costs
# ═══════════════════════════════════════════════════════════════════
cost = [[INF] * MAP_SIZE for _ in range(MAP_SIZE)]


def compute_cost_matrix():
    """Flood-fill from goal. Edge weight = move_dist * cell_cost."""
    gx, gy = world_to_pix(PINGER_X, PINGER_Y)

    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            cost[y][x] = INF

    # If goal on inflated obstacle, find nearest free cell
    if inflated[gy][gx] == 1 or get_cell_cost(gx, gy) >= IMPASSABLE_COST:
        best, best_d = None, INF
        for dy in range(-15, 16):
            for dx in range(-15, 16):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                    if inflated[ny][nx] == 0 and get_cell_cost(nx, ny) < IMPASSABLE_COST:
                        d = abs(dx) + abs(dy)
                        if d < best_d:
                            best_d = d
                            best = (nx, ny)
        if best:
            gx, gy = best
        else:
            return

    cost[gy][gx] = 0.0
    heap = [(0.0, gx, gy)]

    while heap:
        c, cx, cy = heapq.heappop(heap)
        if c > cost[cy][cx]:
            continue
        for dx, dy, move_dist in NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE):
                continue
            if inflated[ny][nx] == 1:
                continue
            cc = get_cell_cost(nx, ny)
            if cc >= IMPASSABLE_COST:
                continue
            # Edge weight = movement distance * cell traversal cost
            nc = c + move_dist * cc
            if nc < cost[ny][nx]:
                cost[ny][nx] = nc
                heapq.heappush(heap, (nc, nx, ny))


# ═══════════════════════════════════════════════════════════════════
#  Gradient follower — trace several cells ahead
# ═══════════════════════════════════════════════════════════════════
LOOK_AHEAD_CELLS = 10


def follow_gradient(rpx, rpy):
    cx, cy = rpx, rpy
    for _ in range(LOOK_AHEAD_CELLS):
        best_c = cost[cy][cx]
        best_nx, best_ny = cx, cy
        for dx, dy, _ in NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                if cost[ny][nx] < best_c:
                    best_c = cost[ny][nx]
                    best_nx = nx
                    best_ny = ny
        if (best_nx, best_ny) == (cx, cy):
            break
        cx, cy = best_nx, best_ny
    return cx, cy


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
#  LiDAR helpers
# ═══════════════════════════════════════════════════════════════════
def lidar_sectors(ranges):
    """Return (front_min, left_min, right_min) distances from the LiDAR.
    Front = ±30°, Left = 60°–120°, Right = 240°–300°."""
    n = len(ranges)
    def sector_min(start_deg, end_deg):
        i0 = int(start_deg / 360.0 * n) % n
        i1 = int(end_deg / 360.0 * n) % n
        if i0 <= i1:
            vals = [r for r in ranges[i0:i1] if r > 0.0]
        else:
            vals = [r for r in (list(ranges[i0:]) + list(ranges[:i1])) if r > 0.0]
        return min(vals) if vals else INF
    front = sector_min(330, 30)
    left  = sector_min(60, 120)
    right = sector_min(240, 300)
    return front, left, right


# ═══════════════════════════════════════════════════════════════════
#  Display
# ═══════════════════════════════════════════════════════════════════
def draw_map(robot_px, robot_py, goal_px, goal_py,
             target_px=None, target_py=None):
    # Background
    display.setColor(0xDDDDDD)
    display.fillRectangle(0, 0, MAP_SIZE, MAP_SIZE)

    # Occupancy grid — colour by occupancy %
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            occ = get_occupancy(x, y)
            v = visits[y][x]
            if v == 0:
                continue  # unseen → leave grey
            if occ > IMPASSABLE:
                display.setColor(0x000000)       # black = wall
            elif occ > WALL_CERTAINTY:
                # Orange–red for uncertain walls (30–90%)
                t = (occ - WALL_CERTAINTY) / (IMPASSABLE - WALL_CERTAINTY)
                r = int(180 + 75 * t)
                g = int(120 * (1.0 - t))
                display.setColor((r << 16) | (g << 8))
            else:
                # Green = free (seen, low occupancy)
                g = int(180 + 60 * (1.0 - occ / WALL_CERTAINTY))
                display.setColor((40 << 16) | (g << 8) | 40)
            display.drawPixel(x, y)

    # Inflated zone — dark grey overlay
    display.setColor(0x444444)
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            if inflated[y][x] == 1 and get_occupancy(x, y) <= WALL_CERTAINTY:
                display.drawPixel(x, y)

    # Direction vector — cyan line
    if target_px is not None and target_py is not None:
        display.setColor(0x00FFFF)
        display.drawLine(robot_px, robot_py, target_px, target_py)
        display.setColor(0xFFFF00)
        display.fillOval(target_px, target_py, 2, 2)

    # Goal — red
    display.setColor(0xFF0000)
    display.fillOval(goal_px, goal_py, 3, 3)

    # Robot — blue
    display.setColor(0x0000FF)
    display.fillOval(robot_px, robot_py, 3, 3)


# ═══════════════════════════════════════════════════════════════════
#  Drain pings
# ═══════════════════════════════════════════════════════════════════
def drain_pings():
    while receiver.getQueueLength() > 0:
        try:
            msg = receiver.getString()
        except Exception:
            msg = receiver.getBytes()
        strength  = receiver.getSignalStrength()
        direction = receiver.getEmitterDirection()
        print(f"[scout] PING '{msg}'  "
              f"strength={strength:.4f}  "
              f"dir=({direction[0]:+.2f}, {direction[1]:+.2f}, {direction[2]:+.2f})")
        receiver.nextPacket()


# ═══════════════════════════════════════════════════════════════════
#  Initial cost matrix — no walls, pure gradient
# ═══════════════════════════════════════════════════════════════════
print("[scout] Computing initial cost matrix ...")
compute_cost_matrix()
print("[scout] Done. Following gradient toward pinger.")

# ═══════════════════════════════════════════════════════════════════
#  Main loop
# ═══════════════════════════════════════════════════════════════════
step_count = 0
prev_cell  = (-1, -1)
target_px, target_py = None, None
target_wx, target_wy = 0.0, 0.0

while robot.step(TIME_STEP) != -1:
    step_count += 1
    drain_pings()

    # ── Pose ──────────────────────────────────────────────────────
    vals         = gps.getValues()
    robot_x      = vals[0]
    robot_y      = vals[1]
    compass_vals = compass.getValues()
    # Webots compass: direction of north in robot's local frame
    # Try all formulas and print to diagnose
    heading      = math.atan2(compass_vals[0], compass_vals[1])

    if step_count <= 3:
        print(f"[DEBUG] compass raw=({compass_vals[0]:.3f}, {compass_vals[1]:.3f}, {compass_vals[2]:.3f})  "
              f"heading={math.degrees(heading):.1f}°  "
              f"GPS=({robot_x:.2f}, {robot_y:.2f})")

    robot_px, robot_py = world_to_pix(robot_x, robot_y)
    goal_px, goal_py   = world_to_pix(PINGER_X, PINGER_Y)
    current_cell = (robot_px, robot_py)

    # ── Only process when entering a new cell ─────────────────────
    if current_cell != prev_cell:
        prev_cell = current_cell

        ranges = lidar.getRangeImage()
        if ranges:
            changed = scan_lidar(robot_x, robot_y, heading, ranges)
            if changed:
                inflate_obstacles()
                compute_cost_matrix()
                print(f"[scout] Occupancy changed → cost matrix updated")

        # Recompute steering target
        tpx, tpy = follow_gradient(robot_px, robot_py)
        target_px, target_py = tpx, tpy
        target_wx, target_wy = pix_to_world(tpx, tpy)

    # ── Reached the goal? ─────────────────────────────────────────
    dist_to_goal = math.hypot(PINGER_X - robot_x, PINGER_Y - robot_y)
    if dist_to_goal < GOAL_TOL:
        print("[scout] ★ Reached the pinger! ★")
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        draw_map(robot_px, robot_py, goal_px, goal_py)
        continue

    # ── Drive ──────────────────────────────────────────────────────
    ranges = lidar.getRangeImage()
    front_d, left_d, right_d = (INF, INF, INF)
    if ranges:
        front_d, left_d, right_d = lidar_sectors(ranges)

    if front_d < SAFE_DISTANCE and ranges:
        # Wall ahead! Force scan + recompute so gradient routes around it
        scan_lidar(robot_x, robot_y, heading, ranges)
        inflate_obstacles()
        compute_cost_matrix()
        # Get new target from updated gradient
        tpx, tpy = follow_gradient(robot_px, robot_py)
        target_px, target_py = tpx, tpy
        target_wx, target_wy = pix_to_world(tpx, tpy)
        print(f"[scout] Wall ahead ({front_d:.2f}m) → forced recompute")

    if target_px is not None:
        lv, rv = steer_to(robot_x, robot_y, heading, target_wx, target_wy)
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)
    else:
        left_motor.setVelocity(CRUISE_SPEED)
        right_motor.setVelocity(CRUISE_SPEED)

    # ── Draw ──────────────────────────────────────────────────────
    draw_map(robot_px, robot_py, goal_px, goal_py, target_px, target_py)

    if step_count % 50 == 0:
        print(f"GPS: {robot_x:.2f}, {robot_y:.2f}  "
              f"Heading: {math.degrees(heading):.1f}°  "
              f"Goal dist: {dist_to_goal:.2f} m  "
              f"Front: {front_d:.2f} L: {left_d:.2f} R: {right_d:.2f}")