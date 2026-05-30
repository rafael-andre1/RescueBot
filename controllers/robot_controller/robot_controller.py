"""
scout — distress-ping rescue robot.

Behaviour:
  1. Spiral search — wander outward in an expanding spiral, reactively
     avoiding obstacles, until a distress ping is received.
  2. On ping — SOS carries no coordinates; engage gradient navigation toward
     the hardcoded goal using the live cost matrix.
  3. Home in — follow the Dijkstra gradient (10-cell lookahead), deflecting
     around obstacles via LiDAR.

A probabilistic occupancy grid (hits / visits → wall or free) is built the
whole time. When a cell changes state the cost matrix is replanned so the
gradient always reflects the current known map.

Time limit: SIM_TIME_LIMIT seconds. On arrival or expiry the occupancy grid
is saved as slam_map.png.
"""
from controller import Robot
import math
import heapq

#  Tuning knobs
TIME_STEP        = 32
CRUISE_SPEED     = 3.0
MAX_SPEED        = 6.28
SAFE_DISTANCE    = 0.15
SIM_TIME_LIMIT   = 105.0        # 1 min 45 s

#  Map / grid parameters 
MAP_SIZE        = 200
MAP_CENTRE      = MAP_SIZE // 2
WORLD_X_MAX     = 4.0
WORLD_Y_MAX     = 3.0
OBSTACLE_INFLATE = 2        # pixels of inflation around walls

#  Occupancy thresholds 
WALL_CERTAINTY  = 0.30      # occupancy above this counts as a wall
IMPASSABLE      = 0.90      # above this → cost = 1000
IMPASSABLE_COST = 1000

#  Goal 
PINGER_X        =  3.0
PINGER_Y        = -2.0
GOAL_TOL        = 0.10

#  Display 
DRAW_INTERVAL   = 16        # refresh the moving robot dot every N steps
                            # (the grid itself is redrawn immediately on a replan)

#  Neighbor definitions 
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
LIDAR_FOV = lidar.getFov()   # Webots scanline: index 0 at +FOV/2, angle decreases (CW) with index


# Setup (extra credit)
camera   = robot.getDevice("camera")
camera.enable(TIME_STEP)


# Setting up the ping receiver
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


def beam_angle(i, n):
    """Robot-relative angle of LiDAR beam i (rad, CCW+, 0 = straight ahead).
    Webots range image: index 0 is at +FOV/2, angle decreases as index grows."""
    return (LIDAR_FOV / 2.0) - (LIDAR_FOV * i / n)


# ═══════════════════════════════════════════════════════════════════
#  MATRIX 1 — occupancy grid (hits / visits → probability)
# ═══════════════════════════════════════════════════════════════════
hits   = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]
visits = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]


def get_occupancy(x, y):
    """Occupancy probability 0.0–1.0 for cell (x, y)."""
    v = visits[y][x]
    if v == 0:
        return 0.0
    return hits[y][x] / v


def is_wall(x, y):
    """Discrete cell state: True if this cell is a wall."""
    return get_occupancy(x, y) > WALL_CERTAINTY


def get_cell_cost(x, y):
    """Occupancy → traversal cost: 1–9 for 0–90%, 1000 for >90%."""
    occ = get_occupancy(x, y)
    if occ > IMPASSABLE:
        return IMPASSABLE_COST
    return 1 + int(occ / IMPASSABLE * 8)


# ═══════════════════════════════════════════════════════════════════
#  Bresenham line — trace a ray through the grid
# ═══════════════════════════════════════════════════════════════════
def bresenham(x0, y0, x1, y1):
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

        angle = heading + beam_angle(i, n)
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

        # Did the hit cell flip free → wall (or wall → free)?
        if is_hit and is_wall(epx, epy) != prev_wall:
            state_changed = True

    return state_changed


# ═══════════════════════════════════════════════════════════════════
#  Inflation — rebuilt whenever the cost matrix is recomputed
# ═══════════════════════════════════════════════════════════════════
inflated = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]


def inflate_obstacles():
    """Mark the inflated zone around every wall cell."""
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
#  MATRIX 2 — cost matrix: Dijkstra flood-fill from the goal
# ═══════════════════════════════════════════════════════════════════
cost = [[INF] * MAP_SIZE for _ in range(MAP_SIZE)]


def compute_cost_matrix():
    """Flood-fill from goal. Edge weight = move_dist * cell_cost."""
    gx, gy = world_to_pix(PINGER_X, PINGER_Y)

    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            cost[y][x] = INF

    # If goal sits on an obstacle, snap to the nearest free cell
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
            nc = c + move_dist * cc
            if nc < cost[ny][nx]:
                cost[ny][nx] = nc
                heapq.heappush(heap, (nc, nx, ny))


def replan():
    """Rebuild inflation + cost matrix. Called only on a cell-state change."""
    inflate_obstacles()
    compute_cost_matrix()


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
                    best_nx, best_ny = nx, ny
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
#  LiDAR sectors
# ═══════════════════════════════════════════════════════════════════
def lidar_sectors(ranges):
    """Return (front, left, right) min distances. Robot-relative angles."""
    n = len(ranges)
    front_vals, left_vals, right_vals = [], [], []
    for i, r in enumerate(ranges):
        if r <= 0.0 or math.isinf(r) or math.isnan(r):
            continue
        deg = math.degrees(beam_angle(i, n))
        while deg > 180.0:  deg -= 360.0
        while deg <= -180.0: deg += 360.0
        if -30.0 <= deg <= 30.0:
            front_vals.append(r)
        elif 60.0 <= deg <= 120.0:
            left_vals.append(r)
        elif -120.0 <= deg <= -60.0:
            right_vals.append(r)
    front = min(front_vals) if front_vals else INF
    left  = min(left_vals)  if left_vals  else INF
    right = min(right_vals) if right_vals else INF
    return front, left, right


# ═══════════════════════════════════════════════════════════════════
#  Display
# ═══════════════════════════════════════════════════════════════════
# ═══════════════════════════════════════════════════════════════════
#  Image save — writes slam_map.png next to this script
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
        gx, gy = world_to_pix(PINGER_X, PINGER_Y)
        for dy in range(-3, 4):
            for dx in range(-3, 4):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < MAP_SIZE and 0 <= ny < MAP_SIZE:
                    pix[nx, ny] = (255, 0, 0)
        img.save(filename)
        print("[scout] map saved → " + filename)
    except ImportError:
        print("[scout] install Pillow to enable image save:  pip install Pillow")


# ═══════════════════════════════════════════════════════════════════
#  Display
# ═══════════════════════════════════════════════════════════════════
def draw_map(robot_px, robot_py, goal_px, goal_py,
             target_px=None, target_py=None):
    display.setColor(0xDDDDDD)
    display.fillRectangle(0, 0, MAP_SIZE, MAP_SIZE)

    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            occ = get_occupancy(x, y)
            if occ > IMPASSABLE:
                display.setColor(0x000000)                 # solid wall
            elif occ > WALL_CERTAINTY:
                t = (occ - WALL_CERTAINTY) / (IMPASSABLE - WALL_CERTAINTY)
                display.setColor((int(180 + 75 * t) << 16) | (int(120 * (1.0 - t)) << 8))
            elif inflated[y][x] == 1:
                display.setColor(0x444444)                 # inflated buffer
            elif visits[y][x] > 0:
                g = int(180 + 60 * (1.0 - occ / WALL_CERTAINTY))
                display.setColor((40 << 16) | (g << 8) | 40)
            else:
                continue                                   # unseen → grey bg
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
#  Initial cost matrix — no walls yet, pure gradient
# ═══════════════════════════════════════════════════════════════════
compute_cost_matrix()

# ═══════════════════════════════════════════════════════════════════
#  Main loop
# ═══════════════════════════════════════════════════════════════════
prev_cell     = (-1, -1)
step_count    = 0
ping_received = False        # spiral-search until a distress ping arrives
goal_px, goal_py = world_to_pix(PINGER_X, PINGER_Y)

# Reactive-driving knobs
AVOID_DISTANCE   = 0.22      # deflect when an obstacle is this close ahead (m)
TURN_SPEED       = 4.0       # wheel speed while pivoting away from an obstacle
SPIRAL_OMEGA0    = 3.0       # initial turn rate of the search spiral
SPIRAL_OMEGA_MIN = 0.4       # the spiral never opens wider than this
SPIRAL_DECAY     = 0.004     # how fast the spiral opens up, per step


def spiral_cmd(t):
    """Expanding Archimedean spiral: cruise forward while the turn rate
    decays over time, so the search radius grows."""
    omega = max(SPIRAL_OMEGA_MIN, SPIRAL_OMEGA0 - SPIRAL_DECAY * t)
    return CRUISE_SPEED - omega, CRUISE_SPEED + omega


def avoid_or(lv, rv, ranges):
    """Reactive obstacle avoidance: if the path ahead is blocked, pivot
    toward the more open side; otherwise pass the wheel speeds through."""
    if not ranges:
        return lv, rv
    front, left, right = lidar_sectors(ranges)
    if front < AVOID_DISTANCE:
        if left > right:
            return -TURN_SPEED, TURN_SPEED      # more room left -> turn left
        return TURN_SPEED, -TURN_SPEED          # turn right
    return lv, rv


while robot.step(TIME_STEP) != -1:
    step_count += 1
    sim_time    = step_count * TIME_STEP / 1000.0
    map_changed = False

    vals         = gps.getValues()
    robot_x, robot_y = vals[0], vals[1]
    compass_vals = compass.getValues()
    heading      = math.atan2(compass_vals[0], compass_vals[1])

    robot_px, robot_py = world_to_pix(robot_x, robot_y)
    current_cell = (robot_px, robot_py)
    ranges       = lidar.getRangeImage()

    # SLAM: update map and replan whenever a cell changes state.
    if current_cell != prev_cell:
        prev_cell = current_cell
        if ranges and scan_lidar(robot_x, robot_y, heading, ranges):
            replan()            # inflate + recompute cost matrix
            map_changed = True

    # Time limit.
    if sim_time >= SIM_TIME_LIMIT:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("[scout] time limit reached (%.1f s)" % sim_time)
        draw_map(robot_px, robot_py, goal_px, goal_py)
        save_map_image()
        break

    # Listen for SOS — flag only, no coordinates read.
    if not ping_received:
        while receiver.getQueueLength() > 0:
            msg = receiver.getString()
            if "SOS" in msg:
                ping_received = True
                print("[scout] SOS received — switching to gradient navigation")
            receiver.nextPacket()

    # Phase 1: no ping yet -> spiral outward, mapping as we go.
    if not ping_received:
        lv, rv = spiral_cmd(step_count)
        lv, rv = avoid_or(lv, rv, ranges)
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)
        if map_changed or step_count % DRAW_INTERVAL == 0:
            draw_map(robot_px, robot_py, goal_px, goal_py)
        continue

    # Phase 2: gradient descent to goal.
    if math.hypot(PINGER_X - robot_x, PINGER_Y - robot_y) < GOAL_TOL:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("[scout] goal reached in %.1f s" % sim_time)
        draw_map(robot_px, robot_py, goal_px, goal_py)
        save_map_image()
        break

    lookahead_px, lookahead_py = follow_gradient(robot_px, robot_py)
    lookahead_wx, lookahead_wy = pix_to_world(lookahead_px, lookahead_py)
    lv, rv = steer_to(robot_x, robot_y, heading, lookahead_wx, lookahead_wy)
    lv, rv = avoid_or(lv, rv, ranges)
    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)

    if map_changed or step_count % DRAW_INTERVAL == 0:
        draw_map(robot_px, robot_py, goal_px, goal_py, lookahead_px, lookahead_py)
