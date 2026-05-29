"""
scout — multi-victim distress-ping rescue robot (iteration 2).

What changed from iteration 1 (the rationale, point by point):

  (2) NO POSITIONAL CHEATING.
      In iteration 1 the beacon broadcast "SOS <x> <y>" and the scout drove to
      those exact world coordinates. That is cheating: a real radio gives you no
      map coordinates. Here the beacons send only their identity ("SOS <name>").
      The scout localises a victim purely from what a radio can actually sense:
        • signal strength  s ≈ 1/r²   ->  range  r = 1/sqrt(s)
        • getEmitterDirection()       ->  bearing in the robot's own frame
      Homing is "turn toward the bearing, drive in"; arrival is "range < tol".
      (The robot still uses its OWN gps/compass for mapping & trajectory — those
       are onboard sensors, not knowledge of the victim's location.)

  (3) SLAM WITH CONFIDENCE LEVELS.
      The occupancy grid is now a Bayesian log-odds map. Every cell holds a
      log-odds value; free space pushes it down, a LiDAR hit pushes it up, and
      the magnitude IS the confidence (|p - 0.5|). Unobserved cells stay at 0
      (p = 0.5, "unknown"). A cell only counts as a wall once its confidence
      crosses a threshold, so a single spurious beam can't paint a wall.

  (4) MORE PINGERS — the world now has several beacons, all on channel 1.

  (5) CLOSEST SIGNAL FIRST — among beacons not yet attended, the scout picks the
      one with the STRONGEST signal (= nearest) as its current target.

  (1-assess) GRAVITY — on reaching a victim, severity is assessed. For now this
      is a random green/yellow/red (placeholder for a real triage sensor).

  (2-map) PRINTED SLAM MAP — when every heard beacon has been attended, the
      scout prints the SLAM map to the console: walls / free / unknown, the full
      robot trajectory, and each victim drawn in its urgency colour.
"""
from controller import Robot
import math
import random

# ───────────────────────────── Tuning knobs ─────────────────────────────
TIME_STEP      = 32
CRUISE_SPEED   = 3.0
MAX_SPEED      = 6.28

# Map / grid parameters
MAP_SIZE       = 200
MAP_CENTRE     = MAP_SIZE // 2
WORLD_X_MAX    = 4.0
WORLD_Y_MAX    = 3.0

# Bayesian log-odds occupancy (SLAM with confidence)
L_OCC          =  0.85      # log-odds added to a cell that a beam ends on
L_FREE         = -0.40      # log-odds added to a cell a beam passes through
L_CLAMP        =  6.0       # clamp magnitude so cells stay correctable
WALL_LOGODDS   =  0.85      # > this  -> confidently a wall (p ≈ 0.70)
FREE_LOGODDS   = -0.40      # < this  -> confidently free

# Homing / arrival (signal-only)
GOAL_TOL       = 0.22       # arrived when estimated range to victim < this (m)
CLOSE_APPROACH = 0.55       # inside this range, ignore obstacle-avoidance so the
                            # scout can actually reach the victim it can "see"

# Reactive driving
AVOID_DISTANCE   = 0.22
TURN_SPEED       = 4.0
SPIRAL_OMEGA0    = 3.0
SPIRAL_OMEGA_MIN = 0.4
SPIRAL_DECAY     = 0.004

DRAW_INTERVAL  = 16

INF = float("inf")

GRAVITIES = ["green", "yellow", "red"]   # triage severity (placeholder = random)

# ═══════════════════════════ Webots device setup ═══════════════════════════
robot = Robot()

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
LIDAR_FOV = lidar.getFov()

camera = robot.getDevice("camera")         # enabled for extra credit / future use
camera.enable(TIME_STEP)

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

gps = robot.getDevice("gps")               # onboard localisation (not cheating)
gps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

display = robot.getDevice("map_display")

# ═══════════════════════════ Coordinate helpers ═══════════════════════════
def world_to_pix(wx, wy):
    px = int(MAP_CENTRE + (wx / WORLD_X_MAX) * MAP_CENTRE)
    py = int(MAP_CENTRE - (wy / WORLD_Y_MAX) * MAP_CENTRE)
    return max(0, min(MAP_SIZE - 1, px)), max(0, min(MAP_SIZE - 1, py))


def beam_angle(i, n):
    """Robot-relative angle of LiDAR beam i (rad, CCW+, 0 = straight ahead)."""
    return (LIDAR_FOV / 2.0) - (LIDAR_FOV * i / n)


# ═══════════════════ SLAM: Bayesian log-odds occupancy grid ═══════════════
# logodds[y][x] : 0 = unknown, >0 = likely wall, <0 = likely free.
# The magnitude is our confidence.
logodds = [[0.0] * MAP_SIZE for _ in range(MAP_SIZE)]
seen    = [[False] * MAP_SIZE for _ in range(MAP_SIZE)]


def prob(x, y):
    """Occupancy probability 0..1 from the log-odds value."""
    return 1.0 / (1.0 + math.exp(-logodds[y][x]))


def confidence(x, y):
    """How sure we are about a cell, 0 (unknown) .. 1 (certain)."""
    return abs(prob(x, y) - 0.5) * 2.0


def is_wall(x, y):
    return logodds[y][x] > WALL_LOGODDS


def bump(x, y, delta):
    if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
        seen[y][x] = True
        v = logodds[y][x] + delta
        logodds[y][x] = max(-L_CLAMP, min(L_CLAMP, v))


# ───── Bresenham ray trace through the grid ─────
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


def scan_lidar(robot_x, robot_y, heading, ranges):
    """Fold one LiDAR scan into the log-odds map. Returns True if any cell
    crossed the wall-confidence threshold (so the display is worth refreshing)."""
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

        prev_wall = is_wall(epx, epy)
        for cx, cy in bresenham(rpx, rpy, epx, epy):
            if not (0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE):
                continue
            if cx == epx and cy == epy and is_hit:
                bump(cx, cy, L_OCC)        # endpoint -> evidence of a wall
            else:
                bump(cx, cy, L_FREE)       # passed through -> evidence of free
        if is_hit and is_wall(epx, epy) != prev_wall:
            state_changed = True

    return state_changed


# ═══════════════════════════ Steering primitives ═══════════════════════════
def steer_to_bearing(bearing):
    """Differential-drive command that turns toward a robot-relative bearing
    (rad, 0 = ahead, +CCW) while driving forward. Bearing comes straight from
    the radio's getEmitterDirection() — no world coordinates involved."""
    error = bearing
    while error >  math.pi: error -= 2.0 * math.pi
    while error < -math.pi: error += 2.0 * math.pi

    K_TURN  = 4.0
    omega   = K_TURN * error
    forward = CRUISE_SPEED * max(0.0, 1.0 - 2.0 * abs(error) / math.pi)
    lv = max(-MAX_SPEED, min(MAX_SPEED, forward - omega))
    rv = max(-MAX_SPEED, min(MAX_SPEED, forward + omega))
    return lv, rv


def spiral_cmd(t):
    """Expanding search spiral, used while no beacon is being homed."""
    omega = max(SPIRAL_OMEGA_MIN, SPIRAL_OMEGA0 - SPIRAL_DECAY * t)
    return CRUISE_SPEED - omega, CRUISE_SPEED + omega


def lidar_sectors(ranges):
    n = len(ranges)
    fv, lv, rv = [], [], []
    for i, r in enumerate(ranges):
        if r <= 0.0 or math.isinf(r) or math.isnan(r):
            continue
        deg = math.degrees(beam_angle(i, n))
        while deg > 180.0:   deg -= 360.0
        while deg <= -180.0: deg += 360.0
        if   -30.0 <= deg <= 30.0:    fv.append(r)
        elif  60.0 <= deg <= 120.0:   lv.append(r)
        elif -120.0 <= deg <= -60.0:  rv.append(r)
    front = min(fv) if fv else INF
    left  = min(lv) if lv else INF
    right = min(rv) if rv else INF
    return front, left, right


def avoid_or(lv, rv, ranges):
    if not ranges:
        return lv, rv
    front, left, right = lidar_sectors(ranges)
    if front < AVOID_DISTANCE:
        if left > right:
            return -TURN_SPEED, TURN_SPEED
        return TURN_SPEED, -TURN_SPEED
    return lv, rv


# ═══════════════════════════ Display (graphical map) ═══════════════════════
URGENCY_HEX = {"green": 0x00DD00, "yellow": 0xFFD000, "red": 0xFF2020}


def draw_map(robot_px, robot_py, trajectory, victims, target_bearing=None):
    display.setColor(0xDDDDDD)
    display.fillRectangle(0, 0, MAP_SIZE, MAP_SIZE)

    # Occupancy: walls shaded by confidence, free space green-ish, unknown grey.
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            if not seen[y][x]:
                continue
            p = prob(x, y)
            if p > 0.5:                                   # wall side
                t = min(1.0, (logodds[y][x]) / L_CLAMP)   # confidence 0..1
                shade = int(160 * (1.0 - t))              # more sure -> darker
                display.setColor((shade << 16) | (shade << 8) | shade)
            else:                                         # free side
                c = confidence(x, y)
                g = int(120 + 110 * c)
                display.setColor((30 << 16) | (g << 8) | 30)
            display.drawPixel(x, y)

    # Full trajectory (polyline).
    display.setColor(0x3070FF)
    for k in range(1, len(trajectory)):
        x0, y0 = world_to_pix(*trajectory[k - 1])
        x1, y1 = world_to_pix(*trajectory[k])
        display.drawLine(x0, y0, x1, y1)

    # Victims, coloured by assessed urgency.
    for v in victims:
        vpx, vpy = world_to_pix(v["x"], v["y"])
        display.setColor(URGENCY_HEX[v["gravity"]])
        display.fillOval(vpx, vpy, 4, 4)

    # Robot.
    display.setColor(0x0000FF)
    display.fillOval(robot_px, robot_py, 3, 3)


# ═══════════════════════════ Printed SLAM map (console) ════════════════════
ANSI = {"green": "\033[92m", "yellow": "\033[93m", "red": "\033[91m"}
ANSI_TRAJ = "\033[94m"
ANSI_RST  = "\033[0m"
PRINT_COLS = 72
PRINT_ROWS = 34


def print_slam_map(trajectory, victims):
    # Base layer from the occupancy grid.
    grid = [[" "] * PRINT_COLS for _ in range(PRINT_ROWS)]
    for r in range(PRINT_ROWS):
        for c in range(PRINT_COLS):
            wx = -WORLD_X_MAX + (c + 0.5) / PRINT_COLS * 2 * WORLD_X_MAX
            wy =  WORLD_Y_MAX - (r + 0.5) / PRINT_ROWS * 2 * WORLD_Y_MAX
            px, py = world_to_pix(wx, wy)
            if not seen[py][px]:
                grid[r][c] = " "
            elif logodds[py][px] > WALL_LOGODDS:
                grid[r][c] = "#"
            elif logodds[py][px] < FREE_LOGODDS:
                grid[r][c] = "."
            else:
                grid[r][c] = ":"          # seen but low confidence

    def to_cell(wx, wy):
        c = int((wx + WORLD_X_MAX) / (2 * WORLD_X_MAX) * (PRINT_COLS - 1))
        r = int((WORLD_Y_MAX - wy) / (2 * WORLD_Y_MAX) * (PRINT_ROWS - 1))
        return max(0, min(PRINT_COLS - 1, c)), max(0, min(PRINT_ROWS - 1, r))

    # Trajectory overlay (don't bury walls).
    traj_cells = set()
    for wx, wy in trajectory:
        c, r = to_cell(wx, wy)
        if grid[r][c] in (" ", ".", ":"):
            grid[r][c] = "+"
            traj_cells.add((r, c))

    # Victim overlay (top priority).
    victim_cells = {}
    for v in victims:
        c, r = to_cell(v["x"], v["y"])
        grid[r][c] = v["gravity"][0].upper()      # G / Y / R
        victim_cells[(r, c)] = v["gravity"]

    # Render with ANSI colour.
    print("\n" + "=" * (PRINT_COLS + 2))
    print(" SLAM MAP  —  # wall   . free   : low-confidence   + trajectory")
    print("=" * (PRINT_COLS + 2))
    for r in range(PRINT_ROWS):
        line = ["|"]
        for c in range(PRINT_COLS):
            ch = grid[r][c]
            if (r, c) in victim_cells:
                line.append(ANSI[victim_cells[(r, c)]] + ch + ANSI_RST)
            elif (r, c) in traj_cells:
                line.append(ANSI_TRAJ + ch + ANSI_RST)
            else:
                line.append(ch)
        line.append("|")
        print("".join(line))
    print("=" * (PRINT_COLS + 2))

    # Legend / triage summary.
    counts = {"red": 0, "yellow": 0, "green": 0}
    for v in victims:
        counts[v["gravity"]] += 1
    print(" Victims attended (closest-first): %d" % len(victims))
    for i, v in enumerate(victims, 1):
        col = ANSI[v["gravity"]]
        print("   %d. %-9s  urgency=%s%-6s%s  at ≈(%.2f, %.2f)"
              % (i, v["id"], col, v["gravity"], ANSI_RST, v["x"], v["y"]))
    print(" Triage totals:  %sRED %d%s   %sYELLOW %d%s   %sGREEN %d%s"
          % (ANSI["red"], counts["red"], ANSI_RST,
             ANSI["yellow"], counts["yellow"], ANSI_RST,
             ANSI["green"], counts["green"], ANSI_RST))
    print("=" * (PRINT_COLS + 2) + "\n")


# ═══════════════════════════ Radio sensing ═══════════════════════════
def parse_id(msg):
    parts = msg.split()
    return parts[1] if len(parts) >= 2 else None


def read_pings():
    """Drain the receiver queue. Returns {id: (strength, bearing_rad)} for every
    beacon heard this step. Bearing is robot-relative (from getEmitterDirection)."""
    out = {}
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        if "SOS" in msg:
            pid = parse_id(msg)
            if pid is not None:
                s = receiver.getSignalStrength()       # ≈ 1 / r²
                d = receiver.getEmitterDirection()     # unit vec, robot frame
                bearing = math.atan2(d[1], d[0])
                out[pid] = (s, bearing)
        receiver.nextPacket()
    return out


def strength_to_range(s):
    return 1.0 / math.sqrt(s) if s > 0 else INF


# ═══════════════════════════ Main loop ═══════════════════════════
prev_cell   = (-1, -1)
step_count  = 0
trajectory  = []                 # full path, list of (wx, wy)
victims      = []                # attended victims: {id, x, y, gravity}
attended     = set()             # ids already reached
last_seen    = {}                # id -> (strength, bearing) most recent reading
heard_ids    = set()             # every id ever heard
current_id   = None              # beacon currently being homed
mission_done = False

print("[scout] online — searching for distress beacons (signal-only homing)")

while robot.step(TIME_STEP) != -1:
    step_count += 1
    map_changed = False

    vals = gps.getValues()
    robot_x, robot_y = vals[0], vals[1]
    cvals = compass.getValues()
    heading = math.atan2(cvals[0], cvals[1])

    robot_px, robot_py = world_to_pix(robot_x, robot_y)
    current_cell = (robot_px, robot_py)
    ranges = lidar.getRangeImage()

    # SLAM: fold a scan in on every new cell, and log the trajectory.
    if current_cell != prev_cell:
        prev_cell = current_cell
        trajectory.append((robot_x, robot_y))
        if ranges and scan_lidar(robot_x, robot_y, heading, ranges):
            map_changed = True

    # Sense the radios; remember the latest reading for each beacon.
    pings = read_pings()
    for pid, sb in pings.items():
        last_seen[pid] = sb
        heard_ids.add(pid)

    # ── Mission complete? Every beacon we've ever heard has been attended. ──
    if heard_ids and heard_ids <= attended:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if not mission_done:
            mission_done = True
            print("[scout] all %d victims attended — mission complete."
                  % len(victims))
            print_slam_map(trajectory, victims)
            draw_map(robot_px, robot_py, trajectory, victims)
        continue

    # ── Pick the CLOSEST un-attended beacon (strongest signal). ──
    candidates = [(s, pid) for pid, (s, _) in last_seen.items()
                  if pid not in attended]
    if candidates:
        candidates.sort(reverse=True)            # strongest signal first
        best_strength, best_id = candidates[0]
        if best_id != current_id:
            current_id = best_id
            print("[scout] homing on %s (strongest signal, est. range %.2f m)"
                  % (best_id, strength_to_range(best_strength)))
    else:
        current_id = None

    # ── Drive. ──
    if current_id is None:
        # No un-attended beacon located yet -> keep exploring.
        lv, rv = spiral_cmd(step_count)
        lv, rv = avoid_or(lv, rv, ranges)
    else:
        strength, bearing = last_seen[current_id]
        est_range = strength_to_range(strength)

        if est_range < GOAL_TOL:
            # Reached the victim: assess gravity, log it, move on.
            gravity = random.choice(GRAVITIES)
            attended.add(current_id)
            victims.append({"id": current_id, "x": robot_x, "y": robot_y,
                            "gravity": gravity})
            print("[scout] reached %s — triage = %s  (at ≈ %.2f, %.2f)"
                  % (current_id, gravity.upper(), robot_x, robot_y))
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            current_id = None
            draw_map(robot_px, robot_py, trajectory, victims)
            continue

        lv, rv = steer_to_bearing(bearing)
        if est_range > CLOSE_APPROACH:           # only dodge while still far out
            lv, rv = avoid_or(lv, rv, ranges)

    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)

    if map_changed or step_count % DRAW_INTERVAL == 0:
        draw_map(robot_px, robot_py, trajectory, victims)
