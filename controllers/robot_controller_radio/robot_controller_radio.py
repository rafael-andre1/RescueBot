from controller import Robot
import math

# ── Tuning ────────────────────────────────────────────────────────────────
TIME_STEP        = 32
CRUISE_SPEED     = 3.0
MAX_SPEED        = 6.28
SIM_TIME_LIMIT   = 105.0            # 1 min 45 s

# ── Radio homing ──────────────────────────────────────────────────────────
RADIO_STOP_DIST  = 0.35             # stop when estimated range < this (m)
CAMERA_DIST      = 0.80             # activate camera scan within this range

# ── Camera red-target detection (255, 0, 0) ───────────────────────────────
RED_R_MIN        = 200
RED_G_MAX        = 60
RED_B_MAX        = 60
RED_PIXEL_MIN    = 20               # qualifying pixels to confirm a person

# ── SLAM map ──────────────────────────────────────────────────────────────
MAP_SIZE         = 400              # doubled resolution vs iteration 1
MAP_CENTRE       = MAP_SIZE // 2
WORLD_X_MAX      = 4.0              # metres from origin to map edge
WORLD_Y_MAX      = 3.0
OBSTACLE_INFLATE = 3                # pixels, scaled up for new resolution

# ── Occupancy thresholds ──────────────────────────────────────────────────
WALL_CERTAINTY   = 0.30
IMPASSABLE       = 0.90

# ── Timing ────────────────────────────────────────────────────────────────
DRAW_INTERVAL        = 16
SIGNAL_PRINT_STEPS   = int(2.0 * 1000 / TIME_STEP)   # console update every 2 s
CAM_PRINT_STEPS      = int(1.0 * 1000 / TIME_STEP)    # camera update every 1 s

INF = float("inf")

# ═══════════════════════════════════════════════════════════════════
#  Webots devices
# ═══════════════════════════════════════════════════════════════════
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

camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
CAM_W = camera.getWidth()
CAM_H = camera.getHeight()

receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

display = robot.getDevice("map_display")

# Camera preview anchor: top-right corner of SLAM display
CAM_PREV_X = MAP_SIZE - CAM_W // 2 - 2
CAM_PREV_Y = 2

# ═══════════════════════════════════════════════════════════════════
#  Coordinate helpers — robot start position is map origin
# ═══════════════════════════════════════════════════════════════════
origin_x = 0.0
origin_y = 0.0


def world_to_pix(wx, wy):
    px = int(MAP_CENTRE + ((wx - origin_x) / WORLD_X_MAX) * MAP_CENTRE)
    py = int(MAP_CENTRE - ((wy - origin_y) / WORLD_Y_MAX) * MAP_CENTRE)
    return max(0, min(MAP_SIZE - 1, px)), max(0, min(MAP_SIZE - 1, py))


def beam_angle(i, n):
    return (LIDAR_FOV / 2.0) - (LIDAR_FOV * i / n)


# ═══════════════════════════════════════════════════════════════════
#  SLAM — occupancy grid + Bresenham ray cast
# ═══════════════════════════════════════════════════════════════════
hits   = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]
visits = [[0] * MAP_SIZE for _ in range(MAP_SIZE)]


def get_occupancy(x, y):
    v = visits[y][x]
    return 0.0 if v == 0 else hits[y][x] / v


def is_wall(x, y):
    return get_occupancy(x, y) > WALL_CERTAINTY


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


def scan_lidar(robot_x, robot_y, heading, ranges):
    n = len(ranges)
    max_range = lidar.getMaxRange() - 0.05
    rpx, rpy  = world_to_pix(robot_x, robot_y)
    for i, r in enumerate(ranges):
        if r <= 0.0 or math.isinf(r) or math.isnan(r):
            continue
        angle   = heading + beam_angle(i, n)
        is_hit  = r <= max_range
        ray_len = r if is_hit else max_range
        hx = robot_x + ray_len * math.cos(angle)
        hy = robot_y + ray_len * math.sin(angle)
        epx, epy = world_to_pix(hx, hy)
        for cx, cy in bresenham(rpx, rpy, epx, epy):
            if not (0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE):
                continue
            if cx == epx and cy == epy and is_hit:
                hits[cy][cx]   += 1
                visits[cy][cx] += 1
            else:
                visits[cy][cx] += 1


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
#  Radio homing
# ═══════════════════════════════════════════════════════════════════
def read_radio():
    best_s = -1.0
    best_b = 0.0
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        if "SOS" in msg:
            s = receiver.getSignalStrength()
            d = receiver.getEmitterDirection()
            b = math.atan2(d[1], d[0])
            if s > best_s:
                best_s, best_b = s, b
        receiver.nextPacket()
    if best_s <= 0:
        return INF, 0.0, 0.0
    return 1.0 / math.sqrt(best_s), best_b, best_s


def steer_to_bearing(bearing):
    """Proportional turn toward a robot-relative bearing (0=ahead, CCW+)."""
    error = bearing
    while error >  math.pi: error -= 2.0 * math.pi
    while error < -math.pi: error += 2.0 * math.pi
    K_TURN  = 4.0
    omega   = K_TURN * error
    forward = CRUISE_SPEED * max(0.0, 1.0 - 2.0 * abs(error) / math.pi)
    lv = max(-MAX_SPEED, min(MAX_SPEED, forward - omega))
    rv = max(-MAX_SPEED, min(MAX_SPEED, forward + omega))
    return lv, rv


# ═══════════════════════════════════════════════════════════════════
#  Camera — red detection + live preview on SLAM display
# ═══════════════════════════════════════════════════════════════════
def scan_for_red():
    """Return (found: bool, count: int) for (255,0,0)-like pixels."""
    img   = camera.getImage()
    count = 0
    for cy in range(CAM_H):
        for cx in range(CAM_W):
            r = camera.imageGetRed(img,   CAM_W, cx, cy)
            g = camera.imageGetGreen(img, CAM_W, cx, cy)
            b = camera.imageGetBlue(img,  CAM_W, cx, cy)
            if r > RED_R_MIN and g < RED_G_MAX and b < RED_B_MAX:
                count += 1
    return count >= RED_PIXEL_MIN, count


def draw_camera_preview():
    """Blit a half-scale camera frame into the top-right corner of the SLAM display."""
    img = camera.getImage()
    for cy in range(0, CAM_H, 2):
        for cx in range(0, CAM_W, 2):
            r = camera.imageGetRed(img,   CAM_W, cx, cy)
            g = camera.imageGetGreen(img, CAM_W, cx, cy)
            b = camera.imageGetBlue(img,  CAM_W, cx, cy)
            display.setColor((r << 16) | (g << 8) | b)
            display.drawPixel(CAM_PREV_X + cx // 2, CAM_PREV_Y + cy // 2)
    # White border
    display.setColor(0xFFFFFF)
    display.drawRectangle(CAM_PREV_X - 1, CAM_PREV_Y - 1,
                          CAM_W // 2 + 2, CAM_H // 2 + 2)


# ═══════════════════════════════════════════════════════════════════
#  Reactive avoidance + search spiral
# ═══════════════════════════════════════════════════════════════════
AVOID_DISTANCE   = 0.22
TURN_SPEED       = 4.0
SPIRAL_OMEGA0    = 3.0
SPIRAL_OMEGA_MIN = 0.4
SPIRAL_DECAY     = 0.004


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
    return (min(fv) if fv else INF,
            min(lv) if lv else INF,
            min(rv) if rv else INF)


def avoid_or(lv, rv, ranges):
    if not ranges:
        return lv, rv
    front, left, right = lidar_sectors(ranges)
    if front < AVOID_DISTANCE:
        return (-TURN_SPEED, TURN_SPEED) if left > right else (TURN_SPEED, -TURN_SPEED)
    return lv, rv


def spiral_cmd(t):
    omega = max(SPIRAL_OMEGA_MIN, SPIRAL_OMEGA0 - SPIRAL_DECAY * t)
    return CRUISE_SPEED - omega, CRUISE_SPEED + omega


# ═══════════════════════════════════════════════════════════════════
#  Display — SLAM + trajectory + pinger marker + camera preview
# ═══════════════════════════════════════════════════════════════════
def draw_map(robot_px, robot_py, trajectory,
             pinger_pix=None, show_camera=False):
    display.setColor(0xDDDDDD)
    display.fillRectangle(0, 0, MAP_SIZE, MAP_SIZE)

    # Occupancy grid
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

    # Trajectory — cyan polyline (every 4th point to keep it fast)
    display.setColor(0x00CCCC)
    traj = trajectory[::4]
    for k in range(1, len(traj)):
        x0, y0 = world_to_pix(*traj[k - 1])
        x1, y1 = world_to_pix(*traj[k])
        display.drawLine(x0, y0, x1, y1)

    # Origin — white cross
    display.setColor(0xFFFFFF)
    display.drawLine(MAP_CENTRE - 5, MAP_CENTRE, MAP_CENTRE + 5, MAP_CENTRE)
    display.drawLine(MAP_CENTRE, MAP_CENTRE - 5, MAP_CENTRE, MAP_CENTRE + 5)

    # Pinger — red square (drawn when proximity stop triggers)
    if pinger_pix is not None:
        display.setColor(0xFF0000)
        display.fillRectangle(pinger_pix[0] - 4, pinger_pix[1] - 4, 8, 8)

    # Robot — blue dot
    display.setColor(0x0000FF)
    display.fillOval(robot_px, robot_py, 3, 3)

    # Camera preview (top-right corner)
    if show_camera:
        draw_camera_preview()


# ═══════════════════════════════════════════════════════════════════
#  Image save — matplotlib composite, multi-robot ready
# ═══════════════════════════════════════════════════════════════════
def save_map_image(robot_maps, filename="slam_map.png"):
    """
    Render a composite SLAM map from one or more robots.

    Parameters
    ----------
    robot_maps : list[dict], each dict contains:
        hits       – MAP_SIZE × MAP_SIZE 2-D list (int)
        visits     – MAP_SIZE × MAP_SIZE 2-D list (int)
        origin     – (world_x, world_y) of this robot's starting position
        trajectory – list of (world_x, world_y) world-coord tuples
        pinger_pos – (world_x, world_y) or None
        label      – display name, e.g. "Robot 1"

    Multi-robot merging
    -------------------
    Pixel (px, py) in robot i's local grid sits at world position:
        wx = origin[0] + (px - MAP_CENTRE) / MAP_CENTRE * WORLD_X_MAX
        wy = origin[1] - (py - MAP_CENTRE) / MAP_CENTRE * WORLD_Y_MAX
    Its offset in the global grid (global MAP_CENTRE = world origin (0, 0)):
        off_px =  round(origin[0] / WORLD_X_MAX * MAP_CENTRE)
        off_py = -round(origin[1] / WORLD_Y_MAX * MAP_CENTRE)
    Shift each robot's hits/visits grid by (off_px, off_py), then sum,
    to build one merged occupancy grid covering all robots' views.
    To generate map1…mapN iteratively, call this function with
    robot_maps[:1], robot_maps[:2], …, robot_maps[:N].
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

        # Source slice (local grid) and destination slice (global grid)
        sx0 = max(0, -off_px);  sx1 = min(MAP_SIZE, MAP_SIZE - off_px)
        dx0 = max(0,  off_px);  dx1 = min(MAP_SIZE, MAP_SIZE + off_px)
        sy0 = max(0, -off_py);  sy1 = min(MAP_SIZE, MAP_SIZE - off_py)
        dy0 = max(0,  off_py);  dy1 = min(MAP_SIZE, MAP_SIZE + off_py)

        if sx0 < sx1 and sy0 < sy1:
            g_hits  [dy0:dy1, dx0:dx1] += h[sy0:sy1, sx0:sx1]
            g_visits[dy0:dy1, dx0:dx1] += v[sy0:sy1, sx0:sx1]

    # ── Occupancy probability; NaN = never visited ────────────────────
    with np.errstate(divide="ignore", invalid="ignore"):
        occ = np.where(g_visits > 0, g_hits / g_visits, np.nan)

    # ── RGBA image: unexplored=light-grey, free=white, wall=dark-grey ─
    # LiDAR ray traces are intentionally suppressed: visited free cells
    # are rendered identical to unvisited free space (plain white).
    img = np.full((MAP_SIZE, MAP_SIZE, 4), [0.93, 0.93, 0.93, 1.0],
                  dtype=np.float32)
    free_mask = (~np.isnan(occ)) & (occ <= WALL_CERTAINTY)
    img[free_mask] = [1.0, 1.0, 1.0, 1.0]
    wall_mask = (~np.isnan(occ)) & (occ > WALL_CERTAINTY)
    t_wall = np.clip(
        (occ[wall_mask] - WALL_CERTAINTY) / (IMPASSABLE - WALL_CERTAINTY), 0, 1)
    shade = 0.35 * (1.0 - t_wall)
    img[wall_mask, 0] = shade
    img[wall_mask, 1] = shade
    img[wall_mask, 2] = shade

    # ── Figure ────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 7), dpi=150)
    ax.imshow(img, origin="upper",
              extent=[-WORLD_X_MAX, WORLD_X_MAX, -WORLD_Y_MAX, WORLD_Y_MAX],
              aspect="equal", interpolation="nearest")
    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Y (m)", fontsize=9)
    n = len(robot_maps)
    ax.set_title("SLAM Map — %d robot%s" % (n, "s" if n != 1 else ""),
                 fontsize=11)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.4, color="#888888")

    # ── Legend base: map-layer patches ───────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor=(0.20, 0.20, 0.20),
                       label="Wall"),
        mpatches.Patch(facecolor=(1.00, 1.00, 1.00),
                       edgecolor="gray", linewidth=0.5, label="Free (explored)"),
        mpatches.Patch(facecolor=(0.93, 0.93, 0.93),
                       edgecolor="gray", linewidth=0.5, label="Unexplored"),
    ]

    # ── Per-robot overlays ────────────────────────────────────────────
    colors = plt.cm.tab10.colors          # up to 10 visually distinct colors

    for i, rm in enumerate(robot_maps):
        c    = colors[i % len(colors)]
        lbl  = rm.get("label", "Robot %d" % (i + 1))
        ox, oy = rm["origin"]
        traj = rm.get("trajectory", [])
        ppos = rm.get("pinger_pos", None)

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

        if ppos is not None:
            ax.plot(ppos[0], ppos[1], marker="*", color="crimson",
                    markersize=14, zorder=6,
                    markeredgecolor="darkred", markeredgewidth=0.5)
            legend_handles.append(
                Line2D([0], [0], marker="*", color="w",
                       markerfacecolor="crimson", markersize=10,
                       label="%s distress signal" % lbl))

    ax.legend(handles=legend_handles, loc="upper left",
              fontsize=7, framealpha=0.9, edgecolor="gray")
    fig.tight_layout()
    fig.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[scout] map saved → " + filename)


# ═══════════════════════════════════════════════════════════════════
#  Main loop
# ═══════════════════════════════════════════════════════════════════
# Initialise origin from first GPS reading
_first_step = True

prev_cell     = (-1, -1)
step_count    = 0
ping_received = False
last_range    = INF
last_bearing  = 0.0
last_strength = 0.0
prev_range    = INF
person_found  = False
camera_active = False
trajectory    = []
pinger_pix    = None    # pixel coords for the Webots display
pinger_world  = None    # world coords (wx, wy) for the saved image

print("[scout] online — spiral search until SOS heard")

while robot.step(TIME_STEP) != -1:
    step_count += 1
    sim_time    = step_count * TIME_STEP / 1000.0

    vals = gps.getValues()
    robot_x, robot_y = vals[0], vals[1]

    # Set map origin to robot's starting position
    if _first_step:
        origin_x  = robot_x
        origin_y  = robot_y
        _first_step = False
        print("[scout] map origin set to (%.3f, %.3f)" % (origin_x, origin_y))

    compass_vals = compass.getValues()
    heading      = math.atan2(compass_vals[0], compass_vals[1])

    robot_px, robot_py = world_to_pix(robot_x, robot_y)
    current_cell = (robot_px, robot_py)
    ranges       = lidar.getRangeImage()

    # SLAM
    if current_cell != prev_cell:
        prev_cell = current_cell
        trajectory.append((robot_x, robot_y))
        if ranges:
            scan_lidar(robot_x, robot_y, heading, ranges)
            inflate_obstacles()

    # Radio — polled every step for fresh bearing + range
    r, b, s = read_radio()
    if r < INF:
        prev_range    = last_range
        last_range    = r
        last_bearing  = b
        last_strength = s
        if not ping_received:
            ping_received = True
            print("[scout] SOS heard — engaging radio homing")
            print("        strength=%.5f | range≈%.2f m | bearing=%+.0f°"
                  % (s, r, math.degrees(b)))

    # Systematic signal updates every 2 sim-seconds while homing
    if ping_received and last_range < INF and step_count % SIGNAL_PRINT_STEPS == 0:
        trend = "closing  ↓" if last_range < prev_range else "moving away ↑"
        print("[radio] strength=%.5f | range≈%.2f m | bearing=%+.0f° | %s"
              % (last_strength, last_range, math.degrees(last_bearing), trend))

    # Time limit
    if sim_time >= SIM_TIME_LIMIT:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("[scout] time limit (%.1f s)" % sim_time)
        draw_map(robot_px, robot_py, trajectory, pinger_pix, camera_active)
        save_map_image([{
            "hits": hits, "visits": visits,
            "origin": (origin_x, origin_y),
            "trajectory": trajectory,
            "pinger_pos": pinger_world,
            "label": "Robot 1",
        }])
        break

    # ── Phase 1: spiral search ────────────────────────────────────
    if not ping_received:
        lv, rv = spiral_cmd(step_count)
        lv, rv = avoid_or(lv, rv, ranges)
        left_motor.setVelocity(lv)
        right_motor.setVelocity(rv)
        if step_count % DRAW_INTERVAL == 0:
            draw_map(robot_px, robot_py, trajectory)
        continue

    # ── Phase 2: radio homing ─────────────────────────────────────

    # Proximity stop
    if last_range < RADIO_STOP_DIST:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        pinger_pix   = (robot_px, robot_py)
        pinger_world = (robot_x, robot_y)
        print("[scout] proximity stop — est. %.2f m in %.1f s"
              % (last_range, sim_time))
        draw_map(robot_px, robot_py, trajectory, pinger_pix, camera_active)
        save_map_image([{
            "hits": hits, "visits": visits,
            "origin": (origin_x, origin_y),
            "trajectory": trajectory,
            "pinger_pos": pinger_world,
            "label": "Robot 1",
        }])
        break

    # Camera: activate, preview, scan
    if last_range < CAMERA_DIST:
        if not camera_active:
            camera_active = True
            print("[camera] activated — robot est. %.2f m from signal source" % last_range)
        if not person_found:
            found, count = scan_for_red()
            if step_count % CAM_PRINT_STEPS == 0:
                print("[camera] %d red pixels detected (threshold: %d)"
                      % (count, RED_PIXEL_MIN))
            if found:
                person_found = True
                print("FOUND PERSON")

    # Steer toward radio bearing.
    # Obstacle avoidance suppressed within CAMERA_DIST — the beacon reads as an obstacle.
    lv, rv = steer_to_bearing(last_bearing)
    if last_range > CAMERA_DIST:
        lv, rv = avoid_or(lv, rv, ranges)
    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)

    if step_count % DRAW_INTERVAL == 0:
        draw_map(robot_px, robot_py, trajectory, pinger_pix, camera_active)
