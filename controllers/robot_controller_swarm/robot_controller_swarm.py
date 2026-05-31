"""
robot_controller_swarm — one scout in a 3-robot SAR swarm.

Each scout is parameterised from the world file via controllerArgs:
    controllerArgs ["<id>" "<colour>"]
        id     — "1" | "2" | "3"
        colour — "red" | "yellow" | "green"   (matches its pinger)

Bound by the world file: receiver_channel == pinger emitter channel.
So colour ↔ channel ↔ scout is set in arena_swarm.wbt; this code just
honours its assigned identity.

Inter-robot coordination: when a scout reaches its person it pickles
its full SLAM state to  ./slam_state_<id>.pkl . Every scout watches
that directory; the n-th scout to arrive loads the n state files in
arrival order and writes  ./slam_map_stage<n>.png  — a combined map
re-centred on the FIRST scout's origin (its start becomes the new (0,0)).
"""
from controller import Robot
import math, os, sys, time, pickle, glob

# ── Identity from controllerArgs ──────────────────────────────────────────
#   controllerArgs ["<id>" "<colour>" "<start_x>" "<start_y>"]
# start_x / start_y are the scout's world translation, declared in the world
# file. They are the ONLY cross-robot spatial datum we keep — no GPS reads.
# Each scout maps in its own local frame (origin = its own start); the merge
# step uses the saved start vectors to translate peers into the first
# finder's frame.
ROBOT_ID    = sys.argv[1] if len(sys.argv) > 1 else "1"
TARGET_COL  = (sys.argv[2] if len(sys.argv) > 2 else "red").lower()
START_X     = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
START_Y     = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
LABEL       = "Robot %s (%s)" % (ROBOT_ID, TARGET_COL)

# Colour-specific camera thresholds: (R_min, R_max, G_min, G_max, B_min, B_max)
COLOUR_BANDS = {
    "red":    (200, 255,   0,  60,   0,  60),
    "yellow": (200, 255, 200, 255,   0,  80),
    "green":  (  0,  80, 200, 255,   0,  80),
}
R_MIN, R_MAX, G_MIN, G_MAX, B_MIN, B_MAX = COLOUR_BANDS.get(
    TARGET_COL, COLOUR_BANDS["red"])
TARGET_PIXEL_MIN = 20

# ── Tuning ────────────────────────────────────────────────────────────────
TIME_STEP        = 32
CRUISE_SPEED     = 3.0
MAX_SPEED        = 6.28
SIM_TIME_LIMIT   = 180.0

RADIO_STOP_DIST  = 0.35
CAMERA_DIST      = 0.80

MAP_SIZE         = 400
MAP_CENTRE       = MAP_SIZE // 2
WORLD_X_MAX      = 4.5
WORLD_Y_MAX      = 3.5
OBSTACLE_INFLATE = 3

WALL_CERTAINTY   = 0.30
IMPASSABLE       = 0.90

DRAW_INTERVAL        = 16
SIGNAL_PRINT_STEPS   = int(2.0 * 1000 / TIME_STEP)
CAM_PRINT_STEPS      = int(1.0 * 1000 / TIME_STEP)

INF = float("inf")

# Shared-state directory (controller's CWD). Stamp on session start so
# stale state files from a prior run are ignored.
SESSION_START   = time.time()
STATE_GLOB      = "slam_state_*.pkl"
OWN_STATE_FILE  = "slam_state_%s.pkl" % ROBOT_ID

# Clear our own stale state file so a previous run's "found" doesn't
# poison this session's staged map.
try:
    if os.path.exists(OWN_STATE_FILE):
        os.remove(OWN_STATE_FILE)
except OSError:
    pass

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

# ── Wheel-encoder odometry (no GPS) ───────────────────────────────────────
# e-puck constants
WHEEL_RADIUS = 0.0205   # m
AXLE_LENGTH  = 0.052    # m  (wheel separation)

left_ps  = robot.getDevice("left wheel sensor")
right_ps = robot.getDevice("right wheel sensor")
left_ps.enable(TIME_STEP)
right_ps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

display = robot.getDevice("map_display")

CAM_PREV_X = MAP_SIZE - CAM_W // 2 - 2
CAM_PREV_Y = 2

# ═══════════════════════════════════════════════════════════════════
#  Coordinate helpers — everything lives in this scout's LOCAL frame
#  (origin = (0,0) = its own start). Compass gives global heading, so
#  every scout's local frame shares the same orientation as the world,
#  which means peer maps can be merged by pure translation.
# ═══════════════════════════════════════════════════════════════════
def world_to_pix(lx, ly):
    # lx, ly are already in the local frame (start = origin).
    px = int(MAP_CENTRE + (lx / WORLD_X_MAX) * MAP_CENTRE)
    py = int(MAP_CENTRE - (ly / WORLD_Y_MAX) * MAP_CENTRE)
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
#  Camera — colour-tuned target detection + live preview
# ═══════════════════════════════════════════════════════════════════
def scan_for_target():
    img   = camera.getImage()
    count = 0
    for cy in range(CAM_H):
        for cx in range(CAM_W):
            r = camera.imageGetRed(img,   CAM_W, cx, cy)
            g = camera.imageGetGreen(img, CAM_W, cx, cy)
            b = camera.imageGetBlue(img,  CAM_W, cx, cy)
            if (R_MIN <= r <= R_MAX and
                G_MIN <= g <= G_MAX and
                B_MIN <= b <= B_MAX):
                count += 1
    return count >= TARGET_PIXEL_MIN, count


def draw_camera_preview():
    img = camera.getImage()
    for cy in range(0, CAM_H, 2):
        for cx in range(0, CAM_W, 2):
            r = camera.imageGetRed(img,   CAM_W, cx, cy)
            g = camera.imageGetGreen(img, CAM_W, cx, cy)
            b = camera.imageGetBlue(img,  CAM_W, cx, cy)
            display.setColor((r << 16) | (g << 8) | b)
            display.drawPixel(CAM_PREV_X + cx // 2, CAM_PREV_Y + cy // 2)
    display.setColor(0xFFFFFF)
    display.drawRectangle(CAM_PREV_X - 1, CAM_PREV_Y - 1,
                          CAM_W // 2 + 2, CAM_H // 2 + 2)


# ═══════════════════════════════════════════════════════════════════
#  Reactive avoidance + search spiral
# ═══════════════════════════════════════════════════════════════════
# ── Proactive, proportional avoidance ─────────────────────────────────────
# Earlier trigger (FAR) lets the scout steer around obstacles smoothly
# instead of charging until NEAR forces a spin-in-place. The bias only
# scales up as proximity grows, so it won't latch onto a wall (no rigid
# wall-following) and the maze approach stays responsive.
AVOID_FAR        = 0.50    # start veering at this front distance
AVOID_NEAR       = 0.18    # full pivot / hard stop below this
TURN_SPEED       = 4.5
SIDE_BIAS_GAIN   = 1.2     # gentle pushback from side clearances
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
    """Proportional steer-and-slow. urgency=0 at AVOID_FAR, =1 at AVOID_NEAR.
    Output blends commanded (lv,rv) with a turn bias toward the more open
    side, plus light side-bias from left/right clearances."""
    if not ranges:
        return lv, rv
    front, left, right = lidar_sectors(ranges)

    # Side pushback (always active but small): pushes away from a near side wall
    side_bias = 0.0
    if left  < AVOID_FAR: side_bias -= SIDE_BIAS_GAIN * (AVOID_FAR - left)
    if right < AVOID_FAR: side_bias += SIDE_BIAS_GAIN * (AVOID_FAR - right)

    if front >= AVOID_FAR:
        # Apply side pushback only.
        return (lv - side_bias, rv + side_bias) if abs(side_bias) > 0.01 else (lv, rv)

    # Front cone is encroaching → compute urgency 0..1
    if front <= AVOID_NEAR:
        urgency = 1.0
    else:
        urgency = 1.0 - (front - AVOID_NEAR) / (AVOID_FAR - AVOID_NEAR)

    # Forward speed scales down with urgency, but never quite halts unless very close
    base = ((lv + rv) * 0.5) * max(0.0, 1.0 - 0.85 * urgency)
    if urgency >= 1.0:
        base = 0.0

    bias = TURN_SPEED * urgency
    if left > right:
        out_l, out_r = base - bias, base + bias
    else:
        out_l, out_r = base + bias, base - bias

    out_l -= side_bias
    out_r += side_bias
    out_l = max(-MAX_SPEED, min(MAX_SPEED, out_l))
    out_r = max(-MAX_SPEED, min(MAX_SPEED, out_r))
    return out_l, out_r


def spiral_cmd(t):
    omega = max(SPIRAL_OMEGA_MIN, SPIRAL_OMEGA0 - SPIRAL_DECAY * t)
    return CRUISE_SPEED - omega, CRUISE_SPEED + omega


# ═══════════════════════════════════════════════════════════════════
#  Display
# ═══════════════════════════════════════════════════════════════════
def draw_map(robot_px, robot_py, trajectory,
             pinger_pix=None, show_camera=False):
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

    display.setColor(0x00CCCC)
    traj = trajectory[::4]
    for k in range(1, len(traj)):
        x0, y0 = world_to_pix(*traj[k - 1])
        x1, y1 = world_to_pix(*traj[k])
        display.drawLine(x0, y0, x1, y1)

    display.setColor(0xFFFFFF)
    display.drawLine(MAP_CENTRE - 5, MAP_CENTRE, MAP_CENTRE + 5, MAP_CENTRE)
    display.drawLine(MAP_CENTRE, MAP_CENTRE - 5, MAP_CENTRE, MAP_CENTRE + 5)

    if pinger_pix is not None:
        display.setColor(0xFF0000)
        display.fillRectangle(pinger_pix[0] - 4, pinger_pix[1] - 4, 8, 8)

    display.setColor(0x0000FF)
    display.fillOval(robot_px, robot_py, 3, 3)

    if show_camera:
        draw_camera_preview()


# ═══════════════════════════════════════════════════════════════════
#  Multi-robot map merge — staged combined PNG re-centred on first finder
# ═══════════════════════════════════════════════════════════════════
def save_map_image(robot_maps, filename, ref_start):
    """Render up to N robot maps into one PNG, translated so that the
    FIRST finder's start position becomes (0,0). Each peer map is
    stored in its own local frame; the only datum needed to align them
    is the start translation vector saved at world-build time."""
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.lines import Line2D
    except ImportError:
        print("[scout %s] install matplotlib + numpy:  pip install matplotlib numpy"
              % ROBOT_ID)
        return

    rsx, rsy = ref_start

    g_hits   = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
    g_visits = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)

    for rm in robot_maps:
        sx, sy = rm["start_pos"]
        # Translation that places this scout's local (0,0) at the
        # right spot in the first finder's frame.
        dx, dy = sx - rsx, sy - rsy
        off_px =  round(dx / WORLD_X_MAX * MAP_CENTRE)
        off_py = -round(dy / WORLD_Y_MAX * MAP_CENTRE)
        h = np.array(rm["hits"],   dtype=np.int32)
        v = np.array(rm["visits"], dtype=np.int32)

        sx0 = max(0, -off_px);  sx1 = min(MAP_SIZE, MAP_SIZE - off_px)
        dx0 = max(0,  off_px);  dx1 = min(MAP_SIZE, MAP_SIZE + off_px)
        sy0 = max(0, -off_py);  sy1 = min(MAP_SIZE, MAP_SIZE - off_py)
        dy0 = max(0,  off_py);  dy1 = min(MAP_SIZE, MAP_SIZE + off_py)

        if sx0 < sx1 and sy0 < sy1:
            g_hits  [dy0:dy1, dx0:dx1] += h[sy0:sy1, sx0:sx1]
            g_visits[dy0:dy1, dx0:dx1] += v[sy0:sy1, sx0:sx1]

    with np.errstate(divide="ignore", invalid="ignore"):
        occ = np.where(g_visits > 0, g_hits / g_visits, np.nan)

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

    # Wider figure + legend anchored outside the axes (right column).
    fig, ax = plt.subplots(figsize=(11, 7), dpi=150)
    ax.imshow(img, origin="upper",
              extent=[-WORLD_X_MAX, WORLD_X_MAX, -WORLD_Y_MAX, WORLD_Y_MAX],
              aspect="equal", interpolation="nearest")
    ax.set_xlabel("X relative to first finder's start (m)", fontsize=9)
    ax.set_ylabel("Y relative to first finder's start (m)", fontsize=9)
    n = len(robot_maps)
    first_lbl = robot_maps[0].get("label", "first finder")
    ax.set_title("SLAM Map — stage %d (%d robot%s arrived) · frame: %s"
                 % (n, n, "s" if n != 1 else "", first_lbl), fontsize=11)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.4, color="#888888")

    legend_handles = [
        mpatches.Patch(facecolor=(0.20, 0.20, 0.20), label="Wall"),
        mpatches.Patch(facecolor=(1.00, 1.00, 1.00),
                       edgecolor="gray", linewidth=0.5, label="Free (explored)"),
        mpatches.Patch(facecolor=(0.93, 0.93, 0.93),
                       edgecolor="gray", linewidth=0.5, label="Unexplored"),
    ]

    colour_map = {"red": "#d62728", "yellow": "#bcbd22", "green": "#2ca02c"}

    for rm in robot_maps:
        lbl  = rm.get("label", "Robot ?")
        col  = colour_map.get(rm.get("colour", ""), "#1f77b4")
        sx, sy = rm["start_pos"]
        sx_r, sy_r = sx - rsx, sy - rsy
        traj = rm.get("trajectory", [])           # local coords (start = 0,0)
        ppos = rm.get("pinger_pos", None)         # local coords

        if len(traj) > 1:
            tx = [p[0] + sx_r for p in traj]
            ty = [p[1] + sy_r for p in traj]
            ax.plot(tx, ty, color=col, linewidth=1.1, alpha=0.85, zorder=3)
            legend_handles.append(
                Line2D([0], [0], color=col, linewidth=1.5,
                       label="%s trajectory" % lbl))

        ax.plot(sx_r, sy_r, marker="P", color=col, markersize=9,
                markeredgecolor="white", markeredgewidth=0.7, zorder=5)
        legend_handles.append(
            Line2D([0], [0], marker="P", color="w", markerfacecolor=col,
                   markersize=8, label="%s start" % lbl))

        if ppos is not None:
            px, py = ppos[0] + sx_r, ppos[1] + sy_r
            ax.plot(px, py, marker="*", color=col, markersize=14, zorder=6,
                    markeredgecolor="black", markeredgewidth=0.5)
            legend_handles.append(
                Line2D([0], [0], marker="*", color="w", markerfacecolor=col,
                       markersize=10, label="%s person found" % lbl))

    # Legend lives in its own column to the right of the map.
    ax.legend(handles=legend_handles, bbox_to_anchor=(1.02, 1.0),
              loc="upper left", borderaxespad=0.0,
              fontsize=8, framealpha=0.95, edgecolor="gray")
    fig.subplots_adjust(left=0.07, right=0.72, top=0.93, bottom=0.08)
    fig.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("[scout %s] map saved → %s" % (ROBOT_ID, filename))


# ═══════════════════════════════════════════════════════════════════
#  Shared-state I/O — pickle-based handoff between scouts
# ═══════════════════════════════════════════════════════════════════
def write_own_state(pinger_local, trajectory, find_time):
    state = {
        "id":         ROBOT_ID,
        "label":      LABEL,
        "colour":     TARGET_COL,
        "start_pos":  (START_X, START_Y),   # world translation from world file
        "hits":       hits,
        "visits":     visits,
        "trajectory": list(trajectory),     # LOCAL coords (start = 0,0)
        "pinger_pos": pinger_local,         # LOCAL coords (start = 0,0)
        "find_time":  find_time,            # sim seconds since this scout started
        "wall_time":  time.time(),          # for cross-scout ordering
    }
    tmp = OWN_STATE_FILE + ".tmp"
    with open(tmp, "wb") as f:
        pickle.dump(state, f, protocol=pickle.HIGHEST_PROTOCOL)
    os.replace(tmp, OWN_STATE_FILE)


def load_all_states():
    """Return all session-valid state files sorted by arrival (wall_time)."""
    out = []
    for path in glob.glob(STATE_GLOB):
        try:
            if os.path.getmtime(path) < SESSION_START - 1.0:
                continue                       # stale from a previous run
            with open(path, "rb") as f:
                out.append(pickle.load(f))
        except (OSError, EOFError, pickle.UnpicklingError):
            continue
    out.sort(key=lambda s: s["wall_time"])
    return out


def emit_staged_map(pinger_local, trajectory, find_time):
    """Write our state, then load every state on disk and render the
    combined map for the current stage (= number of scouts arrived).
    The first finder's start translation defines the merged frame."""
    write_own_state(pinger_local, trajectory, find_time)
    states = load_all_states()
    if not states:
        return
    ref_start = states[0]["start_pos"]          # first finder = global (0,0)
    stage     = len(states)
    fname     = "slam_map_stage%d.png" % stage
    print("[scout %s] %d/%d scouts arrived — rendering %s"
          % (ROBOT_ID, stage, 3, fname))
    save_map_image(states, fname, ref_start)


# ═══════════════════════════════════════════════════════════════════
#  Main loop
# ═══════════════════════════════════════════════════════════════════
_first_step   = True
prev_l        = 0.0
prev_r        = 0.0
local_x       = 0.0      # odometry-tracked position in this scout's local frame
local_y       = 0.0
prev_cell     = (-1, -1)
step_count    = 0
ping_received = False
last_range    = INF
last_bearing  = 0.0
last_strength = 0.0
prev_range    = INF
person_found  = False
camera_active = False
reached       = False
trajectory    = []
pinger_pix    = None
pinger_local  = None

print("[scout %s] online — colour=%s — start=(%.2f, %.2f) — spiral search until SOS heard"
      % (ROBOT_ID, TARGET_COL, START_X, START_Y))

while robot.step(TIME_STEP) != -1:
    step_count += 1
    sim_time    = step_count * TIME_STEP / 1000.0

    # ── Compass: global heading (consistent across scouts → translation-only merge) ──
    compass_vals = compass.getValues()
    heading      = math.atan2(compass_vals[0], compass_vals[1])

    # ── Wheel-encoder odometry: integrate displacement in local frame ──
    l = left_ps.getValue()
    r = right_ps.getValue()
    if _first_step:
        prev_l, prev_r = l, r
        _first_step = False
        print("[scout %s] odometry zeroed — local frame anchored at start"
              % ROBOT_ID)
    dl = (l - prev_l) * WHEEL_RADIUS
    dr = (r - prev_r) * WHEEL_RADIUS
    prev_l, prev_r = l, r
    ds = 0.5 * (dl + dr)
    local_x += ds * math.cos(heading)
    local_y += ds * math.sin(heading)

    robot_px, robot_py = world_to_pix(local_x, local_y)
    current_cell = (robot_px, robot_py)
    ranges       = lidar.getRangeImage()

    if current_cell != prev_cell:
        prev_cell = current_cell
        trajectory.append((local_x, local_y))
        if ranges:
            scan_lidar(local_x, local_y, heading, ranges)
            inflate_obstacles()

    r, b, s = read_radio()
    if r < INF:
        prev_range    = last_range
        last_range    = r
        last_bearing  = b
        last_strength = s
        if not ping_received:
            ping_received = True
            print("[scout %s] SOS heard — engaging radio homing" % ROBOT_ID)
            print("           strength=%.5f | range≈%.2f m | bearing=%+.0f°"
                  % (s, r, math.degrees(b)))

    if ping_received and last_range < INF and step_count % SIGNAL_PRINT_STEPS == 0:
        trend = "closing  ↓" if last_range < prev_range else "moving away ↑"
        print("[radio %s] strength=%.5f | range≈%.2f m | bearing=%+.0f° | %s"
              % (ROBOT_ID, last_strength, last_range, math.degrees(last_bearing), trend))

    # Already reached — passive: just listen for new states from peers
    # so the latest stage map keeps being regenerated as they arrive.
    if reached:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        if step_count % int(1.0 * 1000 / TIME_STEP) == 0:
            # Re-render if disk gained a new state since the last render.
            existing = len(load_all_states())
            if existing > getattr(emit_staged_map, "_last_stage", 0):
                states_now = load_all_states()
                ref_start  = states_now[0]["start_pos"]
                fname      = "slam_map_stage%d.png" % existing
                print("[scout %s] peer arrival detected — rendering %s"
                      % (ROBOT_ID, fname))
                save_map_image(states_now, fname, ref_start)
                emit_staged_map._last_stage = existing
        if sim_time >= SIM_TIME_LIMIT:
            break
        continue

    if sim_time >= SIM_TIME_LIMIT:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("[scout %s] time limit (%.1f s) — gave up" % (ROBOT_ID, sim_time))
        draw_map(robot_px, robot_py, trajectory, pinger_pix, camera_active)
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

    if last_range < RADIO_STOP_DIST:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        pinger_pix   = (robot_px, robot_py)
        pinger_local = (local_x, local_y)
        print("[scout %s] proximity stop — est. %.2f m in %.1f s"
              % (ROBOT_ID, last_range, sim_time))
        draw_map(robot_px, robot_py, trajectory, pinger_pix, camera_active)
        reached = True
        emit_staged_map(pinger_local, trajectory, sim_time)
        emit_staged_map._last_stage = len(load_all_states())
        continue

    if last_range < CAMERA_DIST:
        if not camera_active:
            camera_active = True
            print("[camera %s] activated — robot est. %.2f m from signal source"
                  % (ROBOT_ID, last_range))
        if not person_found:
            found, count = scan_for_target()
            if step_count % CAM_PRINT_STEPS == 0:
                print("[camera %s] %d %s pixels detected (threshold: %d)"
                      % (ROBOT_ID, count, TARGET_COL, TARGET_PIXEL_MIN))
            if found:
                person_found = True
                print("[scout %s] FOUND %s PERSON"
                      % (ROBOT_ID, TARGET_COL.upper()))

    lv, rv = steer_to_bearing(last_bearing)
    if last_range > CAMERA_DIST:
        lv, rv = avoid_or(lv, rv, ranges)
    left_motor.setVelocity(lv)
    right_motor.setVelocity(rv)

    if step_count % DRAW_INTERVAL == 0:
        draw_map(robot_px, robot_py, trajectory, pinger_pix, camera_active)
