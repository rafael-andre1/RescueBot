# Iteration 6: Continuous Auction Market, Opportunistic Rescue, and Benchmarking

## Goals

 1. Many beacons, one shared radio channel
    1. Beacons no longer own a private channel per colour
    2. Each beacon shouts `"SOS <id>"`; colour is decided later by the camera
 2. A market that decides who goes where
    1. Every RescueBot bids on every beacon it can hear, all the time
    2. A central auctioneer assigns, reassigns, and swaps targets to lower the total cost
 3. Pathfinding around discovered walls
    1. Bidirectional Dijkstra over the live occupancy grid
    2. Line of sight shortcut when the target is directly visible
 4. Resolve a ping by distance plus vision, not by touching it
    1. Within `RADIO_STOP_DIST` and confirmed by the camera
    2. Opportunistic: resolve any beacon passed in range, not only the assigned one
 5. One global map for the whole team, updated continuously
 6. A reproducible benchmark across map difficulty, team size, and goal rate

Iteration 5 bound colour, channel, and robot together in the world file, gave each RescueBot exactly one beacon, and merged maps only on arrival. Iteration 6 removes those bindings and turns the team into a continuously re-allocated fleet on a single channel, then measures it.

## What changed from Iteration 5

| Area | Iteration 5 (swarm) | Iteration 6 (this) |
|---|---|---|
| Beacon to robot binding | Fixed in the world file (1 colour, 1 channel, 1 robot) | Dynamic, decided by the auctioneer at runtime |
| Channels | One per colour (1, 2, 3) | One SOS channel (1) for all beacons, one bus channel (2) for the market |
| Number of beacons | 3, fixed | Unbounded, spawned over time |
| Target choice | Hardcoded | Bid based, reassignable, swappable |
| Pathfinding | Potential field only | Bidirectional Dijkstra plus gradient plus line of sight |
| Resolution rule | Range plus camera, assigned beacon only | Range plus camera, any beacon (opportunistic) |
| Map merge | On arrival, recentred on first finder | Continuous, recentred on the world origin |
| Evaluation | Visual maps | Headless sweep with CSV metrics and figures |

## World and Supervisor (`signal_controller.py`)

The supervisor is both the spawner and the auctioneer. It builds the arena, drops the RescueBots, activates beacons over time, runs the market, and (in benchmark mode) records metrics and quits.

### Procedural maps (`bench_maps.py`)

Walls are no longer authored in the world file. The supervisor reads a wall list keyed by difficulty and spawns each one as a `Solid` box at startup, so a single `arena.wbt` serves all three maps. The difficulty is chosen with the `RESCUE_MAP` environment variable.

| Map | Layout | Walls |
|---|---|---|
| `easy` | nearly open, two short walls | 2 |
| `medium` | the original arena | 7 |
| `hard` | serpentine maze with dead end stubs, centre kept clear for the spawn ring | 8 |

Each wall is `(centre_x, centre_y, size_x, size_y)` in metres, height fixed at `WALL_HEIGHT = 0.3`. The same list also feeds the beacon placement check, padded by `WALL_MARGIN = 0.30`, so a beacon never spawns inside or hugging a wall.

### Scout spawning and `controllerArgs`

`NUM_ROBOTS` RescueBots (default 5, overridable via `RESCUE_ROBOTS`) are dropped in a ring of radius `CLUSTER_RADIUS = 0.35` m around the arena centre. Each one is told its own world start through `controllerArgs`, which is the only spatial datum needed to merge per robot maps later.

```
controllerArgs ["<start_x>" "<start_y>"]
```

The robot id comes from the node name `scout_<id>`.

### Beacon spawning: Gaussian gap, separation, floating box

Beacons accumulate over the run (`NUM_GOALS = 0` means spawn forever). The first one appears at `t = 1.0` s. After that, the gap to the next spawn is drawn from a Gaussian whose mean (and median) is the preset interval:

$$ \text{gap} \sim \mathcal{N}(\text{BEACON\_INTERVAL}, \text{BEACON\_STD}) $$

The draw is clamped so it can never be zero or negative:

$$ \text{next\_spawn} = t + \max(\text{BEACON\_MIN\_GAP}, \text{gap}) $$

The schedule compounds off the actual spawn time, so a deferred spawn does not drift the cadence. A spawn is deferred (the schedule is left untouched and retried next step) when no legal position is free, which keeps the cadence honest under crowding instead of forcing a bad placement.

Position is rejected unless it is clear of walls and at least `MIN_BEACON_SEP` from every currently active beacon:

$$ \text{MIN\_BEACON\_SEP} = 2 \times \text{RADIO\_STOP\_DIST} = 1.6 \text{ m} $$

The factor of two is deliberate. A RescueBot resolves within a radius of `RADIO_STOP_DIST`. If two beacons were only `RADIO_STOP_DIST` apart, a robot could sit inside both resolution disks at once and not know which one it is looking at. Forcing centres `2 x RADIO_STOP_DIST` apart makes the two disks disjoint, so the nearest beacon by radio is always the unique one in range. That guarantee is what makes opportunistic resolve on sight safe.

The visible box floats at `MARKER_Z = 0.22` m, above the LiDAR's single horizontal scan layer at roughly 0.06 m. It looks like a ground beacon but is never scanned as a wall, so it leaves no phantom obstacle in the map. The camera still sees it because the camera looks slightly upward by the time the robot is close.

Colour is chosen at random per beacon from red, yellow, green. Colour is not carried in the radio payload; it is something the camera has to read.

### The auction loop

Two radio devices on the supervisor: `auction_rx` listens on channel 2, `auction_tx` broadcasts on channel 2. Each step the supervisor runs four stages.

Stage 0, collect messages: it drains the bus, recording `BID <beacon> <robot> <cost>` into a freshness stamped table and `RESOLVED <robot> <beacon>` into a per step set.

Stage 1, initial assignment: for each unassigned beacon that has been live for at least `AUCTION_WINDOW`, assign it to the free robot with the lowest fresh bid. A bid older than `BID_FRESHNESS` is ignored.

Stage 2, one reassignment or swap per step:
 1. If a cheaper free robot exists for a beacon, move the beacon to it.
 2. If the cheaper robot is busy, swap the two beacons only when the total cost of the pair strictly drops.
 3. An assignee whose remaining cost is below `COMMIT_COST` is committed and untouchable, so a robot is never yanked off a goal it is about to reach.

Stage 3, broadcast: send each robot its authoritative `TASK <robot> <beacon> <colour>` (or `TASK <robot> -1 none` when idle). The robot simply obeys the latest TASK.

Stage 4, rescue resolution (below).

### Rescue resolution: declaration first, ground truth as a safety net

A beacon is rescued when any RescueBot declares it resolved, which happens once that robot is in range and its camera has confirmed a colour. The supervisor honours the declaration directly:

 1. The resolver is the declaring robot if there is one for that beacon.
 2. If nobody declared it, the supervisor falls back to ground truth: the assignee being within `RESCUE_RADIUS` of the beacon. This exists only so the mission cannot stall if a declaration is ever missed.
 3. On resolution it records the latency, frees the assignee (which may be a different robot than the resolver), removes the beacon node, and broadcasts `RESCUED <resolver> <beacon>`.

Because the resolver and the assignee can differ, the robot side splits the two ideas cleanly: the assignee releases its goal, while the resolver gets the credit.

## RescueBot (`robot_controller.py`)

### Pose without GPS

Position is dead reckoned from the wheel encoders, heading from the compass. The map lives in the robot's local frame, with its start as the origin. World position is the local position plus the start vector that came in through `controllerArgs`. Since the compass gives a global heading, every robot's local frame shares the world axes, so maps merge by pure translation.

$$ \Delta s = \frac{(l_t - l_{t-1}) + (r_t - r_{t-1})}{2} \cdot r_w, \qquad \theta = \text{atan2}(c_x, c_y) $$

$$ x_t = x_{t-1} + \Delta s \cos\theta, \qquad y_t = y_{t-1} + \Delta s \sin\theta $$

### Occupancy grid that forgets

A 200 by 200 grid spans 4.0 by 3.0 m in each direction from the origin, so a cell is roughly 4 cm. Each cell holds two counters: `hits` (seen occupied) and `visits` (observed at all). Occupancy is `hits / visits`, and a cell counts as a wall above `WALL_CERTAINTY`.

The grid is adaptive. A LiDAR beam is traced with Bresenham from the robot to its endpoint. The endpoint cell gains occupied evidence; cells the beam passed through lose a unit of wall evidence and gain a visit. Counters saturate at `MAX_COUNT`, so a cleared cell recovers after a few sweeps. A removed beacon stops haunting the map, while a real wall reappears on the next scan. `clear_occupancy_disc()` does the same job deliberately when a beacon is rescued near the robot.

Walls are inflated by `OBSTACLE_INFLATE` cells into a separate grid, so the planner keeps the chassis clear of corners it cannot physically fit through.

### Bidding from the explored map

Every `REBID_PERIOD` the robot turns each beacon it hears into a goal estimate and bids on it.

The range comes from signal strength, the bearing from the emitter direction, and the world goal from the heading:

$$ \hat r = \frac{1}{\sqrt{\text{strength}}}, \qquad \text{goal} = \text{pos} + \hat r \cdot (\cos(\theta + \beta), \sin(\theta + \beta)) $$

The bid is an honest travel cost, not a straight line. `known_flood()` runs Dijkstra over only the cells the robot has actually explored, then the bid is the cheapest known cell plus the unexplored remainder charged at a penalty:

$$ \text{bid} = \min_{\text{known cell } c}\big( \text{cost}(c) + \text{BID\_UNKNOWN\_FACTOR} \cdot \lVert c - \text{goal} \rVert \big) $$

A robot with a clear mapped route bids lower than one that would have to push into the unknown, even if the second is closer in a straight line.

### Pathfinding: bidirectional Dijkstra, gradient, line of sight

Once assigned, the robot keeps two versions of the goal:
 1. The live goal, refreshed every ping, used for direct steering.
 2. The gated goal, which only moves when the estimate shifts by more than `GOAL_MOVE_THRESH`, used to trigger a replan. This decouples cheap re-aiming from expensive replanning.

`bidirectional_dijkstra()` searches from the robot and from the goal at the same time and stops when the two frontiers meet, which explores roughly half the area of a one sided flood. Edge weight uses a cost of destination model: a straight step costs 1, a diagonal 1.5, each scaled up by the destination cell occupancy, with inflated and peer occupied cells excluded. The resulting path is written into a cost field as a descending gradient toward the goal.

Steering then takes the smoothest legal target. If the live goal is in direct line of sight, the robot aims straight at it. Otherwise `get_lookahead_target()` walks down the cost gradient as far as line of sight allows and aims there. Either way `steer_to()` is one proportional controller: heading error sets the turn rate, and forward speed scales down as the error grows, so the robot slows to pivot for sharp turns and runs near cruise when it is pointed right.

$$ \omega = K \cdot \text{error}, \qquad \text{forward} = \text{CRUISE} \cdot \max\!\left(0, 1 - \frac{2 |\text{error}|}{\pi}\right) $$

A reactive override keyed on `SAFE_DISTANCE` exists for the case where the planned step would graze a wall, but on open ground the gradient plus line of sight path carries the motion.

A replan is triggered when a cell flips state (a newly seen wall), when the robot drifts off the path, or when the gated goal moves.

### Camera, resolution, and opportunism

The camera scans for all three colours at once. `detect_any_colour()` counts pixels in each colour band and returns the dominant colour if its count clears `TARGET_PIXEL_MIN`. The threshold is an absolute pixel count, not a percentage. Scanning is enabled within `CAMERA_DIST`.

Resolution is opportunistic and runs every step, independent of assignment:
 1. Pick the closest beacon the robot hears (the separation rule makes this unambiguous in range).
 2. Require its range below `RADIO_STOP_DIST`.
 3. Require it inside the camera frame, `|\beta| \le \text{CAM\_FOV}/2`, so the colour the camera reads belongs to that beacon.
 4. Require the camera to confirm a colour.

When all four hold, the robot records the ping estimate, its own stop point, and the camera colour, marks the beacon so it is never recorded twice, and sends `RESOLVED <id> <beacon>`. A robot therefore resolves any beacon it passes, including ones meant for a peer or not yet assigned to anyone, so a confirmed find is never wasted.

### Peer avoidance

Robots heartbeat their position and motion state on the bus. In a close encounter only one moves: the lower id keeps going, the other yields and freezes. The mover stamps the frozen peer into a transient layer of the grid so the planner routes around it. Hysteresis with `ROBOT_AVOID_DIST` and `ROBOT_RELEASE_DIST` stops the two from oscillating.

### One global map, continuously merged

There is one map for the team, not one per robot. Each robot pickles its live state to `slam_state_<id>.pkl` (occupancy grid, world start, trajectory, ping estimates, stop points, camera colours) about twice a second and on every resolve. Any robot can load every pickle and merge the grids into one global grid by translating each by its start vector, recentred on the world origin `(0, 0)`.

Robot 0 re renders the live global map on a timer, and any robot re renders on a resolve, so the map always reflects every robot's detections. Two galleries are produced:

| Output | Contents |
|---|---|
| `maps_traj/global_map.png` | merged obstacles, every trajectory, starts, stop points, dashed links, camera coloured pings |
| `clean_maps/global_clean_map.png` | merged obstacles and camera coloured pings only, no robots, starts, or trajectories |

Plus a per resolve snapshot, `global_map_stage<N>.png` and `global_clean_map_stage<N>.png`, for the record.

Colour rules on the map:
 1. A beacon star is always the colour the camera identified, so its colour on the map is its real colour.
 2. A trajectory, start, stop circle, and dashed link use a fixed per robot colour (purple, orange, brown, teal, and so on), never a beacon colour, so a path is never confused with a goal and a given robot keeps its colour across the whole run.

The legend sits in a column to the right of the axes in both galleries, so it never covers a ping. `render_global_map.py` regenerates either gallery offline from the pickles, so the figures can be remade without rerunning the simulation.

## Parameters

### Ping resolution

| Parameter | Value | Meaning |
|---|---|---|
| `RADIO_STOP_DIST` | 0.80 m | resolve once the estimated range is under this and colour is confirmed |
| `CAMERA_DIST` | 1.15 m | camera starts scanning within this estimated range |
| front gate | `CAM_FOV / 2` | beacon must be in frame before its colour is trusted |
| `RESCUE_RADIUS` | 0.40 m | supervisor ground truth fallback |
| `MIN_BEACON_SEP` | 1.6 m | spawn separation, so only one ping is ever in resolution range |

### Colour assessment

| Parameter | Value | Meaning |
|---|---|---|
| `TARGET_PIXEL_MIN` | 20 pixels | minimum matching pixels to confirm a colour (count, not percentage) |
| red band | R [200, 255], G [0, 60], B [0, 60] | camera RGB thresholds |
| yellow band | R [200, 255], G [200, 255], B [0, 80] | camera RGB thresholds |
| green band | R [0, 80], G [200, 255], B [0, 80] | camera RGB thresholds |

### Ping generation

| Parameter | Value | Meaning |
|---|---|---|
| first spawn | t = 1.0 s | fixed start |
| inter spawn gap | `gauss(BEACON_INTERVAL, BEACON_STD)` | Gaussian, mean and median equal `BEACON_INTERVAL` |
| `BEACON_INTERVAL` | 5.0 s default, swept 5 to 30 | median gap |
| `BEACON_STD` | 1.5 default, `0.3 x interval` in the sweep | spread of the gap |
| `BEACON_MIN_GAP` | 0.5 s | clamp so a draw is never non positive |
| `NUM_GOALS` | 0 | 0 means spawn forever |

### Bidding and auction

| Parameter | Value | Meaning |
|---|---|---|
| `REBID_PERIOD` | 1.0 s | each robot re bids on every heard beacon this often |
| `AUCTION_WINDOW` | 2.0 s | manager waits this long before the first assignment |
| `BID_FRESHNESS` | 3.0 s | manager ignores bids older than this |
| `BID_UNKNOWN_FACTOR` | 1.5 | cost multiplier on the unexplored remainder of a bid |
| `COMMIT_COST` | 15.0 | an assignee below this remaining cost is committed, never reassigned |

### Motion and grid

| Parameter | Value | Meaning |
|---|---|---|
| `TIME_STEP` | 32 ms | control loop, about 31 Hz |
| `CRUISE_SPEED`, `MAX_SPEED` | 3.0, 6.28 rad/s | wheel speeds |
| `MAP_SIZE`, `WORLD_X_MAX`, `WORLD_Y_MAX` | 200, 4.0, 3.0 | grid roughly 4 cm per cell |
| `WALL_CERTAINTY`, `IMPASSABLE` | 0.30, 0.90 | occupancy thresholds |
| `MAX_COUNT` | 10 | evidence cap, lets the grid forget removed obstacles |
| `OBSTACLE_INFLATE` | 4 cells | safety buffer around walls |
| `GOAL_MOVE_THRESH` | 0.20 m | replan when the goal estimate shifts more than this |
| `ROBOT_AVOID_DIST`, `ROBOT_RELEASE_DIST` | 0.35, 0.55 m | peer stop and resume, with hysteresis |
| `SILENCE_TIMEOUT` | 8.0 s | idle with no pings this long ends the mission |

## Benchmark methodology

The benchmark answers one question: how many concurrent victims can a team of size N clear, on a map of a given difficulty, as the arrival rate rises.

The supervisor reads its configuration from environment variables, runs for a fixed number of sim seconds, appends one row of aggregate metrics to a CSV, then quits cleanly so the next run can start. A short grace window re broadcasts `DONE` first, so every scout process saves its map and exits rather than being orphaned.

| Axis | Values | Set via |
|---|---|---|
| map | easy, medium, hard | `RESCUE_MAP` |
| robots | 1, 2, 3, 4, 5 | `RESCUE_ROBOTS` |
| interval | 5, 10, 15, 20, 25, 30 s (median) | `RESCUE_INTERVAL` |
| seed | 0, 1, 2 | `RESCUE_SEED` |
| gap std | `0.3 x interval` | `RESCUE_STD` |
| run length | 600 sim seconds | `RESCUE_RUN_SECONDS` |

`run_sweep.py` launches one headless Webots process per configuration (`--batch --mode=fast --no-rendering`). Runs are independent, so they can run concurrently with `--jobs N`, each writing its own CSV part that is merged at the end. The full grid is 3 maps times 5 team sizes times 6 intervals times 3 seeds, which is 270 runs.

Each `(config, seed)` is fully reproducible: the seed governs beacon positions, colours, and the Gaussian gaps.

### Metrics per run

| Metric | Definition |
|---|---|
| `spawned`, `rescued` | counts over the run |
| `throughput_min` | rescued per minute, `rescued / run_seconds x 60` |
| `mean_latency`, `median_latency`, `p95_latency` | spawn to rescue delay, in seconds |
| `mean_backlog` | active (unrescued) beacon count, sampled at 1 Hz |
| `final_backlog` | unrescued beacons at the end |

`plot_results.py` averages each metric over the three seeds (error bars are the standard deviation across seeds) and draws one panel per map, one line per team size, against the spawn interval. It also prints a saturation summary: the largest interval each team keeps stable, where stable means mean backlog below 2 and mean latency below 30 s.

## Benchmark results

Three figures, in `benchmark/figures/`. Read them together, because one of them is easy to misread on its own.

### Throughput, the amount of work cleared

Throughput falls as the interval grows, because at long intervals there is simply less to rescue. At long intervals the system is supply limited and all team sizes collapse onto the same low line. At short intervals it is demand limited and team size matters a lot. On easy at a 5 s interval, 5 robots reach roughly 10 to 11 rescues per minute against roughly 1.2 for a single robot, close to an eight to nine times gain, and they rescue almost everything that spawns (about 101 to 113 of about 111 to 118). Map difficulty cuts capacity: peak throughput of about 11 per minute on easy and medium drops to about 5 to 7 per minute on hard.

### Latency, the wait per victim, read with care

The clean trend is that more robots means much lower latency, from roughly 150 to 250 s for a single robot on easy and medium down to roughly 20 to 60 s for five.

The caveat matters. Latency is averaged only over beacons that were actually rescued. At long intervals, small teams, or hard maps, many runs rescue only the first beacon that spawned near the cluster and then little else. For example medium, 1 robot, 15 s interval, seed 0 spawned 13 and rescued 1, giving a latency near 4.9 s that looks excellent but means the team was drowning. So the low latency points on the right of the panels are often survivorship bias, not good performance. Latency should be presented next to throughput and backlog, never alone.

### Backlog, how far behind the team falls, the honest indicator

Backlog is high at short intervals (beacons pile up faster than they clear, roughly 12 to 14 for small teams) and decreases as inflow drops. More robots means consistently lower backlog (five robots bottom out around 2 to 5, one robot stays around 7 to 14). For the degenerate runs above, backlog stays near 10 to 12 even though latency looked tiny, which is exactly why the three metrics must be read together.

### Headline

| Finding | Evidence |
|---|---|
| The team scales well under load | throughput at a 5 s interval grows roughly linearly from 1 to 3 robots, then with diminishing returns |
| At low load team size is irrelevant | curves converge at intervals of 20 s and above |
| Difficulty ranks easy, medium, hard | peak throughput about 11, 11, and 5 to 7 per minute |
| Latency alone is misleading | low latency at high interval coincides with high backlog and near zero throughput |

## Limitations

 1. Resolution depends entirely on the camera now, so a colour band miss leaves only the ground truth fallback, which fires later and without a recorded colour.
 2. Opportunistic resolution bypasses the auction's cost optimisation, so a robot can grab a beacon a peer was already committed to and waste that peer's travel. The commit threshold limits this but does not remove it.
 3. The separation guarantee leaks exactly when the arena is most crowded, because a spawn that cannot find a legal spot is deferred rather than placed; under heavy backlog new beacons can stall.
 4. Latency is conditional on rescue, so degenerate runs (only the first beacon rescued) skew the metric. A filter that drops cells with too few rescues would make the figures cleaner.
 5. Bearing from `getEmitterDirection()` is still the exact engine vector, an approximation of a real antenna sweep, and the original emission strength is assumed known.
 6. The live global render is done by robot 0, so live updates stop if robot 0 exits first, though the per resolve snapshots still cover the record.
