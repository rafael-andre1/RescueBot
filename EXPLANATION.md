# RescueBot — Controllers explained

This is a plain-English walkthrough of how the two controllers in this
project work, based on the inline notes inside `robot_controller.py` and
`signal_controller.py`.

---

## Scout (`controllers/robot_controller/robot_controller.py`)

### Overview

The scout is a distress-ping rescue robot (radio-homing variant).

- **No GPS.** The pose is dead-reckoned from the wheel encoders, with the
  compass providing absolute heading.
- **The goal is unknown.** Every HELP ping gives a robot-relative bearing
  plus a signal strength. Strength → estimated range (`~1/sqrt(s)`);
  bearing + heading → world direction. Together they project an estimated
  goal in the dead-reckoned frame.
- **Path-finding** (Dijkstra over the live occupancy grid) routes around
  walls discovered en route. There is **no** reactive obstacle avoidance
  and **no** spiral search — the planner is the only thing that decides
  where to go, because the cost field always reflects what we've mapped
  so far.

**Lifecycle:** sit still and scan until the first ping arrives → follow
the cost gradient toward the estimated goal → replan when (a) a cell
flips state, or (b) the bearing-derived goal moves more than
`GOAL_MOVE_THRESH`. Arrival is declared when signal strength exceeds
`S_ARRIVE`. The occupancy map is saved as `slam_map.png` on arrival or
when the signal goes silent for `SILENCE_TIMEOUT` seconds.

### Matrix 1 — The occupancy grid

The occupancy grid is where we store where the walls are. The map is
divided into a 200×200 matrix, and each cell is either free or occupied.
Since the LiDAR isn't perfect we don't use a hard boolean — instead we
track an **occupancy certainty** that says *how sure we are* that the
cell contains a wall. The certainty is filled in as the robot moves
towards the goal: every LiDAR beam either bumps the cell's hit counter
(it landed on a wall there) or just bumps its visit counter (the ray
passed through it freely). Occupancy probability = `hits / visits`.

### Matrix 2 — The cost matrix

We also have a cost matrix that originates from the goal. But since we
don't actually know where the goal is, we use the radio signal strength
to roughly estimate the distance and compute the cost field from that
estimate.

- Edge weight is **1 for straight moves and 1.5 for diagonals**.
- The robot looks at the 15 cells around it (plus its own cell) and
  moves toward the cell with the **smallest** cost value. On ties it
  picks the one that requires the **least heading change**, so a
  straight corridor produces straight-line motion instead of zig-zagging
  every time the planner picks a different tied neighbour.

Every time a previously-unknown wall is discovered, we recompute the
cost of that cell. The cost of routing through it shoots up, which
automatically makes the gradient push the robot to re-route around the
new wall. We also pad the walls with an inflation buffer (cells close
to walls are marked as likely walls) so the robot doesn't graze them
just because the raw LiDAR returns are imperfect.

### Full navigation logic

The full loop:

1. **Wait for HELP.** We sit still until the first radio ping arrives.
   If multiple beacons broadcast at once we go toward the one with the
   strongest signal (which means the closest).
2. **Estimate distance and bearing.** We compute an approximate goal
   position from the signal's strength and direction. We only update
   that estimate when it shifts significantly, so radio noise doesn't
   re-trigger replans every tick.
3. **Compute** the cost matrix (over the live occupancy matrix), then
   **move**: the robot picks the cheapest of the 8 cells adjacent to its
   current cell (with the heading tiebreak) and steers toward it.
4. When the robot is close enough to the source — measured by signal
   strength — it has **arrived**. The scout then resets to wait-for-ping
   mode and the cycle repeats for the next help request.

---

## Signal manager (`controllers/signal_controller/signal_controller.py`)

The signal manager is a Webots **Supervisor** that runs *one* distress
beacon through `NUM_BEACONS` randomly placed positions on the map.

### Behaviour

- **At startup** it picks `NUM_BEACONS` random world positions (within
  the scout's map bounds) and prints the list.
- It teleports itself to position #1 and starts emitting `"SOS"` on
  channel 1 **every step** — that frequent cadence keeps the scout's
  radio bearing fresh instead of stale.
- **It does not move on its own timer.** A beacon never disappears
  until the scout has physically reached it (within `RESCUE_RADIUS`).
- When the scout *is* within range, the manager prints
  `beacon #N RESCUED`, advances to position #N+1, and teleports there.
  Broadcasting continues from the new position.
- After all `NUM_BEACONS` have been rescued, the manager stops and the
  controller exits.

Because there's only one physical emitter and it teleports between
positions, the scout always sees a single source at a time — its
existing "go toward the strongest signal" code naturally handles the
hand-off when the beacon jumps to a new location.
