# Iteration 2 — multi-victim triage with honest sensing

This builds directly on iteration 1 (see the parent `project_evolution.md`).
Iteration 1 was a single beacon that broadcast its own coordinates, and a scout
that drove to those coordinates. That works, but it cheats: a radio doesn't hand
you map coordinates. Iteration 2 keeps the same world skeleton and the same
reactive/LiDAR rationale, but fixes the cheat and adds triage.

## Goals (and where each lives in the code)

| # | Goal | Where |
|---|------|-------|
| 1 | Keep iteration-1 structure & rationale | same `worlds/` + `controllers/` layout, same E-puck/LiDAR/Display devices |
| 2 | Remove positional cheating | beacon sends id only; scout homes on signal strength + bearing |
| 3 | SLAM with confidence levels | Bayesian log-odds occupancy grid |
| 4 | More pingers | four beacons in `arena.wbt`, all on channel 1 |
| 5 | Closest signal first | among un-attended beacons, pick strongest signal |
| 6 | Assess gravity | random green/yellow/red on arrival (placeholder triage) |
| 7 | Printed SLAM map | console map: walls/free/unknown + trajectory + colour-coded victims |

## (2) Removing the positional cheat

**Iteration 1:** `pinger` sent `"SOS <x> <y>"`; `scout` parsed those numbers and
drove straight to them. The beacon literally told the robot where it was.

**Iteration 2:** `pinger` sends only `"SOS <name>"` — an identity, nothing else.
It no longer even has a GPS. The scout recovers a victim's location from the two
things a real radio actually exposes in Webots:

- **Signal strength** `getSignalStrength()` ≈ `1 / r²`, so estimated range is
  `r = 1 / sqrt(strength)`.
- **Bearing** `getEmitterDirection()` — a unit vector *in the robot's own frame*;
  the bearing is `atan2(dir.y, dir.x)`.

Homing is then "turn toward the bearing, drive forward"; arrival is "estimated
range < `GOAL_TOL`". No world coordinates ever enter the navigation path.

> The scout still reads its **own** `gps`/`compass` to place LiDAR returns on the
> map and to record its trajectory. That's an onboard localisation sensor, not
> knowledge of the victim — so it isn't the cheat we set out to remove. A later
> iteration could replace it with a pose estimated from odometry + scan-matching.

## (3) SLAM with confidence

The occupancy grid is now a **Bayesian log-odds** map (`logodds[y][x]`):

- start at `0.0` → probability `0.5` → **unknown**;
- a beam that *passes through* a cell adds `L_FREE` (negative) → evidence of free;
- the cell a beam *ends on* adds `L_OCC` (positive) → evidence of a wall;
- values are clamped to ±`L_CLAMP` so the map stays correctable.

Probability is `p = 1/(1+e^-logodds)` and **confidence is `|p − 0.5|`** — the
magnitude of the log-odds. A cell is only treated as a wall once it passes
`WALL_LOGODDS`, so one stray beam can't hallucinate a wall; it takes repeated,
agreeing observations to build confidence. Both the graphical and printed maps
shade by this confidence (low-confidence cells render as `:`).

## (4) More pingers

`arena.wbt` now has `pinger_1..4` spread around the arena, each a red cube with
an `Emitter` on channel 1 and the shared `pinger_controller`. Each derives its id
from `getName()`, so the controller file is identical for all of them.

## (5) Closest-first scheduling

Every step the scout keeps the latest `(strength, bearing)` per beacon it has
heard. Among beacons **not yet attended**, it targets the one with the strongest
signal — i.e. the nearest victim. When that one is reached it drops out of the
pool and the next-nearest becomes the target. The mission ends when every beacon
the scout has ever heard has been attended.

## (6) Gravity / triage

On arrival the scout assigns `random.choice(green/yellow/red)`. This is a stand-in
for a real assessment (camera/vitals); the plumbing — recording the urgency,
colouring the maps, and the triage totals — is what matters here.

## (7) Printed SLAM map

When the mission completes, `print_slam_map()` prints an ASCII rendering of the
log-odds grid: `#` walls, `.` confident-free, `:` seen-but-unsure, blank unknown.
The full trajectory is overlaid as blue `+`, and each victim is drawn as its
urgency initial (`R`/`Y`/`G`) in the matching ANSI colour, followed by a triage
summary. The same information is drawn live on the in-sim `map_display`.

## How to run

1. Open `worlds/arena.wbt` in Webots.
2. Press play. The scout spirals/searches, homes on the nearest beacon by signal,
   triages it, then moves to the next-nearest.
3. When all beacons are attended, the console prints the SLAM map + triage report.
