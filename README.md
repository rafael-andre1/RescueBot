# RescueBot

A GPS-denied multi-robot search-and-rescue simulation in Webots. A team of
e-puck scouts has to find and reach distress beacons that appear over time at
unknown positions. There is no GPS and no map handed to the robots: each scout
localizes by wheel odometry plus a compass, maps the arena into its own
occupancy grid, homes in on a beacon from radio signal strength and bearing
alone, and confirms the beacon's colour with its camera. A central supervisor
runs a continuous auction that hands each beacon to whichever scout can reach it
cheapest, and reassigns on the fly as the maps fill in. The scouts' grids are
fused into one shared world map.

## Requirements

- **Webots R2025a** (the world files declare `#VRML_SIM R2025a`).
- A Python interpreter with **numpy** and **matplotlib** installed, and Webots
  pointed at it (Webots preferences, "Python command"). The map images are
  rendered with matplotlib; if it is missing, the controllers still run and just
  print a `pip install matplotlib numpy` reminder instead of saving maps.
  The benchmark scripts also use Pillow.

## Running it

If Webots is already set up, that is the whole procedure:

1. Open `worlds/arena.wbt` in Webots.
2. Press play.

The world file itself only contains the floor, the walls, and an invisible
supervisor node. Everything else, the scouts and the beacons, is spawned at
runtime by the supervisor, so there is nothing to wire up by hand. On a default
run you get five scouts clustered at the centre and a new beacon roughly every
five seconds (the gap is Gaussian), and it runs until you stop it.

Watch the console for the mission log: beacon spawns, auction assignments and
reassignments, camera confirmations, and rescues.

To see the camera and the live SLAM display per scout, turn on the robot
overlays: Webots menu, View, Optional Rendering, and the scout overlays.

### Where the output goes

Maps are written under `controllers/robot_controller/`, since that is the
working directory the scout controllers share:

- `maps_traj/global_map.png` is the live global map: merged walls, every
  scout's path, start and stop points, and beacons coloured by what the camera
  read. It is refreshed every few seconds and after every rescue.
- `clean_maps/global_clean_map.png` is the same map stripped down to walls and
  beacons, no robots or paths.
- `maps_traj/global_map_stage<N>.png` and the matching clean files are
  per-rescue snapshots kept as a record.
- `slam_state_<id>.pkl` is each scout's saved state, the hand-off the others
  read to build the shared map.
- `slam_map_<id>.png` is a per-scout map written when a scout exits.

### Rebuilding the maps without a new run

If you just want the figures again from the last run's saved state:

```
python controllers/robot_controller/render_global_map.py
```

It reads the `slam_state_*.pkl` files and rewrites both global maps. Pass
`--clean-only` or `--traj-only` to limit it to one gallery.

## Current state

The current system is the **continuous-auction multi-robot rescuer**, the world
`arena.wbt` and the `signal_controller` / `robot_controller` / `beacon_controller`
trio. It works end to end: beacons spawn over time, the auction allocates and
reassigns them, scouts navigate around discovered walls with a bidirectional
Dijkstra planner, resolve a beacon once they are within range and the camera has
confirmed its colour (including beacons they pass by that were not theirs), and
the team's maps fuse into one global map by translation only.

It has been benchmarked across map difficulty, team size and goal spawn rate
(see `benchmark/`). Throughput scales close to linearly with team size on the
open and medium maps and saturates in the dense maze. There is a known
congestion failure mode at high robot density, where scouts packed at the spawn
ring can deadlock on the yield rule; this is analyzed in the write-up and in the
benchmark discussion.

Earlier iterations are kept alongside as separate, self-contained controllers
and worlds (see "Legacy" below). They are not used by `arena.wbt`.

## Files

### Active system (used by `arena.wbt`)

| File | Role |
|---|---|
| `worlds/arena.wbt` | The world. Open this to run. Holds the floor, walls, and the supervisor; scouts and beacons are spawned at runtime. |
| `controllers/signal_controller/signal_controller.py` | The supervisor: spawns the scouts and the beacons, runs the auction, declares rescues, and (in benchmark mode) logs metrics. |
| `controllers/signal_controller/bench_maps.py` | Wall layouts for `easy`, `medium`, `hard`, spawned by the supervisor. |
| `controllers/robot_controller/robot_controller.py` | The scout. One process per scout, all sharing this file: odometry, occupancy mapping, bidding, planning, camera, and resolution. |
| `controllers/robot_controller/render_global_map.py` | Rebuilds the global map images offline from the saved `slam_state_*.pkl`. |
| `controllers/beacon_controller/beacon_controller.py` | A distress beacon. Broadcasts `SOS <id>` every step on channel 1. |

### Benchmark

| File | Role |
|---|---|
| `benchmark/run_sweep.py` | Launches one headless Webots run per configuration and merges the results. |
| `benchmark/plot_results.py` | Turns `results.csv` into the throughput, latency and backlog figures. |
| `benchmark/results.csv` | The metrics, one row per run. |
| `benchmark/figures/` | The generated figures. |
| `benchmark/README.md` | How to run the sweep. |

### Documentation

| File | Role |
|---|---|
| `final_iteration_walkthrough.md` | The current system in depth, plus the benchmark methodology and results. |
| `rescue_swarm_explained.md` | The iteration-5 swarm. |
| `project_evolution.md` | The history from iteration 1 to 5. |

### Legacy (earlier iterations, not used by `arena.wbt`)

| File | Role |
|---|---|
| `worlds/arena_swarm.wbt`, `controllers/robot_controller_swarm/`, `controllers/signal_controller_swarm/` | Iteration 5: three colour-fixed scouts, one beacon each, maps merged on arrival. |
| `controllers/robot_controller_radio/`, `controllers/signal_controller_radio/` | Iteration 3: a single scout homing on one radio beacon. |
| `worlds/robotics.wbt` | Earlier scratch world. |

## Configuration

A normal run needs no configuration. The benchmark reads everything from
environment variables so one world file can serve every configuration:

| Variable | Default | Meaning |
|---|---|---|
| `RESCUE_MAP` | `medium` | `easy`, `medium`, or `hard` |
| `RESCUE_ROBOTS` | `5` | number of scouts |
| `RESCUE_INTERVAL` | `5.0` | median seconds between beacon spawns (gap is Gaussian) |
| `RESCUE_STD` | `1.5` | standard deviation of the spawn gap |
| `RESCUE_SEED` | `42` | random seed (positions, colours, gaps) |
| `RESCUE_RUN_SECONDS` | `0` | run length; `0` means run forever (interactive) |
| `RESCUE_RESULT_CSV` | empty | path to append one metrics row; empty means do not log |

See `benchmark/README.md` for the full sweep.

## Known caveats

- **`arena.wbt` carries the seven `medium` walls statically**, while the
  supervisor also spawns walls from `bench_maps` for the chosen `RESCUE_MAP`.
  For the default `medium` run they sit in the same places, so the interactive
  experience is correct. But a sweep that sets `RESCUE_MAP=easy` or `hard` still
  has those static medium walls present, so those maps are contaminated. Remove
  the static walls from `arena.wbt` (leaving only the floor and the supervisor)
  before trusting the easy and hard benchmark rows.
- **This is mapping, not full SLAM.** Pose comes from odometry and the compass
  and is never corrected from the map (no loop closure or scan matching). The
  shared compass is what lets the grids merge by translation alone; on real
  hardware with heading drift this would need true rotational alignment.
- **Congestion at high density.** With three or more scouts packed at the spawn
  ring, the fixed-priority yield rule can deadlock and stall a run. It strikes a
  minority of seeds and inflates the variance rather than the peak numbers.
- **Simulation only**, with an idealized radio model (clean range and bearing).


**Small Remark**: code documentation (structured comments) is refactored using LLMs to ensure no piece of code is left unexplained. Additionally, code (including functions) was also optimized using AI for simplicity, efficiency and comprehension. Nonetheless, the code remains of our authorship, and is ALWAYS manually checked whenever any of the afforementioned changes happen. 