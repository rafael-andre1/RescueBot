#!/usr/bin/env python3
"""
run_sweep.py — batch benchmark runner for RescueBot.

Sweeps (map × robots × spawn-interval × seed), launching Webots headless once
per config. The signal_controller reads the config from RESCUE_* env vars,
runs for RUN_SECONDS sim-seconds, appends one metrics row to the shared CSV,
then calls simulationQuit so this script can move to the next config.

Usage:
    python3 benchmark/run_sweep.py                 # full sweep
    python3 benchmark/run_sweep.py --quick         # tiny smoke sweep
    python3 benchmark/run_sweep.py --maps easy hard --robots 1 3 5

Output: benchmark/results.csv  (one row per run). Then:
    python3 benchmark/plot_results.py benchmark/results.csv

Requires Webots on PATH (or set WEBOTS_BIN). The controllers must use the
Python that has numpy/matplotlib/Pillow (set in Webots' Python command).
"""
import os
import sys
import time
import argparse
import subprocess

HERE      = os.path.dirname(os.path.abspath(__file__))
REPO      = os.path.dirname(HERE)
WORLD     = os.path.join(REPO, "worlds", "arena.wbt")
WEBOTS    = os.environ.get("WEBOTS_BIN", "webots")
RESULTS   = os.path.join(HERE, "results.csv")

# ── Sweep definition (edit here, or override on the CLI) ─────────────
MAPS      = ["easy", "medium", "hard"]
ROBOTS    = [1, 2, 3, 4, 5]
INTERVALS = [5, 10, 15, 20, 25, 30]      # MEDIAN seconds between goals
SEEDS     = [0, 1, 2]                     # repetitions per cell
RUN_SECONDS = 300.0                       # sim-seconds measured per run
STD_FRAC  = 0.3                           # Gaussian gap std = STD_FRAC * interval


def run_one(map_name, robots, interval, seed):
    env = dict(os.environ)
    env.update({
        "RESCUE_MAP":         map_name,
        "RESCUE_ROBOTS":      str(robots),
        "RESCUE_INTERVAL":    str(interval),
        "RESCUE_STD":         str(round(STD_FRAC * interval, 3)),
        "RESCUE_SEED":        str(seed),
        "RESCUE_RUN_SECONDS": str(RUN_SECONDS),
        "RESCUE_RESULT_CSV":  RESULTS,
    })
    cmd = [WEBOTS, "--batch", "--mode=fast", "--no-rendering",
           "--minimize", "--stdout", "--stderr", WORLD]
    # Generous wall-clock cap: fast mode is usually << real-time, but 5 robots
    # doing Dijkstra can be heavy. Kill if it overruns badly.
    timeout = max(120, RUN_SECONDS * 2)
    t0 = time.time()
    try:
        subprocess.run(cmd, env=env, timeout=timeout,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ok = "ok"
    except subprocess.TimeoutExpired:
        ok = "TIMEOUT"
    print("  [%s] map=%s robots=%d interval=%ds seed=%d  (%.0fs wall)"
          % (ok, map_name, robots, interval, seed, time.time() - t0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true",
                    help="tiny sweep to smoke-test the pipeline")
    ap.add_argument("--maps", nargs="+", default=None)
    ap.add_argument("--robots", nargs="+", type=int, default=None)
    ap.add_argument("--intervals", nargs="+", type=int, default=None)
    ap.add_argument("--seeds", nargs="+", type=int, default=None)
    ap.add_argument("--fresh", action="store_true",
                    help="delete an existing results.csv before starting")
    args = ap.parse_args()

    maps      = args.maps      or MAPS
    robots    = args.robots    or ROBOTS
    intervals = args.intervals or INTERVALS
    seeds     = args.seeds     or SEEDS
    if args.quick:
        maps, robots, intervals, seeds = ["easy"], [1, 3], [5, 30], [0]

    if args.fresh and os.path.exists(RESULTS):
        os.remove(RESULTS)

    total = len(maps) * len(robots) * len(intervals) * len(seeds)
    print("Sweep: %d runs → %s" % (total, RESULTS))
    print("  maps=%s robots=%s intervals=%s seeds=%s run=%.0fs"
          % (maps, robots, intervals, seeds, RUN_SECONDS))
    if not os.path.exists(WORLD):
        sys.exit("World not found: %s" % WORLD)

    n = 0
    for m in maps:
        for r in robots:
            for it in intervals:
                for s in seeds:
                    n += 1
                    print("[%d/%d]" % (n, total), end=" ")
                    run_one(m, r, it, s)
    print("Done. Rows in %s" % RESULTS)


if __name__ == "__main__":
    main()
