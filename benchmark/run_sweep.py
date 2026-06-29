#!/usr/bin/env python3
"""
Batch runner for the RescueBot benchmark.

Walks every (map, robots, spawn-interval, seed) combo and fires up a headless
Webots for each one. signal_controller picks the config out of the RESCUE_* env
vars, runs for RUN_SECONDS, writes one row of metrics to a CSV, then quits itself
so we can launch the next.

Usage:
    python3 benchmark/run_sweep.py                 # full sweep (sequential)
    python3 benchmark/run_sweep.py --jobs 16       # 16 runs at once
    python3 benchmark/run_sweep.py --quick         # tiny smoke sweep
    python3 benchmark/run_sweep.py --maps easy hard --robots 1 3 5

Output: benchmark/results.csv  (one row per run). Then:
    python3 benchmark/plot_results.py benchmark/results.csv

Requires Webots on PATH (or set WEBOTS_BIN). The controllers must use the
Python that has numpy/matplotlib/Pillow (set in Webots' Python command).

Parallelism: each config is an independent headless Webots launch, so they can
run concurrently (use --jobs N). Each run writes its own CSV part under
benchmark/_parts/; the parts are merged into results.csv at the end. A 14900K
(32 threads) handles ~16 jobs comfortably; each run uses ~1-2 cores.
"""
import os
import sys
import csv
import glob
import time
import argparse
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE      = os.path.dirname(os.path.abspath(__file__))
REPO      = os.path.dirname(HERE)
WORLD     = os.path.join(REPO, "worlds", "arena.wbt")
WEBOTS    = os.environ.get("WEBOTS_BIN", "webots")
RESULTS   = os.path.join(HERE, "results.csv")
PARTS_DIR = os.path.join(HERE, "_parts")

# ── Sweep definition (edit here, or override on the CLI) ─────────────
MAPS      = ["easy", "medium", "hard"]
ROBOTS    = [1, 2, 3, 4, 5]
INTERVALS = [5, 10, 15, 20, 25, 30]      # MEDIAN seconds between goals
SEEDS     = [0, 1, 2]                     # repetitions per cell
RUN_SECONDS = 600.0                       # sim-seconds measured per run (10 min)
STD_FRAC  = 0.3                           # Gaussian gap std = STD_FRAC * interval


def run_one(idx, total, map_name, robots, interval, seed):
    """Launch one headless Webots run. Writes its metrics row to a unique
    CSV part so concurrent runs never touch the same file."""
    part_csv = os.path.join(PARTS_DIR, "run_%05d.csv" % idx)
    env = dict(os.environ)
    env.update({
        "RESCUE_MAP":         map_name,
        "RESCUE_ROBOTS":      str(robots),
        "RESCUE_INTERVAL":    str(interval),
        "RESCUE_STD":         str(round(STD_FRAC * interval, 3)),
        "RESCUE_SEED":        str(seed),
        "RESCUE_RUN_SECONDS": str(RUN_SECONDS),
        "RESCUE_RESULT_CSV":  part_csv,
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
    except subprocess.TimeoutExpired:
        print("[%d/%d] [TIMEOUT] map=%s robots=%d interval=%ds seed=%d  (%.0fs wall)"
              % (idx, total, map_name, robots, interval, seed, time.time() - t0),
              flush=True)
        return part_csv
    # webots.exe is a launcher that can return BEFORE its simulation finishes
    # (it hands the world to a detached app instance). Hold this worker's slot
    # until the metrics row actually lands, so concurrency stays capped at
    # --jobs instead of firing every run at once.
    deadline = t0 + timeout
    while not os.path.exists(part_csv) and time.time() < deadline:
        time.sleep(1.0)
    ok = "ok" if os.path.exists(part_csv) else "NODATA"
    print("[%d/%d] [%s] map=%s robots=%d interval=%ds seed=%d  (%.0fs wall)"
          % (idx, total, ok, map_name, robots, interval, seed, time.time() - t0),
          flush=True)
    return part_csv


def merge_parts(part_files):
    """Concatenate per-run CSV parts into RESULTS (header written once)."""
    rows, header = [], None
    for pf in sorted(part_files):
        if not (pf and os.path.exists(pf)):
            continue
        with open(pf, newline="") as f:
            r = list(csv.reader(f))
        if not r:
            continue
        header = r[0]
        rows.extend(r[1:])
    if header is None:
        print("No result rows produced (every run failed?).")
        return
    with open(RESULTS, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print("Merged %d rows -> %s" % (len(rows), RESULTS))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true",
                    help="tiny sweep to smoke-test the pipeline")
    ap.add_argument("--maps", nargs="+", default=None)
    ap.add_argument("--robots", nargs="+", type=int, default=None)
    ap.add_argument("--intervals", nargs="+", type=int, default=None)
    ap.add_argument("--seeds", nargs="+", type=int, default=None)
    ap.add_argument("--jobs", "-j", type=int, default=1,
                    help="number of Webots runs to execute concurrently")
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

    # Fresh parts dir each sweep so a stale part can't leak into the merge.
    if os.path.isdir(PARTS_DIR):
        for old in glob.glob(os.path.join(PARTS_DIR, "*.csv")):
            os.remove(old)
    else:
        os.makedirs(PARTS_DIR)

    tasks = [(m, r, it, s)
             for m in maps for r in robots for it in intervals for s in seeds]
    total = len(tasks)
    jobs  = max(1, args.jobs)
    print("Sweep: %d runs, %d concurrent -> %s" % (total, jobs, RESULTS))
    print("  maps=%s robots=%s intervals=%s seeds=%s run=%.0fs"
          % (maps, robots, intervals, seeds, RUN_SECONDS))
    if not os.path.exists(WORLD):
        sys.exit("World not found: %s" % WORLD)

    t0 = time.time()
    parts = []
    if jobs == 1:
        for i, (m, r, it, s) in enumerate(tasks, 1):
            parts.append(run_one(i, total, m, r, it, s))
    else:
        with ThreadPoolExecutor(max_workers=jobs) as ex:
            futs = [ex.submit(run_one, i, total, m, r, it, s)
                    for i, (m, r, it, s) in enumerate(tasks, 1)]
            for fut in as_completed(futs):
                parts.append(fut.result())

    merge_parts(parts)
    print("Done in %.0fs (%.1f min)." % (time.time() - t0, (time.time() - t0) / 60.0))


if __name__ == "__main__":
    main()
