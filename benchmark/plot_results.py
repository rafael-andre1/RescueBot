#!/usr/bin/env python3
"""
Turn benchmark/results.csv into figures and a short text summary.

For each map it draws three panels against the goal spawn-interval, one line per
robot count, averaged over the seeds (error bars are the std across seeds):

    throughput  (rescues/min)   how much the team clears
    mean latency (s)            spawn-to-rescue delay
    mean backlog (#)            how long the queue gets

It also prints, per (map, robots), the largest interval each team keeps stable
before backlog and latency blow up.

    python3 benchmark/plot_results.py [results.csv]

Writes benchmark/figures/*.png
"""
import os
import sys
import csv
import statistics
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE    = os.path.dirname(os.path.abspath(__file__))
CSV     = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results.csv")
FIGDIR  = os.path.join(HERE, "figures")
METRICS = [("throughput_min", "Throughput (rescues/min)", "throughput"),
           ("mean_latency",   "Mean rescue latency (s)",  "latency"),
           ("mean_backlog",   "Mean backlog (# unrescued)", "backlog")]


def load(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def fnum(v):
    try:
        return float(v)
    except (ValueError, TypeError):
        return None


def aggregate(rows, metric):
    """(map, robots, interval) → (mean, std) over seeds for one metric."""
    buckets = defaultdict(list)
    for r in rows:
        v = fnum(r.get(metric))
        if v is None:
            continue
        key = (r["map"], int(r["robots"]), float(r["interval"]))
        buckets[key].append(v)
    out = {}
    for key, vals in buckets.items():
        out[key] = (statistics.mean(vals),
                    statistics.pstdev(vals) if len(vals) > 1 else 0.0)
    return out


def plot_metric(rows, metric, ylabel, slug, maps, robot_list, intervals):
    agg = aggregate(rows, metric)
    fig, axes = plt.subplots(1, len(maps), figsize=(5 * len(maps), 4.2),
                             sharey=True, squeeze=False)
    for ax, m in zip(axes[0], maps):
        for rb in robot_list:
            xs, ys, es = [], [], []
            for it in intervals:
                if (m, rb, it) in agg:
                    mean, std = agg[(m, rb, it)]
                    xs.append(it); ys.append(mean); es.append(std)
            if xs:
                ax.errorbar(xs, ys, yerr=es, marker="o", capsize=3,
                            label="%d robot%s" % (rb, "s" if rb != 1 else ""))
        ax.set_title("map: %s" % m)
        ax.set_xlabel("Goal spawn interval (s)")
        ax.grid(True, ls="--", alpha=0.4)
    axes[0][0].set_ylabel(ylabel)
    axes[0][-1].legend(fontsize=8, title="team size")
    fig.suptitle(ylabel + " vs goal rate", fontsize=12)
    fig.tight_layout()
    out = os.path.join(FIGDIR, "%s.png" % slug)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  wrote", out)


def print_summary(rows, maps, robot_list, intervals):
    lat = aggregate(rows, "mean_latency")
    bk  = aggregate(rows, "mean_backlog")
    thr = aggregate(rows, "throughput_min")
    print("\n=== SUMMARY (saturation = largest interval kept stable) ===")
    print("Heuristic: 'stable' = mean backlog < 2 AND mean latency < 30 s.\n")
    for m in maps:
        print("map: %s" % m)
        for rb in robot_list:
            stable_iv = None
            for it in sorted(intervals):              # large interval = easy load
                b = bk.get((m, rb, it)); l = lat.get((m, rb, it))
                if b and l and b[0] < 2.0 and l[0] < 30.0:
                    stable_iv = it
                    break
            # peak sustained throughput across intervals
            peak = max((thr[(m, rb, it)][0] for it in intervals
                        if (m, rb, it) in thr), default=0.0)
            sat = ("stable from ~%ds gap" % stable_iv) if stable_iv \
                  else "saturated at all tested rates"
            print("  %d robot(s): peak %.1f rescues/min | %s"
                  % (rb, peak, sat))
        print()


def main():
    if not os.path.exists(CSV):
        sys.exit("No results CSV at %s — run run_sweep.py first." % CSV)
    rows = load(CSV)
    if not rows:
        sys.exit("Results CSV is empty.")
    os.makedirs(FIGDIR, exist_ok=True)

    maps       = sorted({r["map"] for r in rows},
                        key=lambda x: {"easy": 0, "medium": 1, "hard": 2}.get(x, 9))
    robot_list = sorted({int(r["robots"]) for r in rows})
    intervals  = sorted({float(r["interval"]) for r in rows})
    print("Loaded %d rows | maps=%s robots=%s intervals=%s"
          % (len(rows), maps, robot_list, intervals))

    for metric, ylabel, slug in METRICS:
        plot_metric(rows, metric, ylabel, slug, maps, robot_list, intervals)
    print_summary(rows, maps, robot_list, intervals)


if __name__ == "__main__":
    main()
