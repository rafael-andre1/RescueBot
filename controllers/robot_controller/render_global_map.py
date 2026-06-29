#!/usr/bin/env python3
"""
Rebuild the global map from saved state, no sim run needed.

Each robot drops its SLAM state to slam_state_<id>.pkl while it runs: the grid,
its world start, the path it took, and the pings it confirmed. This reads all of
them back, stitches them into one map centred on the world origin, and writes
both versions:

    maps_traj/global_map.png         walls + paths + starts + stops + pings
    clean_maps/global_clean_map.png  walls + pings only

The legend sits to the right of the plot in both, so it can't land on a ping.

    python render_global_map.py [state_dir] [--clean-only] [--traj-only]

state_dir defaults to this file's folder, which is where the pickles end up.
"""
import os
import sys
import glob
import pickle

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

# these have to match robot_controller.py, otherwise the offline map comes
# out looking different from the one the sim drew
MAP_SIZE       = 200
MAP_CENTRE     = MAP_SIZE // 2
WORLD_X_MAX    = 4.0
WORLD_Y_MAX    = 3.0
WALL_CERTAINTY = 0.30
IMPASSABLE     = 0.90
GLOBAL_REF_START = (0.0, 0.0)            # shared centre = world origin

STATE_GLOB     = "slam_state_*.pkl"
MAPS_TRAJ_DIR  = "maps_traj"
CLEAN_MAPS_DIR = "clean_maps"

# beacon dots always take the colour the camera read off the box
COLOUR_MPL = {"red": "red", "yellow": "gold", "green": "limegreen"}

# one fixed colour per robot, kept off red/yellow/green so a path never reads
# as a ping. same dict the controller uses
ROBOT_TRAJ_COLOURS = {
    0: "tab:purple",
    1: "tab:orange",
    2: "tab:brown",
    3: "tab:cyan",
    4: "tab:pink",
    5: "slategray",
    6: "navy",
    7: "darkviolet",
}
_ROBOT_TRAJ_PALETTE = list(ROBOT_TRAJ_COLOURS.values())


def robot_colour(rid):
    if rid in ROBOT_TRAJ_COLOURS:
        return ROBOT_TRAJ_COLOURS[rid]
    return _ROBOT_TRAJ_PALETTE[rid % len(_ROBOT_TRAJ_PALETTE)]


def load_all_states(state_dir):
    """Load every slam_state_*.pkl in state_dir, sorted by id."""
    out = []
    for path in glob.glob(os.path.join(state_dir, STATE_GLOB)):
        try:
            with open(path, "rb") as f:
                out.append(pickle.load(f))
        except (OSError, EOFError, pickle.UnpicklingError) as e:
            print("  ! skipping %s (%s)" % (path, e))
    out.sort(key=lambda s: s.get("id", 0))
    return out


def save_global_map(robot_maps, filename, ref_start, clean=False):
    """Same merge and render the controller does, legend pushed off to the side."""
    rsx, rsy = ref_start

    g_hits   = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
    g_visits = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.int32)
    for rm in robot_maps:
        sx, sy = rm["start_pos"]
        dx, dy = sx - rsx, sy - rsy
        off_px =  round(dx / WORLD_X_MAX * MAP_CENTRE)
        off_py = -round(dy / WORLD_Y_MAX * MAP_CENTRE)
        h = np.array(rm["hits"],   dtype=np.int32)
        v = np.array(rm["visits"], dtype=np.int32)
        sx0 = max(0, -off_px); sx1 = min(MAP_SIZE, MAP_SIZE - off_px)
        dx0 = max(0,  off_px); dx1 = min(MAP_SIZE, MAP_SIZE + off_px)
        sy0 = max(0, -off_py); sy1 = min(MAP_SIZE, MAP_SIZE - off_py)
        dy0 = max(0,  off_py); dy1 = min(MAP_SIZE, MAP_SIZE + off_py)
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

    fig, ax = plt.subplots(figsize=(11, 7) if not clean else (9, 7), dpi=150)
    ax.imshow(img, origin="upper",
              extent=[-WORLD_X_MAX, WORLD_X_MAX, -WORLD_Y_MAX, WORLD_Y_MAX],
              aspect="equal", interpolation="nearest")
    ax.set_xlabel("X (m)  — shared world frame", fontsize=9)
    ax.set_ylabel("Y (m)  — shared world frame", fontsize=9)
    ax.grid(True, linestyle="--", linewidth=0.4, alpha=0.4, color="#888888")

    legend_handles = [
        mpatches.Patch(facecolor=(0.20, 0.20, 0.20), label="Wall"),
        mpatches.Patch(facecolor=(1.00, 1.00, 1.00),
                       edgecolor="gray", linewidth=0.5, label="Free (explored)"),
        mpatches.Patch(facecolor=(0.93, 0.93, 0.93),
                       edgecolor="gray", linewidth=0.5, label="Unexplored"),
    ]

    total_rescues = 0
    seen_colours  = set()

    # ping stars, on both versions, coloured by whatever the camera called it
    for rm in robot_maps:
        sx, sy = rm["start_pos"]
        sxr, syr = sx - rsx, sy - rsy
        rps = rm.get("rescued_positions", [])
        rcs = rm.get("rescued_colours", [])
        total_rescues += len(rps)
        for j, (px, py) in enumerate(rps):
            cname = rcs[j] if j < len(rcs) else None
            mc = COLOUR_MPL.get(cname, "crimson")
            if cname in COLOUR_MPL:
                seen_colours.add(cname)
            ax.plot(px + sxr, py + syr, linestyle="none", marker="*", color=mc,
                    markersize=15, zorder=6,
                    markeredgecolor="black", markeredgewidth=0.6)
    for cname in ("red", "yellow", "green"):
        if cname in seen_colours:
            legend_handles.append(
                Line2D([0], [0], marker="*", color="w",
                       markerfacecolor=COLOUR_MPL[cname], markeredgecolor="black",
                       markersize=12, label="%s beacon (camera id)" % cname))

    # paths, starts, stop circles and the dashed link, only on the full map
    if not clean:
        for idx, rm in enumerate(robot_maps):
            rid = rm.get("id", idx)
            lbl = rm.get("label", "RescueBot %d" % rid)
            sx, sy = rm["start_pos"]
            sxr, syr = sx - rsx, sy - rsy
            tcol = robot_colour(rid)
            traj = rm.get("trajectory", [])
            rps  = rm.get("rescued_positions", [])
            rss  = rm.get("rescued_stops", [])
            if len(traj) > 1:
                tx = [p[0] + sxr for p in traj]
                ty = [p[1] + syr for p in traj]
                ax.plot(tx, ty, color=tcol, linewidth=1.1, alpha=0.85, zorder=3)
                legend_handles.append(
                    Line2D([0], [0], color=tcol, linewidth=1.5,
                           label="%s trajectory" % lbl))
            ax.plot(sxr, syr, marker="P", color=tcol, markersize=9,
                    markeredgecolor="white", markeredgewidth=0.7, zorder=5)
            legend_handles.append(
                Line2D([0], [0], marker="P", color="w", markerfacecolor=tcol,
                       markersize=8, label="%s start" % lbl))
            has_stop = False
            for j, (stx, sty) in enumerate(rss):
                sxp, syp = stx + sxr, sty + syr
                ax.plot(sxp, syp, marker="o", markerfacecolor="none",
                        markeredgecolor=tcol, markeredgewidth=1.8,
                        markersize=11, zorder=5)
                has_stop = True
                if j < len(rps):
                    ppx, ppy = rps[j][0] + sxr, rps[j][1] + syr
                    ax.plot([sxp, ppx], [syp, ppy], color=tcol,
                            linewidth=1.0, linestyle="--", alpha=0.7, zorder=4)
            if has_stop:
                legend_handles.append(
                    Line2D([0], [0], marker="o", color="w",
                           markerfacecolor="none", markeredgecolor=tcol,
                           markeredgewidth=1.8, markersize=10,
                           label="%s stop point" % lbl))
        ax.set_title("Global SLAM map — %d RescueBot%s · %d rescue%s"
                     % (len(robot_maps), "s" if len(robot_maps) != 1 else "",
                        total_rescues, "s" if total_rescues != 1 else ""),
                     fontsize=11)
    else:
        ax.set_title("Global obstacle + ping map — %d rescue%s"
                     % (total_rescues, "s" if total_rescues != 1 else ""),
                     fontsize=11)

    # legend off to the right on both, so it never lands on the map
    ax.legend(handles=legend_handles, bbox_to_anchor=(1.02, 1.0),
              loc="upper left", borderaxespad=0.0,
              fontsize=8, framealpha=0.95, edgecolor="gray")
    fig.subplots_adjust(left=0.07, right=0.72 if not clean else 0.78,
                        top=0.93, bottom=0.08)

    fig.savefig(filename, dpi=150, bbox_inches="tight", format="png")
    plt.close(fig)
    print("  wrote %s" % filename)


def main():
    args = [a for a in sys.argv[1:]]
    clean_only = "--clean-only" in args
    traj_only  = "--traj-only" in args
    args = [a for a in args if not a.startswith("--")]

    state_dir = args[0] if args else os.path.dirname(os.path.abspath(__file__))
    print("Reading state from: %s" % state_dir)

    states = load_all_states(state_dir)
    if not states:
        print("No %s found in %s — run the sim first (or pass the right dir)."
              % (STATE_GLOB, state_dir))
        return 1
    total = sum(len(s.get("rescued_positions", [])) for s in states)
    print("Loaded %d RescueBot state(s), %d rescue(s) total." % (len(states), total))

    os.makedirs(os.path.join(state_dir, MAPS_TRAJ_DIR),  exist_ok=True)
    os.makedirs(os.path.join(state_dir, CLEAN_MAPS_DIR), exist_ok=True)

    if not clean_only:
        save_global_map(states,
                        os.path.join(state_dir, MAPS_TRAJ_DIR, "global_map.png"),
                        GLOBAL_REF_START, clean=False)
    if not traj_only:
        save_global_map(states,
                        os.path.join(state_dir, CLEAN_MAPS_DIR, "global_clean_map.png"),
                        GLOBAL_REF_START, clean=True)
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
