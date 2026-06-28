# RescueBot benchmark

Sweeps **map difficulty × team size × goal spawn-rate × seed** and produces
article-ready figures and a saturation summary.

## What varies

| Axis      | Values (default)                  | Set via env (per run)   |
|-----------|-----------------------------------|-------------------------|
| map       | `easy`, `medium`, `hard`          | `RESCUE_MAP`            |
| robots    | 1, 2, 3, 4, 5                     | `RESCUE_ROBOTS`         |
| interval  | 5, 10, 15, 20, 25, 30 s (median)  | `RESCUE_INTERVAL`       |
| seed      | 0, 1, 2                           | `RESCUE_SEED`           |
| run length| 300 sim-seconds                   | `RESCUE_RUN_SECONDS`    |
| gap std   | 0.3 × interval (Gaussian)         | `RESCUE_STD`            |
| csv out   | `benchmark/results.csv`           | `RESCUE_RESULT_CSV`     |

Goal inter-arrival times are **Gaussian** with median = `RESCUE_INTERVAL`.
Maps are spawned procedurally by the supervisor (see
`controllers/signal_controller/bench_maps.py`), so the single `arena.wbt`
serves all three difficulties.

## Metrics logged per run (one CSV row)

`spawned`, `rescued`, `throughput_min` (rescues/min), `mean/median/p95
latency` (spawn→rescue, s), `mean_backlog` (unrescued queue), `final_backlog`.

## Run it

```bash
# smoke-test the whole pipeline first (a handful of short runs)
python3 benchmark/run_sweep.py --quick --fresh

# full sweep (3 maps × 5 robots × 6 rates × 3 seeds = 270 runs)
python3 benchmark/run_sweep.py --fresh

# figures + summary you can paste into the paper
python3 benchmark/plot_results.py benchmark/results.csv
#  → benchmark/figures/{throughput,latency,backlog}.png
```

Needs Webots on `PATH` (or `WEBOTS_BIN=/path/to/webots`), and the Python set
in Webots' *Python command* must have numpy + matplotlib + Pillow.

## Notes

- A single run = one headless Webots launch (`--batch --mode=fast
  --no-rendering`). The supervisor auto-exits at `RESCUE_RUN_SECONDS` and
  appends its row, then `simulationQuit` lets the runner advance.
- 270 runs is a lot of wall-clock; start with `--quick`, then trim
  `INTERVALS`/`SEEDS` at the top of `run_sweep.py` or pass `--intervals`/
  `--seeds` to focus on the saturation region.
- Each `(config, seed)` is fully reproducible (the seed governs beacon
  positions, colours and the Gaussian gaps).
