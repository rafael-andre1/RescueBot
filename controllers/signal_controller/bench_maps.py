"""
bench_maps.py — wall layouts for the benchmark, keyed by difficulty.

Each wall is (centre_x, centre_y, size_x, size_y) in metres; height is fixed.
The signal_controller spawns these as Solid boxes at startup (so one world
file serves all three maps — the map is chosen via the RESCUE_MAP env var)
and also uses them to keep beacons clear of walls.

  easy   — nearly open, 2 short walls
  medium — the original 7-wall arena
  hard   — serpentine maze with dead-end stubs (centre kept clear so the
           scouts' spawn ring never lands on a wall)
"""

WALL_HEIGHT = 0.3
WALL_Z      = 0.15

MAPS = {
    "easy": [
        (0.0,  1.2,  0.05, 1.0),   # vertical wall, kept off the central spawn ring
        (1.5, -1.0,  1.0,  0.05),
    ],
    "medium": [
        (1.5,   0.0,  0.05, 2.0),
        (-1.5, -1.0,  2.0,  0.05),
        (0.0,   2.0,  1.5,  0.05),
        (-2.0,  0.5,  0.05, 2.0),
        (2.0,  -1.5,  2.0,  0.05),
        (2.5,   1.25, 0.05, 2.0),
        (-1.0,  1.0,  1.0,  0.05),
    ],
    "hard": [
        (-2.5,  0.0,  0.05, 4.0),   # left wall, gaps top & bottom
        (-0.8,  1.0,  0.05, 4.0),   # gap at the bottom
        (0.8,  -1.0,  0.05, 4.0),   # gap at the top
        (2.5,   0.0,  0.05, 4.0),   # right wall, gaps top & bottom
        (-1.6, -1.2,  1.4,  0.05),  # dead-end stub
        (1.6,   1.2,  1.4,  0.05),  # dead-end stub
        (0.0,  -2.4,  1.2,  0.05),  # lower baffle
        (0.0,   2.4,  1.2,  0.05),  # upper baffle
    ],
}


def get_map(name):
    """Return the wall list for a difficulty name (default medium)."""
    return MAPS.get(name, MAPS["medium"])
