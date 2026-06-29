"""
Wall layouts for the benchmark, one list per difficulty.

A wall is (centre_x, centre_y, size_x, size_y) in metres, height is fixed.
signal_controller spawns these as Solid boxes at startup, so the single world
file covers all three maps (pick one with the RESCUE_MAP env var). It also reads
the same list to keep beacons from spawning on top of a wall.

  easy:   almost open, just two short walls
  medium: the original 7-wall arena
  hard:   serpentine maze with dead-end stubs. the centre is left clear so the
          spawn ring never drops a robot onto a wall
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
