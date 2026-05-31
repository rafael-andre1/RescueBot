# Iteration 5: RescueBot Swarm

## Goals

 1. Mutliple beacons broadcasting on dedicated channels
 2. Multiple RescueBots (swarm), one per beacon channel
 3. Urgency tier from beacon color (red, yellow, green)
 4. One incrementally built shared map without GPS
 5. Smoother and faster obstacle avoidance
 6. Earlier and more robust visual confirmation of the distress signal


## World Updates (`arena_swarm.wbt`)

| Element | Choice | Why |
|---|---|---|
| Floor | $9 \times 7$ m | Big enough to host $3$ corner beacons and intricate interior walls |
| Outer walls | 4 boxes forming a sealed perimeter | Keeps RescueBots inside the mapped area |
| Obstacles | I or L shaped barriers near each beacon corner | Forces a detour &stresses navigation |
| Central clutter | 4 free standing partial walls | Breaks long open lanes; leaves a clear start triangle |
| Beacons | 3 colored `Robot` nodes in 3 far corners | One per channel; emissive material so the camera sees pure RGB |
| RescueBots | 3 e-pucks near origin in a triangle | $\approx 0.25$ m apart |


## Beacons

Each beacon is a `Robot` with one `Emitter` and one colored emissive `Box`. The Python controller is the same for all three, channel and color are configured in the world file.

| Beacon | Channel | Color | Position | Urgency tier |
|---|---|---|---|---|
| `pinger_red` | 1 | $(1, 0, 0)$ | $(+3.8, +2.8)$ | Critically Time Sensitive |
| `pinger_yellow` | 2 | $(1, 1, 0)$ | $(-3.8, +2.8)$ | Important |
| `pinger_green` | 3 | $(0, 1, 0)$ | $(+3.8, -2.8)$ | Routine |

`signal_controller_swarm.py` retains previous functioning.


## RescueBots

Each RescueBot is an e-puck with `receiver_channel` matching exactly one beacon. Color identity and start position are passed via `controllerArgs`.

| Scout | Channel | Target color | Start position |
|---|---|---|---|
| `RescueBot_red` | 1 | red | $(0.0, +0.30)$ |
| `RescueBot_yellow` | 2 | yellow | $(-0.26, -0.15)$ |
| `RescueBot_green` | 3 | green | $(+0.26, -0.15)$ |


### Identity from controllerArgs

```

controllerArgs ["<id>" "<color>" "<start_x>" "<start_y>"]

```

| Arg | Used for |
|---|---|
| `id` | Pickle filename, log prefix |
| `color` | Selects the camera RGB band |
| `start_x`, `start_y` | Used to align peer maps later |

| Color | $R$ range | $G$ range | $B$ range |
|---|---|---|---|
| red | $[200, 255]$ | $[0, 60]$ | $[0, 60]$ |
| yellow | $[200, 255]$ | $[200, 255]$ | $[0, 80]$ |
| green | $[0, 80]$ | $[200, 255]$ | $[0, 80]$ |


## Navigation Without GPS

Iteration 3 used `GPS` for the RescueBot position and for the SLAM grid origin. Iteration 5 removes the GPS device entirely. Position comes from wheel encoders; heading from the compass.

### Local Frame

Each RescueBot treats its own start as its private origin $(0, 0)$. The map lives in this local frame. Because compass heading is global, every RescueBot's local frame shares the same axes as the world, which means peer maps can later be merged by pure translation, no rotation.

### Wheel Encoder Odometry

| Constant | Value | Meaning |
|---|---|---|
| `WHEEL_RADIUS` | $0.0205$ m | e-puck wheel radius |
| `AXLE_LENGTH` | $0.052$ m | e-puck wheel separation |

Each step reads both wheel position sensors and converts the angular delta to linear delta:

$$ \Delta l = (l_t - l_{t-1}) \cdot r_w $$

$$ \Delta r = (r_t - r_{t-1}) \cdot r_w $$

$$ \Delta s = \frac{\Delta l + \Delta r}{2} $$

Heading $\theta$ comes from the compass:

$$ \theta = \text{atan2}(c_x, c_y) $$

Local position integrates each step:

$$ x_t = x_{t-1} + \Delta s \cdot \cos \theta $$

$$ y_t = y_{t-1} + \Delta s \cdot \sin \theta $$

`world_to_pix(lx, ly)` no longer subtracts an origin because the origin is already $(0, 0)$:

$$ p_x = \text{MAP\_CENTRE} + \frac{l_x}{W_x} \cdot \text{MAP\_CENTRE} $$

$$ p_y = \text{MAP\_CENTRE} - \frac{l_y}{W_y} \cdot \text{MAP\_CENTRE} $$


### Faster Obstacle Avoidance: Potential Field

We use the antenna sweep approximation to compute the attraction vector each robot is, well, attracted to.

Then, each time lidar finds an obstacle which is within the limit avoidance distance (0.3m), it computes multiple smaller repulsion vectors. 

These vectors are all combined, producing the direction in which the robot can simultaneously:

 1. avoid the obstacle
 2. retain stability and speed
 3. approximate to distress signal

| Vector | Direction | Magnitude |
|---|---|---|
| Attraction | ping bearing in robot frame | $1$ |
| Repulsion (per close lidar beam, forward $180°$ only) | opposite of beam direction | $G \cdot t^2$, with $t = (D - r)/D$ |

| Parameter | Value | Reasoning |
|---|---|---|
| `NAV_REPULSE_DIST` $D$ | $0.30$ m | Tight: RescueBot grazes walls without bouncing off early |
| `NAV_REPULSE_GAIN` $G$ | $2.2$ | Strength vs unit attraction; small enough that the ping wins on open ground |
| `NAV_SLOW_DIST` | $0.28$ m | Front cone clearance below this throttles forward speed |
| `NAV_MIN_SPEED` | $1.2$ rad/s | Floor on forward speed so the RescueBot never crawls |
| `NAV_BEAM_STRIDE` | $3$ | Subsamples lidar to $\approx 120$ beams per step |



Why this is faster and smoother than the previous avoidance:

| Old behaviour | New behaviour |
|---|---|
| Threshold trigger flips wheel speeds abruptly | Heading changes continuously as obstacles enter and leave range |
| Commits to a side (left or right) | No side commitment; ping bearing always in the sum |
| Stutters between "go to ping" and "spin away" | One smooth steering command per step |
| Could lock onto the wrong wall | Cannot follow a wall; only deflects to keep moving toward the ping |




## Visual Distress Confirmation

To confirm urgency and finding ping, two conditions must hold:

 1. can clearly see the distress signal
 2. is close enough to reach it without obstacles

| Condition | Threshold | What it ensures |
|---|---|---|
| Camera color confirmed | `person_found` $=$ True | The right beacon is actually in view |
| Radio range close enough | $\hat r < $ `RADIO_STOP_DIST` $= 0.80$ m | Beacon fills a centered, comfortable fraction of the frame |


| Parameter | Value | Reasoning |
|---|---|---|
| `CAMERA_DIST` | $1.15$ m | Beacon already occupies a meaningful pixel patch |
| `TARGET_PIXEL_MIN` | $20$ | Rejects single pixel noise and distant specks |
| `RADIO_STOP_DIST` | $0.80$ m | Far enough back that the beacon is centered, with empty frame around it |

### Urgency Tier from Color

Each RescueBot has a different color band hardcoded by its `color` arg:

```py

 dict_colors = {red: critical, yellow:important, green:routine}

```

### Beacon Position Estimate

Uses the approximated distance, calculated from the radio.


## Shared State via Pickle

RescueBots never exchange anything during the run. Each one writes a single pickle file to disk on arrival; later arrivals merge any pickles already present.

### Filenames

| File | Written by | Contents |
|---|---|---|
| `slam_state_1.pkl` | RescueBot_red | full state when red arrives |
| `slam_state_2.pkl` | RescueBot_yellow | full state when yellow arrives |
| `slam_state_3.pkl` | RescueBot_green | full state when green arrives |
| `slam_map_stage1.png` | the 1st arrival | map of just that one RescueBot |
| `slam_map_stage2.png` | the 2nd arrival | combined map of first two |
| `slam_map_stage3.png` | the 3rd arrival | combined map of all three |

### Pickle Payload

Each `slam_state_<id>.pkl` is one Python dict:

| Key | Type | Frame | Why it is needed |
|---|---|---|---|
| `id` | str | n/a | Identifies the RescueBot |
| `label` | str | n/a | Legend label |
| `colour` | str | n/a | Plot color |
| `start_pos` | tuple | world | Translation that places this RescueBot's $(0, 0)$ inside the merged frame |
| `hits` | $400 \times 400$ list | local pixels | Per cell wall hit counts |
| `visits` | $400 \times 400$ list | local pixels | Per cell ray pass counts |
| `trajectory` | list of `(lx, ly)` | local | Path polyline |
| `pinger_pos` | `(lx, ly)` | local | Estimated beacon position |
| `stop_pos` | `(lx, ly)` | local | Where the RescueBot halted |
| `find_time` | float | sim seconds | Time from sim start to arrival |
| `wall_time` | float | unix seconds | Used for sorting arrivals chronologically |



## Combining Maps Without GPS

This is the trick that lets the swarm cooperate without exchanging position during the run.

Every RescueBot knows its world translation at startup through `controllerArgs`. That one 2D vector is all another RescueBot needs to match the other robots' origins and combine the map.


The first RescueBot to arrive becomes the reference. Its start position becomes the global $(0, 0)$ of the merged map. Every other RescueBot's local map is translated by

$$ \Delta = \text{peer.start\_pos} - \text{ref.start\_pos} $$

converted to pixel offsets:

$$ \text{off}_{px} = \text{round}\left(\frac{\Delta_x}{W_x} \cdot \text{MAP\_CENTRE}\right) $$

$$ \text{off}_{py} = -\text{round}\left(\frac{\Delta_y}{W_y} \cdot \text{MAP\_CENTRE}\right) $$

The peer's `hits` and `visits` grids are added cell by cell into a global grid using array slicing with these offsets. Trajectories, stop points, and beacon estimates undergo the same translation when plotted.

No rotation is ever needed because every RescueBot's local frame is already axis aligned with the world (compass).

### Staged Output

When a RescueBot arrives:

 1. It writes its own pickle.
 2. It calls `load_all_states()` which reads every valid pickle and sorts by `wall_time` (true arrival order).
 3. It calls `save_map_image()` with the full list and `ref_start = states[0]["start_pos"]`.
 4. It saves `slam_map_stage<N>.png` where $N$ is the number of pickles found.




## Map Visualisation

| Marker | Meaning |
|---|---|
| Filled colored cross | Scout start position (after translation into the merged frame) |
| Open colored circle | Scout stop position |
| Filled colored star | Estimated beacon position from radio range and bearing |
| Dashed colored line | Connects stop to beacon estimate; shows radio error visually |
| Colored polyline | Scout trajectory |
| Dark grey | Wall cells |
| White | Free explored cells |
| Light grey | Unexplored cells |

The legend lives in a column to the right of the axes so it never overlaps the map.


## Limitations

 1. Not tested for when 2 pings are coming from the same space (robot collision and mutual harm)
 2. Each robot is only mapped to one channel (maybe after a robot reaches its destination it can try and chase another ping, the one it's closest to, and lock in on that channel after a channel sweep)
 3. Efficiency issues might happen: no Dijkstra or mechanism to avoid local minima, might get stuck in endless loop of wrong positions and directions due to lack of memory, no way to get optimal path planning for rescue team to reach pings
 4. Each channel is being associated to a color automatically: that color must be randomly assigned to pings and the robot must only aquire that color for the plot once it reaches the military team and outputs the map
    1. maybe not a bad idea to have more than 3 colors, and then give a list of urgency based on the most ammount of red in the camera vision
