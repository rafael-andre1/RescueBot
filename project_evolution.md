# Iteration 1: Baseline

## Goals

 1. World Baseline
    1. Flat surface (Z up coordinates)
    2. Easy wall creation
 2. Basic Robot
    1. Basic wall avoidance
    2. Signal receiver (channel based)
    3. Also features camera
 3. Basic Pinger (Distress Beacon)
    1. Fixed position signal (channel based) generator robot

These small steps represent the most basic skeleton of our intermediate goals:

 1. obstacle avoidance
 2. primitive pathfinding through LiDAR


## The world (`arena.wbt`)

| Element | Choice | Why |
|---|---|---|
| `coordinateSystem` | `ENU` (Z-up) | Standard for ground robotics; matches ROS convention and most tutorials. |
| `basicTimeStep` | 32 ms | Webots default. ~30 Hz physics is plenty for a slow ground robot and keeps real-time factor close to 1. |
| Floor | `Floor 10 * 10 m` | Large enough that the robot can wander for minutes before reaching an edge, small enough that the LiDAR's 5 m range covers most of it. |
| Walls | Three free-standing `Solid { Box }` blocks, 0.3 m tall | Cheaper to author than a closed maze, and copy-pasting one block is the simplest possible "add a wall" UX. Height of 0.3 m is well above the LiDAR (which sits at ~0.08 m), so the sensor always sees them. |
| Wall `boundingObject` | Duplicates the visual `Box` size | Required for actual physical collision: without it, the robot would pass straight through. |

**To add a wall in a demo:** copy any `Solid` block, change `translation`, `name`, and both `size` fields. Webots hot-reloads.

## RescueBot

Very simple differential drive controller, inspired by the simplest version of our previous project.


### Controller

Each simulation step (every 32 ms):


1. **LiDAR**: 
   1. `getRangeImage()` returns 360 distances
   2. looks at only the forward ±30° arc (`FRONT_ARC_DEG = 60`)
   3. considers side rays irrelevant for forward collision avoidance
2. **Decide**: 
   1. If the minimum distance in the forward arc is below `SAFE_DISTANCE`, set the motors to opposite velocities (spin in place left)
   2. Otherwise drive both wheels forward at `CRUISE_SPEED`

#### Parameter Reasoning

| Parameter | Value | Reasoning |
|---|---|---|
| `SAFE_DISTANCE` | 0.4 m | ~ 2x robot body length to allow full safe rotation|
| `FRONT_ARC_DEG` | 60° (±30°) | balance between avoiding useless turns and considering a good range|
| `CRUISE_SPEED` | 3.0 rad/s (~0.12 m/s) | enough time to detect walls before collision |
| `TURN_SPEED` | 2.0 rad/s | slower than cruise|


## Distress Beacon

The simplest possible Webots actor: a static red cube with one device.

| Parameter | Value | Reasoning |
|---|---|---|
| `Emitter.channel` | 1 | Matches the RescueBot's receiver. |
| `Emitter.range` | -1 | Infinite range. Iteration 1 doesn't care about distance attenuation; we want the ping to always arrive so we can test the receive path. (Later iterations can shorten this to simulate a real radio.) |
| Payload | `"SOS"` (UTF-8 bytes) | Smallest meaningful message. The RescueBot's signal-strength reading and `getEmitterDirection()` vector carry the useful information; the payload is just a label. |
| `PING_INTERVAL` | 3.0 s | Slow enough that console output is readable, fast enough to see multiple pings during a short demo. |



## Limitations 

 1. Not chasing ping
 2. Stuttery movement


# Iteration 2: Chasing Ping with Exact Coordinates

## Goals

 1. Know where I am
    1. GPS (position)
    2. Compass (heading)
 2. Search, don't just wander
    1. Expanding spiral until a ping arrives
 3. Act on the ping
    1. Read the source position from the message
    2. Drive straight to it, dodging walls
 4. Build a map while moving (SLAM occupancy grid)

Iteration 1 only avoided walls. Iteration 2 turns it into a rescuer, and uses GPS and compass for global navigation.

## World Updates 

| Element | Choice | Why |
|---|---|---|
| `GPS` + `Compass` on RescueBot | onboard | Need position + heading to map and to steer toward a target. |
| `Display` "map_display" 200*200 | turret slot | Live view of the occupancy map being built. |
| Pinger `GPS` | added | So the beacon can broadcast its own coordinates. |
| Pinger payload | `"SOS <x> <y>"` | Carries the position, not just a label|


## RescueBot

The robot itself was updated to **e-puck** for convenience.

It works by **spiralling outward until it hears a ping, then driving straight to the coordinates inside that ping** while avoiding walls the whole time.

### Controller

Each step:

1. **Localise**: read GPS (position) and compass (heading).
2. **Map (SLAM)**: ray-trace the LiDAR into an occupancy grid
   1. **cells hit** = likely wall 
   2. **cells passed** = likely free
3. **Listen**: wipe receiver, lock in on `"SOS x y"`
4. **Drive**:
   1. No ping yet: spiral search
   2. Ping received: steer straight at the target
   3. Wall ahead (`AVOID_DISTANCE`): avoid then resume
5. **Stop** within `GOAL_TOL` of the target

### SLAM map

The map is essentially a matrix simulating an occupancy grid. 

#### Dijkstra

Currently, this only works because we have the exact positioning of the rescue beacon: 
 1. `compute_cost_matrix()` floods **from the goal outward** (not from the robot!)
 2. Every reachable cell is marked as `cost[y][x]`, which is the cheapest path back to the goal
 3. Edge weight = `move_dist * cell_cost`
    1. `cell_cost` is 1-9 scaled by occupancy, 1000 if impassable 


#### Wall Pixel Expansion

We were having a few inneficient runs where Dijkstra was assuming the pixel immediately behind the wall was free.

As a solution, every wall cell gets expanded outward by `OBSTACLE_INFLATE = 2` pixels in all directions, written into a separate `inflated[y][x]` grid. This is just to match the approximated wall thickness, avoiding cases where Dijkstra tries to path plan on unseen sections of the wall, effectively treating them as impassable.


## Gradient follower

`follow_gradient()` starts at the robot's current pixel and greedily steps to whichever of the 8 neighbours has the lowest cost: 10 times. It returns that lookahead pixel, which the robot then steers toward.

Preferred over "steer straight at the goal" because the cost surface already encodes obstacles and occupancy. Descending the gradient automatically routes around walls without any extra collision logic. The 10-cell lookahead turns a per-pixel descent (which would produce jerky, pixel-by-pixel steering corrections) into a smooth intermediate waypoint a meaningful distance ahead.

| Cell state | How | Shown as |
|---|---|---|
| wall | `hits / visits` high | dark |
| free | visited, low hits | green |
| unseen | never scanned | grey |

The map is built and displayed every iteration. 

#### New Parameters

| Parameter | Value | 
|---|---|
| `GOAL_TOL` | 0.10 m |
| `AVOID_DISTANCE` | 0.22 m 
| `SPIRAL_DECAY` | 0.004 /step | 
| `WALL_CERTAINTY` | 0.30 | 
| `PING_INTERVAL` | 3.0 s | 


## Distress Beacon

| Parameter | Value | Reasoning |
|---|---|---|
| Payload | `"SOS <x> <y>"` | Now carries its own GPS position|
| `Emitter.channel` | 1 | Matches the RescueBot's receiver|


## Limitations

 1. Reaching ping is trivial: SOS message shares exact coordinates
 2. Only 1 distress signal


# Iteration 3: Signal Only Mapping


## Goals

 1. Remove the coordinate cheat
    1. Beacon broadcasts `"SOS"` only: no GPS payload
    2. RescueBot must use radio physics to find the source
 2. Navigate by signal
    1. Estimate range from signal strength ($\text{power} \propto 1/r^2$)
    2. Estimate bearing from the emitter direction vector
 3. Confirm with camera
    1. Activate camera when within short distance
    2. Count red pixels to confirm a person is visible (will change in the future)
 4. Better map
    1. Double grid resolution
    2. Anchor the map to the robot's own starting position, not the world origin
    3. Robust: prepared to receive multiple robot outputs and combine them
    4. Log and draw the full trajectory

## World Updates


Iteration 2 solved pathfinding by reading exact coordinates out of the SOS message: cheating! 

Iteration 3 removes that shortcut and forces the RescueBot to work from physics, by using predictable signal attenuation and direction.


| Element | Change | Why |
|---|---|---|
| Pinger `GPS` | removed | Beacon no longer needs to know its own position; it just shouts `"SOS"` |
| Pinger payload | `"SOS <x> <y>"` $\rightarrow$ `"SOS"` | Coordinate sharing is cheating |
| Map display | $200 * 200 \to 400 * 400$ | Higher resolution improves final map quality |
| Map origin | world centre $\rightarrow$ robot start | Useful for when we have multiple robots |

## RescueBot

The robot still spirals outward until it hears a ping, but what happens next is completely different. Instead of reading a target coordinate and handing it to Dijkstra, it **steers toward the radio source proportionally**, closing in until the estimated range drops below a proximity threshold.

### Bearing Parameter

**Bearing**: angle between the direction the robot is currently facing and the direction the signal is coming from.

In order to properly guide the robot, it uses `getEmitterDirection()` as an approximation of an antenna sweep, where:

 1. d[0] = how much of the direction is forward/back
 2. d[1] = how much is left/right
 3. d[2] = how much is up/down (ignored for a ground robot)

This information is purely directional, and so, to get bearing values from this, we can use $atan2$ to find the angle that is created by the directional vector and the x-axis. We use $atan2$ as opposed to $atan$ due to the fact that we need to cover a full $[-\pi, \pi]$ range:

$$ bearing = atan2(d_1,d_0) $$


| Distress Signal Position | $d_0$ | $d_1$ | $\text{atan2}(d_1, d_0)$ |
|---|---|---|---|
| Straight ahead | 1 | 0 | 0 |
| 90° to the left | 0 | 1 | $+\pi/2$ |
| 90° to the right | 0 | −1 | $-\pi/2$ |



### Controller

Each step:

1. **Localise**: GPS (position) + compass (heading), same as before.
2. **Map (NOT SLAM)**: ray-trace LiDAR into the occupancy grid, inflate obstacles, same as before.
3. **Listen**: for every `"SOS"` packet, record signal strength and get sweep direction
4. **Drive**:
   1. No ping yet: spiral search
   2. Ping received: compute bearing and steer toward it
   3. Wall ahead and range > `CAMERA_DIST`: reactive avoidance 
   4. Range < `RADIO_STOP_DIST`: stop, mark pinger position, save map
5. **Camera scan**: once within `CAMERA_DIST`, activate camera and count red pixels each step.

### Radio Chasing

`read_radio()` drains the receiver queue and returns three values from the strongest packet:

| Value | Formula | What it represents |
|---|---|---|
| `est_range` | `1 / sqrt(getSignalStrength())` | radio power falls as $1/r^2$ so range is $1/\sqrt{\text{strength}}$ |
| `bearing` | `atan2(d[1], d[0])` from `getEmitterDirection()` | Robot-relative angle to the emitter|
| `raw_strength` | `getSignalStrength()` | Logged for debugging and tracking |

`steer_to_bearing()` is a proportional controller:
 
 - large bearing error $\rightarrow$ nearly pure rotation
 - small error $\rightarrow$ mostly forward thrust


### Dijkstra removed

Iteration 2's cost matrix, `compute_cost_matrix()`, `replan()`, and `follow_gradient()` are all gone. They were only useful because the goal pixel was hardcoded. Without a known goal, there is nothing to flood-fill from. 

**Radio bearing replaces the gradient as the steering signal.**

### Camera confirmation

| Parameter | Value | Reasoning |
|---|---|---|
| `CAMERA_DIST` | 0.80 m | Close enough that a person fills a meaningful fraction of the frame |
| `RED_R_MIN` | 200 | Permissive high-red threshold |
| `RED_G_MAX / RED_B_MAX` | 60 | Keeps flesh-tone orange or pink from triggering |
| `RED_PIXEL_MIN` | 20 | Rejects single-pixel noise and distant specks |

Once within `CAMERA_DIST`, `scan_for_red()` runs every step and a half-scale preview of the camera frame is blitted into the top-right corner of the SLAM display. When 20 or more qualifying pixels are found, `"FOUND PERSON"` is printed to the console.

### Mapping Improvements

| Change | Detail |
|---|---|
| Resolution | 200 $\rightarrow$ 400 pixels; `OBSTACLE_INFLATE` scaled up to 3 to match |
| Map origin | First GPS reading is stored as `(origin_x, origin_y)` and subtracted in `world_to_pix()`, so the centre of the map is always the robot's start |
| Trajectory | Every cell-change appends `(robot_x, robot_y)` to a list; drawn as a cyan polyline on both the live display and the saved image |

| Cell state | How | Shown as |
|---|---|---|
| wall | `hits / visits > WALL_CERTAINTY` | dark grey $\rightarrow$ black |
| free | visited, low hits | green tint |
| unseen | never scanned | grey |
| trajectory | logged world coords | cyan polyline |
| pinger | proximity stop position | red square |

### New Parameters

| Parameter | Value |
|---|---|
| `MAP_SIZE` | 400 px |
| `OBSTACLE_INFLATE` | 3 px |
| `RADIO_STOP_DIST` | 0.35 m |
| `CAMERA_DIST` | 0.80 m |
| `RED_PIXEL_MIN` | 20 |
| `SIGNAL_PRINT_STEPS` | every 2 sim-s |

## Distress Beacon

| Parameter | Value | Reasoning |
|---|---|---|
| Payload | `"SOS"` | GPS coordinates removed; the radio signal itself is now the only information the RescueBot uses |
| `GPS` | removed | Beacon no longer needs to know its own position |
| `Emitter.channel` | 1 | Unchanged |

## Limitations

 1. **Bearing is not physically realistic**: `getEmitterDirection()` gives the exact vector between the two robots as computed by the physics engine, so we consider this simply an approximation to a rotating antenna
 2. We assume the emitter's original strength (fits military use case)
 3. **Avoidance is suppressed near the beacon**: within `CAMERA_DIST`, obstacle avoidance is disabled because the beacon's body registers as an obstacle and would cause the robot to veer away. This means the robot approaches the last 0.8 m blind to other obstacles.


# Iteration 4 - Bidirectional Dijkstra

 - faz aqui a explicação João 

# Iteration 5 - RescueBot Swarm 


## Limitations

 1. Not tested for when 2 pings are coming from the same space (robot collision and mutual harm)
 2. Each robot is only mapped to one channel (maybe after a robot reaches its destination it can try and chase another ping, the one it's closest to, and lock in on that channel after a channel sweep)
 3. Efficiency issues might happen: no Dijkstra or mechanism to avoid local minima


# How to Run

 1. Choose controller method in `arena.wbt`
 2. Enable console in $\text{Tools}$
 3. Enable $\text{Overlays} \rightarrow \text{'scout' overlays} \rightarrow \text{camera}$
 4. Add `arena.wbt` to webots
 5. Will run automatically
 6. Console outputs information like ping received
