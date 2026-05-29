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
| Floor | `Floor 10 × 10 m` | Large enough that the robot can wander for minutes before reaching an edge, small enough that the LiDAR's 5 m range covers most of it. |
| Walls | Three free-standing `Solid { Box }` blocks, 0.3 m tall | Cheaper to author than a closed maze, and copy-pasting one block is the simplest possible "add a wall" UX. Height of 0.3 m is well above the LiDAR (which sits at ~0.08 m), so the sensor always sees them. |
| Wall `boundingObject` | Duplicates the visual `Box` size | Required for actual physical collision — without it, the robot would pass straight through. |

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
| `Emitter.channel` | 1 | Matches the scout's receiver. |
| `Emitter.range` | -1 | Infinite range. Iteration 1 doesn't care about distance attenuation; we want the ping to always arrive so we can test the receive path. (Later iterations can shorten this to simulate a real radio.) |
| Payload | `"SOS"` (UTF-8 bytes) | Smallest meaningful message. The scout's signal-strength reading and `getEmitterDirection()` vector carry the useful information; the payload is just a label. |
| `PING_INTERVAL` | 3.0 s | Slow enough that console output is readable, fast enough to see multiple pings during a short demo. |



## Limitations 

 1. Not chasing ping
 2. Stuttery movement


# Iteration 2: Chasing Ping

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
| `GPS` + `Compass` on scout | onboard | Need position + heading to map and to steer toward a target. |
| `Display` "map_display" 200×200 | turret slot | Live view of the occupancy map being built. |
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
| `Emitter.channel` | 1 | Matches the scout's receiver|


## Limitations

 1. Reaching ping is trivial
 2. Only 1 distress signal



# How to Run

 1. Add `arena.wbt` to webots
 2. Will run automatically
 3. Console outputs information like ping received
