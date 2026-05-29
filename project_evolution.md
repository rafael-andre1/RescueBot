# Iteration 1

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

Very simple differential drive controller.

It works by...

Motivation for this was...


### Controller

Each simulation step (every 32 ms) it does three things in order:

1. **Drain the receiver queue.** Every waiting packet is printed with its payload, signal strength, and the unit vector from `getEmitterDirection()`. Crucially, that direction vector is already in the *robot's own frame* — exactly what a pathfinding routine needs as input.
2. **Read the LiDAR.** `getRangeImage()` returns 360 distances. The controller looks at only the forward ±30° arc (`FRONT_ARC_DEG = 60`), since side rays are irrelevant for forward collision avoidance.
3. **Decide.** If the minimum distance in the forward arc is below `SAFE_DISTANCE`, set the motors to opposite velocities (spin in place left). Otherwise drive both wheels forward at `CRUISE_SPEED`. This is the classic "Braitenberg-lite" reactive strategy — no map, no memory, no goal.

#### Parameter Reasoning

| Parameter | Value | Reasoning |
|---|---|---|
| `TIME_STEP` | 32 ms | Must equal `basicTimeStep` (or be a multiple). |
| `SAFE_DISTANCE` | 0.4 m | About 2× the robot's body length, so the robot starts turning before it would actually collide. Small enough to navigate gaps between walls. |
| `FRONT_ARC_DEG` | 60° (±30°) | Wide enough to catch obliquely-approaching walls, narrow enough that a wall on the side doesn't trigger a useless spin. |
| `CRUISE_SPEED` | 3.0 rad/s (~0.12 m/s) | Walking pace; gives the LiDAR several steps to detect a wall before impact at the configured `SAFE_DISTANCE`. |
| `TURN_SPEED` | 2.0 rad/s | Slower than cruise so the spin is controlled; opposite signs on the two motors → pure rotation about the robot center. |


## Distress Beacon

The simplest possible Webots actor: a static red cube with one device.

| Parameter | Value | Reasoning |
|---|---|---|
| `Emitter.channel` | 1 | Matches the scout's receiver. |
| `Emitter.range` | -1 | Infinite range. Iteration 1 doesn't care about distance attenuation; we want the ping to always arrive so we can test the receive path. (Later iterations can shorten this to simulate a real radio.) |
| Payload | `"SOS"` (UTF-8 bytes) | Smallest meaningful message. The scout's signal-strength reading and `getEmitterDirection()` vector carry the useful information; the payload is just a label. |
| `PING_INTERVAL` | 3.0 s | Slow enough that console output is readable, fast enough to see multiple pings during a short demo. |







# How to Run

 1. Add `arena.wbt` to webots
 2. Will run automatically
 3. Console outputs information like ping received
