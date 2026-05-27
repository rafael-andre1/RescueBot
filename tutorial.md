**Setup (one time, ~2 min)**

1. Download & install Webots: https://cyberbotics.com/#download
2. Open Webots → `Wizards → New Project Directory…` → pick a folder, name it `my_project`. This auto-creates `worlds/` and `controllers/`.
3. `Wizards → New Robot Controller…` → Language: **Python** → Name: `robot_controller`. Repeat for `pinger_controller`.
4. Replace the contents of each generated file with the code blocks from my previous message:
   - `worlds/arena.wbt` ← world file
   - `controllers/robot_controller/robot_controller.py` ← scout controller
   - `controllers/pinger_controller/pinger_controller.py` ← pinger controller
5. In Webots: `File → Open World…` → pick `worlds/arena.wbt`.
6. Press the ▶ play button (top toolbar).

**Verify it works**

- Scout (blue box) drives forward, turns when it nears a wall.
- Console (bottom panel) prints `[pinger] sent HELP` and `[scout] PING 'HELP' …` every 3 seconds.
- Right-click the scout → `Show Camera Overlay` to see the camera feed.
- `View → Optional Rendering → Show Lidar Ray Paths` to visualize the LiDAR.

---

**What each file does**

`worlds/arena.wbt` — the scene. Defines:

- `WorldInfo` + `Viewpoint` + lighting — boilerplate, sets coordinates to ENU (Z is up) and where the camera sits.
- `Floor` — the flat 10×10 m surface.
- Three `Solid { … Box … }` blocks — the walls. Each one is positioned by `translation x y z` (z = half its height so it rests on the floor), and its physical shape is duplicated in `boundingObject` so things actually collide with it. **To add a wall:** copy a block, rename it, change `translation` and both `size` lines.
- `Robot { name "scout" … }` — the rover. Its `children` list is the device assembly: a Box body, a yellow front-marker, a 360° `Lidar` on top, a forward-facing `Camera`, a `Receiver` on channel 1, and two `HingeJoint`s driving `RotationalMotor`s named `left wheel motor` / `right wheel motor`. The `controller "robot_controller"` line tells Webots which Python folder to run for this robot.
- `Robot { name "pinger" … }` — a stationary red cube with an `Emitter` on channel 1 and the `pinger_controller` attached.

`controllers/robot_controller/robot_controller.py` — the scout's brain. Each simulation step it:

1. **Drains pings** — pulls any packet out of the `receiver` queue and prints the text + signal strength + the direction unit vector from `getEmitterDirection()` (this is the value we'll use in Part 2 to steer toward the pinger).
2. **Reads the LiDAR** — `lidar.getRangeImage()` returns 360 distances, one per ray. `front_min_distance()` looks at the front ±30° arc.
3. **Picks an action** — if anything in that arc is closer than `SAFE_DISTANCE` (0.4 m), it sets the wheel motors to opposite speeds (spin in place left); otherwise both wheels drive forward at `CRUISE_SPEED`.

The camera is enabled but unused for now — that just keeps the feed warm so we can read from it in later parts.

`controllers/pinger_controller/pinger_controller.py` — the help-request beacon. Every `PING_INTERVAL` seconds (3 s) it calls `emitter.send(b"HELP")` on channel 1. Because the scout's `Receiver` is on the same channel, the message arrives there.

**Knobs you might tweak**

- `SAFE_DISTANCE` in `robot_controller.py` — how close to a wall before the scout turns.
- `CRUISE_SPEED` / `TURN_SPEED` — how fast it moves / spins (motor `maxVelocity` is 10 in the world file, so don't exceed that).
- `PING_INTERVAL` in `pinger_controller.py` — ping cadence.
- Pinger location — `translation 3 -2 0.05` in the world file; move it anywhere on the floor.

