# Lab 4 – Navigation and 2D SLAM (TurtleBot3) – Study Notes

> ⚠️ **No lab handout is in the workspace.** These notes are reconstructed from the Simulink models in `Lab4_simulink.zip` (`Lab4_Task_1.slx` to `Lab4_Task_4.slx`), the course plan on Lecture 1 slide 5 ("LAB 4 – 2D SLAM"), the Lab 2 handout (whose Task 4 and Task 5 lead straight into this lab) and the Lecture 7 and 10 notes. Get the official task list from Canvas and check it against this. The four task names below are *inferred*; the methods are the ones the lectures teach for exactly this hardware.

## What this lab is about, and why it matters

Lab 2 taught the TurtleBot to go where you point it. Lab 4 asks the harder questions from Lecture 1: *what is around me, and how do I get from A to B without hitting it?* You will turn the LiDAR's ring of range readings into a map of the room, use that map (or the live scan) to steer around obstacles, plan a path on the map, and follow it. Every method you need already exists in the lecture notes; this lab is where you wire them together on a real robot, which is the course's definition of "2D SLAM".

## Roadmap: the main topics

1. What is in the folder and what the Simulink template tells you.
2. The four likely tasks and the toolkit that maps onto them.
3. Recipe A: registering scans into the world frame (the Lab 2 Task 4 hint, done properly).
4. Recipe B: building an occupancy grid (resolution, ray casting, cell updates).
5. Recipe C: reactive obstacle avoidance on live LiDAR (Lindqvist APF, Bug behaviour).
6. Recipe D: planning on the grid (A*, D*, PRM, RRT) and following the path with the Lab 2 controllers.
7. Recipe E (optional): correcting odometry drift with scan matching (ICP).
8. Plots and report checklist.
9. Debugging checklist and examiner questions.

---

## 1. What is in the folder

`Lab4_simulink.zip` unpacks to four models, `Lab4_Task_1.slx` to `Lab4_Task_4.slx`. All four are **byte-identical** to each other and to the Lab 2 template. Each one contains:

- a `ros2lib/Subscribe` on **`/odom`** (the robot's wheel-odometry pose, a quaternion for heading),
- a `ros2lib/Subscribe` on **`/scan`** (the LiDAR `LaserScan` message),
- a **Proportional Controller** subsystem with inports *Desired position, Position X, Position Y, Orientation (theta)* and outports *Linear Velocity (v), Angular Velocity (w)*,
- a **STOP slider switch** that forces both outputs to zero,
- a **Command Velocity Publisher** that packs $v$ and $\omega$ into a `Twist` on **`/cmd_vel`**.

So three facts are certain even without the handout: the lab runs on the **TurtleBot3 Burger** over **ROS 2** (set the Domain ID to 3X exactly as in Lab 2 and the TurtleBot notes), the raw material is **odometry plus LiDAR**, and the output is still just **$(v, \omega)$**. Everything you add sits between the `/scan` subscriber and the controller.

`Report Template.docx` is the same one-and-a-half-page template as the other labs.

**Everyday analogy:** Lab 2 gave you a car with a steering wheel and a speedometer. Lab 4 adds headlights and a notepad: you look at what the lights show, draw it on the notepad, and use the drawing to pick your route.

---

## 2. The four likely tasks and the toolkit

> ⚠️ Inferred. The template has four models and the course plan says "2D SLAM", so the natural progression is map → avoid → plan → close the loop.

| Likely task | What you build | Lecture it comes from |
|---|---|---|
| Task 1: Map the room | register every scan into the world frame with the odometry pose, then rasterise into an **occupancy grid** | Lecture 10 §12 (`scanmap`, `plot_occgrid`), Lab 2 Task 4 |
| Task 2: Reactive avoidance | drive to a waypoint while live LiDAR points push you away: the **Lindqvist APF**, or a **Bug** wall-follow | Lecture 7 Part I §8–9 |
| Task 3: Plan and follow | run **A\*/D\*/PRM/RRT** on your grid, then follow the waypoints with the Lab 2 point controller | Lecture 7 Part II, Lecture 4 §8 |
| Task 4: Close the loop | reduce odometry drift by **scan matching (ICP)** between consecutive scans, or revisit a start point and measure the error | Lecture 10 §11 |

Whatever the real wording, these four recipes are the toolbox you will draw from.

---

## 3. Recipe A: registering scans into the world frame

This is Lab 2 Task 4(b) done carefully, and it is the input to everything else.

A `LaserScan` message gives you `ranges` (an array of $N$ distances) plus `angle_min` and `angle_increment`. Reading $i$ was taken at bearing

$$ \beta_i = \text{angle\_min} + i \cdot \text{angle\_increment} $$

in the **LiDAR frame**, which on the Burger points the same way as the robot (the sensor sits centred on top). ⚠️ If your plots are mirrored or rotated by a fixed angle, the LiDAR frame is offset from the base frame; check the `/scan` header `frame_id` and the robot's URDF or ask the assistant.

**Step 1, polar to Cartesian in the robot frame** (Lecture 1):

$$ {}^B p_i = (r_i \cos\beta_i,\ r_i \sin\beta_i). $$

Drop readings with $r_i = 0$, $r_i = \infty$ or NaN, and anything beyond the sensor's trustworthy range (Lab 2 Task 4(a) had you measure that; the LDS-01 is usable to roughly 3.5 m).

**Step 2, robot frame to world frame** with the odometry pose $(x, y, \theta)$ at the moment of the scan (heading from the quaternion, see the TurtleBot notes):

$$ {}^W p_i = \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix} {}^B p_i + \begin{pmatrix} x \\ y \end{pmatrix}. $$

This is ${}^W p = {}^W\xi_B \cdot {}^B p$, Lecture 1 slide 16, in two lines of code. In MATLAB: `pW = (rot2(theta) * pB')' + [x y]`, or `homtrans(SE2(x,y,theta).T, pB')`.

**Worked example.** Robot at $(2, 1)$, heading $\theta = 90°$. Reading at bearing $\beta = 0$ (dead ahead) with $r = 1.5$ m gives ${}^B p = (1.5, 0)$. Rotate by 90°: $(1.5\cos 90° - 0,\ 1.5 \sin 90° + 0) = (0, 1.5)$. Add the position: ${}^W p = (2, 2.5)$. The robot faces +y, so the wall 1.5 m ahead is indeed at $y = 2.5$.

**Step 3, time alignment.** Use the odometry message closest in time to the scan's header stamp. The Lab 2 question "why does turning distort the points?" has the answer here: one sweep of the LDS-01 takes about 0.2 s, so at $\omega = 1$ rad/s the first and last rays differ by over 11° of heading but are stamped with one pose. **Mitigation:** register only when $|\omega|$ is below a small threshold, or interpolate $\theta$ across the sweep ($\theta_i = \theta + \omega \cdot i \cdot \Delta t_{ray}$).

**Common mistake:** using the *current* odometry when you process a scan that arrived a tenth of a second ago. At 0.2 m/s that is 2 cm of smear per scan, and it accumulates into fuzzy walls.

---

## 4. Recipe B: building an occupancy grid

An **occupancy grid** (Lecture 7 Part II, Lecture 10 §12) is a picture of the floor divided into square cells, each holding "is this cell blocked?".

### 4.1 Choose the resolution
Pick a cell size $c$. Smaller is more detailed but slower to plan on. With a robot 0.18 m wide and a room of about 8 m by 6 m, $c = 0.05$ m gives a $160 \times 120$ grid: fine enough to show door frames, small enough for A* in a fraction of a second. $c = 0.1$ m is a good first try.

Cell indices from a world point: $i = \lfloor (x - x_{min}) / c \rfloor + 1$, $j = \lfloor (y - y_{min}) / c \rfloor + 1$ (MATLAB is 1-indexed). Fix $x_{min}, y_{min}$ once, generously, before you start.

### 4.2 Mark hits and frees
Each registered ray tells you two things: the cell at the **end point** is occupied, and every cell **along the ray** from the robot to that point is free (the laser went through it).

- **Hit cell:** convert ${}^W p_i$ to $(i, j)$ and increment a "hit" counter.
- **Free cells:** walk from the robot's cell to the hit cell with **Bresenham's line algorithm** (the toolbox has `bresenham`) and increment a "miss" counter in each cell passed.

### 4.3 Update rule
Simplest: `occupied = hits > misses`, or a probability $p = (\text{hits}+1)/(\text{hits}+\text{misses}+2)$ and threshold at 0.5. The standard form is **log-odds**: keep $l_{ij}$, add $+0.9$ for a hit and $-0.4$ for a miss, clamp to $[-5, 5]$, and read occupancy as $p = 1/(1 + e^{-l})$.

**Worked example.** A cell is hit 3 times and seen through once: $l = 3(0.9) - 0.4 = 2.3$, $p = 1/(1 + e^{-2.3}) = 0.91$, occupied. A cell seen through 5 times and hit once (a passing person): $l = 0.9 - 2.0 = -1.1$, $p = 0.25$, free. The person did not leave a permanent blob, which is the point of counting both.

### 4.4 Make it usable for planning
Lecture 7 Part II planners want a binary map with 1 = blocked. Threshold at $p > 0.5$, then **inflate** obstacles by the robot's radius (about 0.1 m, so 1 to 2 cells at $c = 0.1$ m) with `imdilate` or a loop, so a path for a point robot is safe for a real one. Unknown cells (never observed) can be treated as blocked for safety or free for exploration; say which you chose in the report.

**Everyday analogy:** colouring in graph paper. Every laser hit is a pencil dot; every laser path is an eraser swipe. After a few hundred scans the walls are dark and the floor is clean.

---

## 5. Recipe C: reactive avoidance on live LiDAR

For the "drive to a goal but don't hit anything" task you do not need the map at all; the live scan is enough. Lecture 7 Part I §8 describes an APF built for exactly this sensor.

### 5.1 Lindqvist APF, step by step (per control cycle)
1. Take the current scan in the **robot frame** (Recipe A step 1 only, no world transform).
2. Keep points with $\|\rho_i\| < r_F$. Choose $r_F$ about 0.6 to 1.0 m for the Burger.
3. Repulsive force (Lecture 7 Part I §8):
   $$ F^r = \sum_i L^r \left(1 - \frac{\|\rho_i\|}{r_F}\right)^2 \frac{-\rho_i}{\|\rho_i\|}. $$
   Start with $L^r$ around 0.05 to 0.1 per point, because the LDS-01 returns hundreds of points per sweep and they add up. A common refinement is to normalise by the number of points inside $r_F$ so a long wall does not dominate.
4. Attractive part: distance $d$ and bearing $\theta_{ref}$ to the waypoint, in the world frame from odometry.
5. Control law for a ground robot (slide 20):
   $$ u_v = K_d\, d + F^r_x, \qquad u_\omega = K_\omega(\theta_{ref} - \theta) + F^r_y\,\text{sign}(u_v). $$
   Saturate $u_v$ to the Burger's 0.22 m/s and $u_\omega$ to about 1.5 rad/s.

**Worked example.** Waypoint 2 m ahead ($d = 2$, heading error 0), $K_d = 0.3$, $K_\omega = 1.5$. A box edge produces 20 points at about 0.4 m, slightly to the right of the nose (bearing $-20°$), with $r_F = 0.8$, $L^r = 0.05$. Weight per point: $(1 - 0.4/0.8)^2 = 0.25$. Unit vector away from a point at bearing $-20°$ is $(-\cos 20°, +\sin 20°) = (-0.94, 0.34)$. Summed force: $20 \times 0.05 \times 0.25 \times (-0.94, 0.34) = (-0.235, 0.085)$. Then $u_v = 0.6 - 0.235 = 0.365 \to$ saturated to 0.22 m/s (slow down a little), and $u_\omega = 0 + 0.085 \cdot 1 = 0.085$ rad/s: a gentle turn to the **left**, away from the box on the right. Exactly the behaviour you want.

**Common mistake:** forgetting that $F^r$ is in the robot frame while $\theta_{ref}$ is in the world frame. In the slide-20 law that is fine, because $F^r_x$ and $F^r_y$ are used as forward and sideways *corrections*, not as world directions. If instead you use the waypoint-shifting version (slide 19), you must rotate $F^r$ into the world frame with `rot2(theta)` before adding it to the waypoint.

### 5.2 Bug-style wall following
Lab 2 Task 5 already had you keep a distance from a wall. A **Bug2** behaviour (Lecture 7 Part I §9) combines that with "head to goal": drive the m-line towards the goal; when the front sector of the scan shows a range below a threshold, switch to wall-following until you are back on the m-line and closer to the goal than where you left it. Implement it as a two-state machine in a MATLAB Function block. Bug2 is complete (it finds a path if one exists), which the APF is not: the APF can get stuck in a U-shaped local minimum, which is a good thing to demonstrate and discuss in the report.

---

## 6. Recipe D: plan on the grid, then follow the path

### 6.1 Plan
Take the inflated binary grid from Recipe B and any planner from Lecture 7 Part II:

```matlab
ds = Dstar(map);          % or PRM(map), or DXform(map)
ds.plan(goalCell);        % D* plans backwards from the goal
p  = ds.query(startCell); % returns an N×2 list of cells
```

Remember the order: **plan, then query**. Convert the start and goal from metres to cells with the formula in §4.1, and convert the returned cells back to metres with $x = x_{min} + (i - 0.5)c$. D* is the natural choice if you will update the map while driving, because `modify_cost` plus a re-plan is cheap (Lecture 7 Part II §8). PRM or RRT is the choice if you want to discuss sampling-based planning.

### 6.2 Thin the path
A grid path has a waypoint every cell. Keep every $k$-th point, or drop points that lie on a straight line between their neighbours, so the controller has a few metres between waypoints.

### 6.3 Follow with the Lab 2 controller
Feed the waypoints one at a time to the **move-to-a-point** controller from Lecture 4 §8 (the Proportional Controller subsystem already in the template). Switch to the next waypoint when the distance drops below a **switch radius**, about 0.15 to 0.25 m; waiting for zero distance makes the robot dither at each corner. At the final waypoint, use move-to-a-pose if a heading is required.

**Worked example.** Grid $c = 0.1$ m, $x_{min} = y_{min} = -2$. Start at world $(0, 0)$ is cell $(21, 21)$; goal $(3, 1.5)$ is cell $(51, 36)$. D* returns 40 cells; keeping every fifth gives 8 waypoints. With $K_v = 0.5$ and a switch radius of 0.2 m the robot reaches each within a couple of seconds and never fully stops until the goal.

### 6.4 Re-plan when the world changes
If a new scan marks cells on your path as occupied (someone put a box down), raise their cost with `modify_cost` and call `plan` again; keep the current waypoint index. This is the "2-layer architecture" of Lecture 7's last slide: global planner on the map, local controller (or APF from Recipe C) for the last metre.

---

## 7. Recipe E (optional): correcting drift with scan matching

Odometry drifts (Lecture 10 §2–4), so after a few laps your map's walls appear twice, slightly shifted. **Iterative Closest Point (ICP)**, Lecture 10 §11, estimates the rigid transform between two consecutive point clouds:

```matlab
T = icp(pPrev', pCurr', 'T0', transl2(dx_odom, dy_odom), 'distthresh', 0.3);
```

Seed it with the odometry increment, apply the small correction it returns to your pose estimate, and register the scan with the corrected pose. Even this one-step "laser odometry" sharpens the map noticeably. If you also return to the start and compare the first and last scans, the difference is a **loop-closure** error you can report, which is the first step towards the pose-graph SLAM of Lecture 10 §10.

> ⚠️ `icp` lives in Peter Corke's Machine Vision Toolbox, not the Robotics Toolbox; check it is installed on the lab PC, or use MATLAB's `matchScans` / `pcregistericp` from the Navigation and Computer Vision toolboxes.

---

## 8. Plots and report checklist

- **Map:** the occupancy grid as an image with equal axes, the robot's odometry path drawn over it, and the raw registered point cloud as a comparison.
- **Avoidance:** XY plot of the path with the obstacle points, the goal and, for the APF, arrows of $F^r$ at a few instants; time plots of $v$ and $\omega$ with saturation visible.
- **Planning:** the grid with the planned path, the thinned waypoints and the executed path; a table of planner, planning time, path length.
- **Drift:** a before/after ICP map, or the start-to-end position error after a loop.
- **Discussion:** resolution chosen and why; how unknown cells were treated; a failure case (APF local minimum, odometry drift) and what you did about it.
- Code and a video in the submission zip, as the template demands.

---

## 9. Debugging checklist and examiner questions

| Symptom | Likely cause | Fix |
|---|---|---|
| Walls appear curved or doubled | pose/scan time mismatch, registering while turning | Recipe A step 3 |
| Map is mirrored or rotated | LiDAR frame offset or angle sign | check `angle_min`, `frame_id` |
| Grid is all occupied | no free-space ray casting, or threshold too low | Recipe B §4.2 |
| Robot oscillates near obstacles | $L^r$ too large, or $r_F$ too large so far points push | lower gain, normalise by point count |
| Robot stops in a U-shaped bay | APF local minimum | switch to Bug behaviour or re-plan on the map |
| Planner returns empty path | start or goal inside an inflated obstacle, or `query` before `plan` | check cells, order of calls |
| Robot overshoots corners | switch radius too small or $K_v$ too high | Recipe D §6.3 |
| Nothing moves | teleop still running, wrong Domain ID, STOP slider on | TurtleBot notes §11 |

**Examiner questions to be ready for.** What is the difference between an occupancy grid and a point cloud? Why do you cast rays for free space instead of only marking hits? Why does turning distort the scan, and what did you do about it? Is the APF complete, and what does complete mean? Why inflate obstacles before planning? Why must you call `plan` before `query`, and what does D* let you do that A* does not? Where does the drift in your map come from, and how would full SLAM fix it?

---

## In a nutshell

Lab 4 turns the TurtleBot's LiDAR and odometry into navigation. You convert each scan from polar readings to points in the robot frame, move them into the world frame with the odometry pose, and paint them into an occupancy grid by marking hit cells and ray-casting free cells. On the live scan, a LiDAR-based artificial potential field (or a Bug wall-follow) pushes the robot away from nearby points while it heads for a waypoint. On the finished grid, a planner such as D*, PRM or RRT returns a cell path that you thin into waypoints and follow with the Lab 2 move-to-a-point controller, switching waypoints inside a small radius. Odometry drift blurs the map, and scan matching with ICP is the first tool for correcting it. The handout itself is not in the workspace, so match these recipes to the task list on Canvas.

## Key terms

- **Occupancy grid:** a map of square cells, each marked free, occupied or unknown.
- **Resolution (cell size $c$):** metres per cell; trades detail against planning speed.
- **Registration:** placing a scan's points in the world frame using the robot's pose at scan time.
- **LaserScan:** the ROS message with `ranges`, `angle_min` and `angle_increment`.
- **Ray casting / Bresenham:** stepping through the cells a laser beam crossed to mark them free.
- **Log-odds update:** adding a fixed amount per hit or miss to a cell's log-odds and thresholding.
- **Inflation:** growing obstacles by the robot's radius so a point-robot path is safe.
- **Artificial potential field (APF):** goal attracts, obstacle points repel; summed into a steering command.
- **Lindqvist APF:** the LiDAR-point version with weight $(1 - \|\rho\|/r_F)^2$ per point.
- **Local minimum:** a spot where attraction and repulsion cancel and the APF stalls.
- **Bug2:** head along the m-line, follow walls when blocked, leave when back on the m-line and closer.
- **Completeness:** a planner finds a path if one exists, or reports failure, in finite time.
- **D\*:** dynamic A\* that plans from the goal and re-plans cheaply after cost changes.
- **Switch radius:** distance at which the follower moves on to the next waypoint.
- **Two-layer architecture:** global planner on the map plus local reactive controller.
- **ICP (iterative closest point):** finds the rigid transform aligning two point clouds; laser odometry.
- **Loop closure:** recognising a previously visited place and correcting accumulated drift.

## Check yourself

1. Robot at $(1, 2)$ with heading $180°$ sees a return at bearing $+90°$ (its left) at 2 m. Where is the point in the world frame?
2. Why do you cast rays to mark free cells rather than only marking the cells where the laser hit?
3. Give one reason to pick an APF over Bug2 for obstacle avoidance, and one reason to pick Bug2 over an APF.
4. You call `ds.query(start)` and get an empty path even though the room is open. Name two likely causes.
5. After three laps of the room the walls in your map appear twice, 20 cm apart. What is happening and which recipe addresses it?

<details>
<summary>Answers</summary>

1. Robot-frame point: $(2\cos 90°, 2\sin 90°) = (0, 2)$. Rotate by $180°$: $(0\cdot(-1) - 2\cdot 0,\ 0\cdot 0 + 2\cdot(-1)) = (0, -2)$. Add position: $(1, 0)$. Facing $-x$, its left is $-y$, so a point 2 m to the left is at $y = 0$.
2. Hits alone never erase anything, so a person walking through the room or a spurious return leaves a permanent obstacle. Marking the cells the beam passed through as free lets later evidence out-vote old hits and leaves the floor clean.
3. APF: smooth, fast, needs no map and no state machine, handles many obstacles at once. Bug2: it is complete, so it cannot get stuck in a U-shaped local minimum the way an APF can.
4. The start or goal cell lies inside an inflated obstacle (or outside the grid), or `query` was called before `plan`, or the start and goal were given in metres instead of cell indices.
5. Odometry drift: wheel slip and heading error accumulate, so later scans are registered with a wrong pose. Recipe E, scan matching with ICP to correct the pose increment, and ultimately loop closure or SLAM, addresses it.

</details>
