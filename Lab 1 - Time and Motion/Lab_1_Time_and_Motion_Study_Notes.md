# Lab 1 – Time and Motion: Study Notes

> Built from the handout `R7010_lab1.pdf` (7 pages), the supplementary code in `lab1.zip` (folders `Task2`, `Task3`, `Task4`) and the `Report Template.docx`. The theory behind every task is in the Lecture 3 notes (`Lecture3/Lecture_3_Time_and_Motion_Study_Notes.md`) and the pose algebra is in the Lecture 1 notes (`Lecture1/Lecture_1_Position_and_Orientation_Study_Notes.md`). These lab notes repeat just enough of that theory for you to do the tasks, and they point you to the lecture notes for the full derivations.

## What this lab is about, and why it matters

Lab 1 takes the trajectory tools from Lecture 3 (quintic polynomials, trapezoidal profiles, multi-segment via-point trajectories, pose interpolation) and makes you use them on four very different robots: a floating 3D object, a ground robot in a maze, a six-joint PUMA 560 arm drawing on a wall, and a small drone flying through and around structures. The common thread is one question: *how do I turn a list of places I want to visit into a smooth, timed motion?* Everything you do later with the real TurtleBot (Lab 2) and Crazyflie (Lab 3) assumes you can answer it.

## Roadmap: the main topics

1. What is in `lab1.zip` and how the lab is marked (the report template).
2. The toolbox tools you will lean on, in one page each: `tpoly`, `lspb`, `mtraj`, `mstraj`, `ctraj`/`interp`, `SE3`, roll-pitch-yaw and quaternions.
3. Task 1: Cartesian motion of a 3D object through poses A, B, C, D.
4. Task 2: a ground robot's via-point trajectory through a maze, with a door that blocks the way for 5 seconds.
5. Task 3: the PUMA 560 drawing letters on a vertical wall and on an inclined wall.
6. Task 4: paths for a micro aerial vehicle (MAV) through two obstacle courses, plus inspection flights around a fountain and a wind turbine.
7. The optional exercises from Chapter 3 of the book (7, 8, 9 and 13), with worked numbers.
8. Report checklist and the questions the examiner is likely to ask.

---

## 1. What you are given

### 1.1 The zip file

The handout says the folder names match the task numbers. There is no `Task1` folder because Task 1 needs nothing but the toolbox.

| Folder | File | What it is |
|---|---|---|
| `Task2` | `run.m` | Loads `map1`, plots it, defines start and goal, defines a via-point `path`, builds a trajectory with `mstraj`, animates it. **This is where your path goes.** |
| `Task2` | `animatepath.p` | Animates the robot on the map. The third argument `obstacle` (0 or 1) switches the door on. |
| `Task3` | `run_vertical.m`, `run_inclined.m` | Load the PUMA 560, define the wall plane, load an example trajectory, convert it to end-effector poses, solve inverse kinematics and animate. **You replace the "load example" part with your own trajectory.** |
| `Task3` | `Wall.p`, `plot_qtraj.p` | Draw the wall; animate the arm and draw the pen's trace where the pen touches the wall. |
| `Task3` | `q_vertical.mat`, `q_inclined.mat` | Example trajectories (a variable `q`, one row per time step, columns x, y, z in the frame used by the scripts). |
| `Task4` | `run.m`, `load_map.p`, `plot_path.p`, `points_to_idx.p`, `maps/map1.txt`, `maps/map2.txt` | Load and draw a 3D box world from a text file. |
| `Task4` | `fountain.m`, `fountain.ply` | Load and show the LTU fountain as a 3D point cloud. |
| `Task4` | `windturbine.m`, `Points10.mat` | Load and plot the wind-turbine point cloud (coordinates divided by 10). |

⚠️ The `.p` files are MATLAB **p-code**, which is obfuscated and cannot be read. Everything these notes say about `Wall`, `plot_qtraj`, `animatepath`, `load_map`, `plot_path` and `points_to_idx` is inferred from how `run.m` calls them. If one of them behaves differently from what is described here, trust the behaviour you see and ask the lab assistant.

### 1.2 How you are marked

The `Report Template.docx` is the same for all four labs. It asks for a compact report of about 1.5 pages of text plus figures, with three sections: **Objective** (the tasks in your own words, and the difficulties you expected), **Methodology** (the method, with equations and code snippets) and **Results** (plots, what was actually hard, what you learned). You submit the report, the full code and recorded videos as **one zip named with your name or group number**. Section 8 turns this into a checklist.

---

## 2. The tools you will use (short version of Lecture 3)

You met all of these in Lecture 3. Here is the minimum you need for the lab, written as "what it does, what you give it, what you get back".

### 2.1 `tpoly`: one smooth move with a quintic polynomial

A **quintic polynomial** is a fifth-order polynomial in time. It has six coefficients, so you can pin down six things: position, velocity and acceleration at the start and at the end. That is exactly what you need for a motion that starts and stops gently.

```matlab
[s, sd, sdd] = tpoly(0, 1, 50);          % 50 samples from 0 to 1, zero velocity at both ends
[s, sd, sdd] = tpoly(0, 1, 50, 0.5, 0);  % start with velocity 0.5 per step, end at rest
```

You give it start value, end value, number of steps (or a time vector), and optionally the start and end velocities. You get back the position `s`, velocity `sd` and acceleration `sdd` at each step. Velocity here is *per step*, not per second, unless you pass a time vector.

**Analogy:** a lift that starts gently, speeds up, and eases to a stop. The quintic is the mathematical shape of a gentle lift ride.

### 2.2 `lspb`: linear segment with parabolic blends (the trapezoid)

A trapezoidal velocity profile: accelerate (parabolic blend), cruise at constant speed (linear segment), decelerate. It is faster than the quintic for the same peak speed because it spends most of its time *at* the speed limit.

```matlab
[s, sd, sdd] = lspb(0, 1, 50);        % toolbox picks the cruise speed
[s, sd, sdd] = lspb(0, 1, 50, 0.025); % you set the cruise speed V (per step)
```

The cruise speed is not free. For distance $d$ in $T$ steps you need $d/T < V \le 2d/T$ (section 7 shows why). Pick $V$ outside that window and `lspb` throws an error.

### 2.3 `mtraj`: several axes at once, one segment

`mtraj(@tpoly, q0, qf, N)` or `mtraj(@lspb, q0, qf, N)` runs the chosen one-axis function on every element of the vectors `q0` and `qf` and stacks the results, so all axes start and finish together. Good for a single move between two configurations.

### 2.4 `mstraj`: many segments through via points (the workhorse of this lab)

This is the function you will use most. It takes a list of **via points** (places the path must visit) and produces one long trajectory that passes through them, with **rounded corners** so it never has to stop.

```matlab
traj = mstraj(VIA, QDMAX, TSEG, Q0, DT, TACC)
```

| Argument | Meaning | Typical choice in this lab |
|---|---|---|
| `VIA` | matrix of via points, one per row, one column per axis (**not** including the start) | `[20 30; 20 60; ...]` |
| `QDMAX` | maximum speed per axis (units per second) | `[1 1]` for Task 2 |
| `TSEG` | duration of each segment in seconds; use `[]` to let `QDMAX` decide | `[]` (or set it to control timing) |
| `Q0` | the starting configuration (a row) | `path(1,:)` |
| `DT` | sample time in seconds | `0.2` |
| `TACC` | acceleration (blend) time in seconds; `0` gives sharp corners, larger values give rounder corners | `1` to `2` |

Two rules from Lecture 3 you must remember:

- You give **either** `QDMAX` **or** `TSEG`, and pass `[]` for the other. With `QDMAX`, each segment's duration is set by the *slowest* axis, so all axes arrive together.
- `TACC = 0` means the velocity jumps at every corner, which is an **acceleration spike**. The handout's phrase "smooth (without acceleration jumps)" is telling you to use `TACC > 0`. The price is that the path **cuts the corner**: it no longer passes exactly through the via point. Plan clearance for that.

**Analogy:** a bus route. The via points are the stops you must pass near, `QDMAX` is the speed limit, and `TACC` is how wide the bus swings around each corner. A bus that stops dead at every corner (`TACC = 0`) is jerky; one that swings wide (`TACC` large) is smooth but needs a wider road.

### 2.5 Poses: `SE3`, roll-pitch-yaw, `ctraj` and `interp`

A **pose** is position plus orientation. In the toolbox an `SE3` object holds a 4×4 homogeneous transformation. You build one from a translation and an orientation:

```matlab
T = SE3(x, y, z) * SE3.rpy(roll, pitch, yaw);   % angles in radians, ZYX convention
T.t          % the translation as a column vector
T.torpy      % back to [roll pitch yaw]
```

The handout writes orientation as $(\phi, \theta, \psi)$. In Corke's convention these are **roll, pitch, yaw** applied in the ZYX order (yaw about $z$, then pitch about the new $y$, then roll about the new $x$). See Lecture 1 notes, section on Euler angles, and remember that **pitch $= \pm\pi/2$ is the singular configuration** of this representation (gimbal lock). That matters in Task 1.

To move smoothly between two poses in one segment:

```matlab
Ts = ctraj(T0, T1, 50);              % 50 poses, position and orientation both eased with lspb
Ts = T0.interp(T1, tpoly(0, 1, 50)); % same idea, quintic timing
Ts.animate                            % watch a frame fly
```

`ctraj` interpolates the translation linearly and the rotation by **spherical linear interpolation** (slerp) on unit quaternions, so the orientation stays a valid rotation at every step. Never interpolate the nine numbers of a rotation matrix directly (Lecture 3, slide 27): the result is not a rotation.

---

## 3. Task 1: Cartesian motion of a 3D object

### 3.1 What is asked

Move a 3D object, drawn as a 3-axis frame, from pose A to pose D via B and C. The givens, as $(x, y, z, \phi, \theta, \psi)$:

- A = $(0, 0, 0, 0, 0, 0)$
- B = $(0, 0, 1, 0, \pi/2, 0)$
- C = $(1, 0, 1, 0, -\pi, 0)$
- D = $(1, 0, -0.5, 0, \pi/2, 0)$

The ToDo says: *design the trajectory that smoothly moves the object from A to D; it must be smooth throughout, not just within segments, so it must not slow down at the corners (rounded corners).* Visualise the four poses as points, the path, and an animated frame following it.

### 3.2 What it tests

Multi-segment trajectories (`mstraj`) on a six-dimensional configuration vector, and the difference between interpolating positions and interpolating orientations. Lecture 3, sections on multi-dimensional and multi-segment trajectories and interpolation of orientation.

### 3.3 Read the poses before you code

Picture it. A is at the origin, upright. B is 1 m straight up, pitched 90° (nose down or up depending on sign). C is 1 m up and 1 m along $x$, rotated 180° in pitch (upside down). D is 1 m along $x$ but 0.5 m *below* the floor, pitched 90° again. So the path goes up, across, then down past the start height, while the object tumbles about its $y$ axis. Only $x$, $z$ and $\theta$ ever change; $y$, $\phi$ and $\psi$ stay zero.

### 3.4 Step-by-step approach

**Step 1. Put the poses in a matrix, one row each.**

```matlab
A = [0 0  0    0  0     0];
B = [0 0  1    0  pi/2  0];
C = [1 0  1    0 -pi    0];
D = [1 0 -0.5  0  pi/2  0];
via = [B; C; D];          % mstraj wants the via points without the start
```

**Step 2. Decide speeds and timing.** The handout gives none, so choose sensible ones and say so in the report. Positions in metres per second, angles in radians per second:

```matlab
qdmax = [0.5 0.5 0.5  1 1 1];   % 0.5 m/s translation, 1 rad/s rotation
dt    = 0.05;                   % 20 samples per second
tacc  = 0.5;                    % half a second of blending at each corner
traj  = mstraj(via, qdmax, [], A, dt, tacc);
```

What happens inside: segment A→B must move 1 m in $z$ (2 s at 0.5 m/s) and $\pi/2$ rad in $\theta$ (1.57 s at 1 rad/s); the slower one, 2 s, sets the segment time, and $\theta$ is slowed to arrive at the same moment. Segment B→C moves 1 m in $x$ (2 s) and $3\pi/2$ rad in pitch (4.7 s), so this segment takes 4.7 s and the translation crawls. Segment C→D moves 1.5 m in $z$ (3 s) and $3\pi/2$ rad in pitch (4.7 s). Total roughly $2 + 4.7 + 4.7 \approx 11.4$ s plus blends, so about 230 rows of six columns.

**Step 3. Turn each row into a pose and animate.**

```matlab
Ts = SE3(traj(:,1:3)) .* SE3.rpy(traj(:,4:6));  % element-wise product: N translations times N rotations
% if the element-wise form complains, build it in a loop:
for k = 1:size(traj,1)
    Ts(k) = SE3(traj(k,1:3)) * SE3.rpy(traj(k,4:6));
end
figure; Ts.animate('retain');            % frame flying along the path
hold on
plot3(traj(:,1), traj(:,2), traj(:,3), 'b')               % the path
P = [A; B; C; D];  plot3(P(:,1), P(:,2), P(:,3), 'ro', 'MarkerFaceColor', 'r')   % the poses as points
xlabel x; ylabel y; zlabel z; axis equal; grid on
```

Also plot the six columns of `traj` against time and, if you want to show smoothness, the numerical derivative `diff(traj)/dt` (velocity should have no jumps).

### 3.5 The orientation trap in this task

**Angle wrapping.** C has $\theta = -\pi$ and D has $\theta = +\pi/2$. `mstraj` just interpolates numbers, so it will sweep from $-\pi$ to $+\pi/2$, a swing of $3\pi/2$ (270°), when the *short* way round is $\pi/2$ (90°), because $-\pi$ and $+\pi$ are the same orientation. Likewise B→C sweeps $\pi/2 \to -\pi$, which is 270° the long way. Decide which you want. If you want the short way, replace $-\pi$ by $+\pi$ in C (then B→C is 90° and C→D is 90°). ⚠️ The handout does not say which direction of rotation it expects; both are "correct" poses. State your choice in the report.

**Gimbal lock.** B and D have pitch $\theta = \pi/2$, which is the singularity of roll-pitch-yaw. *Generating* a rotation matrix from $(\phi, \theta, \psi)$ with `SE3.rpy` is perfectly fine at the singularity, so the approach above works. The trouble only comes if you convert a rotation matrix *back* to angles with `torpy` near $\theta = \pi/2$: roll and yaw become ambiguous and the recovered angles can jump. So: interpolate the *angles*, convert to poses, and do not round-trip.

**Cleaner alternative.** Interpolate the position with `mstraj` on the $xyz$ columns only, and the orientation with unit quaternions:

```matlab
qA = UnitQuaternion.rpy(0,0,0);  qB = UnitQuaternion.rpy(0,pi/2,0); % etc.
qs = interp(qA, qB, 50, 'shortest');   % slerp, shortest arc
```

Slerp has no gimbal lock and `'shortest'` fixes the wrap problem automatically. The cost is that stitching segments without stopping is harder with `interp`, so the `mstraj`-on-angles approach is usually the pragmatic choice for this task.

**Common mistake:** using `ctraj` for each of the three segments and concatenating. Each `ctraj` segment starts and ends at zero velocity, so the object **stops at B and C**, which is exactly what the handout forbids.

---

## 4. Task 2: ground robot through a maze

### 4.1 What is asked

- Initial position $[50, 30]$, goal $[20, 5]$, on the 100×100 map `map1` drawn by `Task2/run.m`.
- Design a smooth multi-segment trajectory (no acceleration jumps) from start to goal with maximum velocity $[1, 1]$, with `obstacle = 0`.
- Increase the velocity bound and observe.
- Assume the door blocks the path for 5 s; generate a new path that passes the door with correct timing, with `obstacle = 1`.
- Plot the robot, the map and the smooth path.

### 4.2 What it tests

`mstraj` with via points, speed limits and blend time; reading a map; reasoning about *timing* as well as geometry (Lecture 3, multi-segment trajectories).

### 4.3 Read `run.m` first

```matlab
load map1; about map          % a 100x100 matrix: 1 = wall, 0 = free
plot(Bug2(map))               % borrows the Bug2 planner's plot method just to draw the map
p_init = [50 30];  p_goal = [20 5];
path = [p_init; 20 30; 20 60; 50 60; 50 90; 90 90; 90 10; 20 10; p_goal];
QDMAX = [1 1];  Q0 = path(1,:);  DT = 0.2;
traj = mstraj(path, QDMAX, [], Q0, DT, 0);
obstacle = 1;                 % 0: no door, 1: door present
animatepath(traj, DT, obstacle)
```

Three things to notice:

1. `path` includes the start as its first row and `Q0` is also the start, so `mstraj` begins at the start and the first "segment" has zero length. Harmless, but you can pass `path(2:end,:)` as the via points instead.
2. `TACC` is `0`. That is the sharp-cornered, acceleration-jump version. **You must raise it** to meet the "smooth" requirement.
3. ⚠️ The example `path` is the same list of points as the `mstraj` demo in the Lecture 3 script, and from the handout's Figure 2 it appears to run straight through the right-hand wall (the segment from $(50, 90)$ to $(90, 90)$ crosses the wall at $x \approx 63$, which reaches up to $y \approx 92$). Treat it as a placeholder that shows the syntax, not as a valid route.

### 4.4 Read the map precisely

The figure in the handout is a sketch. Get the exact wall cells from the matrix, because `mstraj` with blending will cut corners and you need to know how much room you have:

```matlab
[r, c] = find(map);        % occupied cells: r = y index (row), c = x index (column)
figure; imagesc(map); axis xy; axis equal; colormap(flipud(gray))
```

The toolbox convention is `map(y, x)`: rows are $y$, columns are $x$. From the handout's Figure 2a the walls are, roughly: a left wall at $x \approx 10$ from $y \approx 10$ to $92$; a horizontal wall at $y \approx 75$ from $x \approx 10$ to $36$; a horizontal wall at $y \approx 52$ from $x \approx 26$ to $68$; a right wall at $x \approx 63$ from $y \approx 10$ to $92$; a bottom wall at $y \approx 20$ from $x \approx 10$ to $78$. With `obstacle = 1` a **door** appears at $y \approx 50$ from $x \approx 68$ to $90$. ⚠️ These coordinates are read off a small figure; confirm them with `find(map)` before you plan.

**Where are you?** The start $(50, 30)$ sits in the lower room: between $y = 20$ and $y = 52$, between $x = 10$ and $x = 63$. The goal $(20, 5)$ is *below* the bottom wall. The only way out of the lower room is the gap at the top-left, where the $y = 52$ wall stops short of the left wall (between $x \approx 10$ and $26$). So the route is forced: up through that gap, around the end of the $y = 75$ wall, over the top of the right wall, down the right-hand corridor past the door, under the end of the bottom wall (which stops at $x \approx 78$), then left along the bottom to the goal.

### 4.5 Step-by-step approach

**Step 1. Choose via points with clearance.** Stay at least 3 or 4 cells from every wall, because the blend will shave the corners. A route in the spirit of the map (check every number against your own `find(map)`):

```matlab
via = [18 30;    % slide left along the lower room
       18 60;    % up through the gap at the left end of the y=52 wall
       45 60;    % right, under the y=75 wall's free end... (keep clear of the y=52 wall below)
       45 96;    % up over the top of the right wall
       75 96;    % across the top
       85 96;    % and a bit further so the next corner is clear of the wall end at x~63
       85  8;    % straight down the right corridor to below the bottom wall
       20  8;    % left along the bottom
       p_goal];  % finish at (20,5)
```

If a via point looks tight on your map, move it. The animation will show any clipping.

**Step 2. Build the trajectory with blending.**

```matlab
QDMAX = [1 1];  DT = 0.2;  TACC = 2;
traj = mstraj(via, QDMAX, [], p_init, DT, TACC);
```

Units: the map is in cells, so `QDMAX = [1 1]` is one cell per second per axis, and the diagonal speed can reach $\sqrt{2}$. `TACC = 2` blends each corner over 2 s, which at 1 cell/s cuts about 1 cell off the corner. The route above is roughly 30 + 30 + 36 + 40 + 88 + 65 + 3 ≈ 290 cells long, so expect about 290 s of motion and about 1450 rows.

**Step 3. Check for collisions numerically**, not just by eye:

```matlab
hit = any(arrayfun(@(k) map(round(traj(k,2)), round(traj(k,1))) == 1, 1:size(traj,1)));
```

`hit` must be `false`. Remember `map(y, x)`: row index first.

**Step 4. Animate and plot.** `animatepath(traj, DT, 0)` for the door-free case. Also plot `traj(:,1)` and `traj(:,2)` against time and their differences to show the velocities have no steps.

**Step 5. Increase the velocity bound** to, say, `[3 3]` and `[10 10]`. You should see: fewer rows (the whole trip is faster), the corners cut more deeply for the same `TACC` (because in 2 s at 3 cells/s the blend spans 6 cells), and eventually the path clipping walls. That is the observation the handout wants: speed and blend time trade against clearance. If you want the same corner rounding at higher speed, shrink `TACC` in proportion.

### 4.6 The door (`obstacle = 1`)

The handout says the door blocks the path for 5 s. The door sits across the right-hand corridor at $y \approx 50$, which your route passes on the way down. You have two legitimate strategies:

- **Timing.** Arrive at the door after it has cleared. At 1 cell/s you reach $y \approx 50$ in the corridor after about 200 s anyway, far longer than 5 s, so the plain route already passes "with correct timing". If you raised the speed so you arrive before 5 s, add time: pass a `TSEG` vector of segment durations instead of `QDMAX`, lengthening the segments before the door, or insert a waiting via point (repeat the point just above the door and give that zero-length segment a duration of 5 s via `TSEG`).
- **Geometry.** If the door does not reach the right boundary (it appears to span $x \approx 68$ to $90$, with the map going to 100), route the descent at $x \approx 95$ instead of $85$ and avoid it entirely. Whether that gap really exists depends on `animatepath`'s drawing of the door, which you cannot read; test it.

⚠️ Because `animatepath.p` is p-code, the exact moment the door appears and disappears is not documented. Run it once with `obstacle = 1` and watch, then plan.

**Common mistakes:** forgetting that `map` is indexed `(y, x)`; leaving `TACC = 0`; using the placeholder path; planning right against the walls and then being surprised that blending clips them.

---

## 5. Task 3: the PUMA 560 draws letters on a wall

### 5.1 What is asked

- Draw the first letter of your last name on a **vertical** wall, running `run_vertical.m`.
- Draw the first letter of your last name *and* the first letter of your first name **at different speeds** on the vertical wall.
- Draw the first letter of your last name on the **inclined** wall, running `run_inclined.m`.

Givens: the wall is the plane $ax + by + cz + d = 0$; vertical case $x = 1$ (the $y$–$z$ plane at $x = 1$); inclined case "$\cos(10°)\,x + \cos(10°)\,z = \cos(10°)$ (xy plane pitch 80°)"; the pen tip is at body-frame coordinate $(0, 0, 0.4)$ of the end-effector; if the tip does not touch the wall nothing is drawn. Visualise the arm, the wall and the drawing, animated.

### 5.2 What it tests

Pose composition (Lecture 1), trajectories in Cartesian space (Lecture 3), and your first taste of **inverse kinematics**: turning a desired end-effector pose into joint angles.

### 5.3 Read the script

Both scripts have the same shape:

```matlab
lim = [-2 2 -2 2 -2 2];
mdl_puma560                      % creates p560 (a SerialLink arm) and poses qz, qr, qn, ...
p560.plot(qz, 'workspace', lim)  % draw the arm at the "zero" pose
a = 1; b = 0; c = 0; d = -1;     % vertical wall x = 1   (run_inclined: a=cos10°, b=0, c=sin10°, d=-cos10°)
wall = Wall(a, b, c, d, lim);  wall.plotwall();
load q_vertical.mat;  traj = q;  % example trajectory: rows of [x y z], YOU REPLACE THIS
Tp = SE3(0.6, 0, 0) * SE3(traj) * SE3.oa([0 1 0], [1 0 0.1]);
q  = p560.ikine6s(Tp);
plot_qtraj(q, wall, p560)
```

⚠️ The handout writes the inclined wall as $\cos 10°\,x + \cos 10°\,z = \cos 10°$, but the script uses $c = \sin 10°$, that is $\cos 10°\,x + \sin 10°\,z = \cos 10°$. The script's version is the sensible one: its normal vector $(\cos 10°, 0, \sin 10°)$ is a vertical wall tipped back by 10°, which matches "pitch 80°" and the handout's Figure 3b. Treat the handout's $\cos 10°\,z$ as a typo and follow the code.

### 5.4 Decode the pose chain (the heart of this task)

`Tp = SE3(0.6, 0, 0) * SE3(traj) * SE3.oa([0 1 0], [1 0 0.1])` builds, for every row of `traj`, the pose of the **end-effector frame**. Read it right to left, as Lecture 1 taught:

1. **`SE3.oa(o, a)`** makes a pure rotation from two vectors: $a$, the **approach vector**, becomes the tool's $z$ axis, and $o$, the **orientation vector**, becomes the tool's $y$ axis. Here $a = (1, 0, 0.1)$, normalised to $(0.995, 0, 0.0995)$, so the tool $z$ axis points along $+x$ (straight at the wall) and tilts up by about $5.7°$. $o = (0, 1, 0)$ keeps the tool's $y$ along world $y$.
2. **`SE3(traj)`** with an $N \times 3$ matrix gives $N$ pure translations, one per row. This is *where the tool frame origin is*, before the final shift.
3. **`SE3(0.6, 0, 0)`** shifts everything 0.6 m towards the wall.

So the tool origin is at $(0.6 + x_k,\ y_k,\ z_k)$ and the tool $z$ axis points at $(0.995, 0, 0.0995)$. The **pen tip** is 0.4 m along the tool $z$ axis, so its world position is

$$ p_{tip} = \begin{pmatrix} 0.6 + x_k + 0.4 \cdot 0.995 \\ y_k \\ z_k + 0.4 \cdot 0.0995 \end{pmatrix} = \begin{pmatrix} 0.998 + x_k \\ y_k \\ z_k + 0.040 \end{pmatrix}. $$

In words: with $x_k = 0$ the tip lands at $x \approx 1$, exactly on the vertical wall. That is why the 0.6 m shift and the 0.4 m pen were chosen together. In MATLAB you can check any pose with `Tp(k) * [0; 0; 0.4; 1]`, or `Tp(k).t + 0.4 * Tp(k).a`.

**Everyday analogy:** you are holding a 40 cm pointer and standing 60 cm from a whiteboard. If you hold the pointer level, its tip touches the board. Move your hand back 1 cm and it draws nothing; move it forward and you push into the board. Your `traj` is the position of your *hand*, not the tip.

### 5.5 Designing a letter: vertical wall

**Step 1. Draw the letter as via points in the wall plane.** The wall plane is $y$–$z$. Letters in the handout's Figure 3a are roughly 0.3 to 0.5 m tall, centred around $y = 0$, $z \approx 0.2$ to $0.5$. For an "L", for instance:

```matlab
% [x y z] of the TOOL ORIGIN; x = 0 means the tip is on the wall, x = -0.1 lifts the pen 10 cm off
L = [ 0  -0.15  0.50;     % top of the vertical stroke
      0  -0.15  0.10;     % bottom
      0   0.15  0.10 ];   % end of the horizontal stroke
```

**Step 2. Make it a smooth trajectory in time.**

```matlab
qdmax = [0.1 0.1 0.1];   dt = 0.05;  tacc = 0.3;
traj = mstraj(L(2:end,:), qdmax, [], L(1,:), dt, tacc);
```

At 0.1 m/s the two strokes (0.4 m and 0.3 m) take 7 s, about 140 rows. Rounded corners are fine here; `tacc` controls how rounded the letter's corner looks.

**Step 3. Multi-stroke letters: lift the pen.** The handout says nothing is drawn when the tip is off the wall. So between strokes, add via points with $x = -0.05$ (tool pulled 5 cm back): the arm moves, nothing is drawn. For an "A" you draw the two legs and lift between the leg and the crossbar. Because the blend cuts corners, add the lift point *at* the stroke's end, then the travel point, then the lowered point at the next stroke's start.

**Step 4. Run the chain.** Leave the `Tp`, `ikine6s`, `plot_qtraj` lines exactly as given. `plot_qtraj(q, wall, p560)` animates the arm through the joint trajectory `q` and traces the tip where it touches the wall.

### 5.6 Two letters at two speeds

Build each letter with its own `qdmax` and concatenate:

```matlab
traj1 = mstraj(L1(2:end,:), [0.1 0.1 0.1], [], L1(1,:), dt, tacc);   % slow
link  = [ -0.05 traj1(end,2) traj1(end,3);  -0.05 L2(1,2) L2(1,3) ];  % lift, travel to letter 2
trajL = mstraj(link, [0.2 0.2 0.2], [], traj1(end,:), dt, tacc);
traj2 = mstraj(L2(2:end,:), [0.3 0.3 0.3], [], L2(1,:), dt, tacc);   % fast: three times the speed
traj  = [traj1; trajL; traj2];
```

Because `plot_qtraj` has no time argument, "speed" shows up as **sample density**: the fast letter has fewer rows and the animation sweeps through it faster. Say this in the report, and back it up with a plot of the tip speed `vecnorm(diff(traj))/dt` against time, which should show two plateaus (0.1 and 0.3 m/s).

### 5.7 The inclined wall

Now the wall is $\cos 10°\,x + \sin 10°\,z = \cos 10°$, or equivalently $x_{tip} = 1 - \tan 10°\, z_{tip}$. In words: the wall leans back, so the higher you draw, the further from the base the wall surface is... actually the *closer*: at $z_{tip} = 0.5$, $x_{tip} = 1 - 0.176 \cdot 0.5 = 0.912$. The pen must follow the lean.

**Step 1.** Design the letter in the wall's own coordinates, as tip positions $(y_{tip}, z_{tip})$, exactly like the vertical case.

**Step 2.** Convert tip positions to tool-origin positions, using the pen offset from section 5.4:

$$ x_k = x_{tip} - 0.6 - 0.398 = (1 - 0.176\, z_{tip}) - 0.998, \qquad y_k = y_{tip}, \qquad z_k = z_{tip} - 0.040. $$

**Worked example.** Tip at $(y, z) = (-0.15, 0.50)$ on the inclined wall: $x_{tip} = 1 - 0.176 \cdot 0.5 = 0.912$, so $x_k = 0.912 - 0.998 = -0.086$, $y_k = -0.15$, $z_k = 0.46$. At the bottom of the stroke, $z_{tip} = 0.10$: $x_{tip} = 0.982$, $x_k = -0.016$, $z_k = 0.06$. So the stroke is a straight line in 3D that leans with the wall. In code:

```matlab
tipYZ = [-0.15 0.50; -0.15 0.10; 0.15 0.10];                % the letter on the wall
xk = (1 - tand(10) * tipYZ(:,2)) - 0.998;
traj_via = [xk, tipYZ(:,1), tipYZ(:,2) - 0.040];
```

⚠️ `run_inclined.m` keeps the same approach vector $(1, 0, 0.1)$ as the vertical case, so the pen meets the inclined wall at a slight angle rather than perpendicularly. It still draws, because what matters is where the *tip* is. If you want the pen perpendicular to the wall, use `SE3.oa([0 1 0], [cosd(10) 0 sind(10)])` and redo the offset arithmetic with $a = (0.985, 0, 0.174)$: tip $= $ origin $+ 0.4 a$, so $x_{tip} = x_{origin} + 0.394$ and $z_{tip} = z_{origin} + 0.069$. Say which you did.

**Common mistake:** designing the inclined letter with $x_k = 0$ as for the vertical wall. The tip then floats in front of the wall at the top and pushes into it at the bottom, and only one point of the letter gets drawn.

### 5.8 Inverse kinematics with `ikine6s`, and its pitfalls

**Inverse kinematics** (IK) means: given where you want the hand, find the joint angles. `ikine6s` is the **closed-form** (analytic) solver for 6-axis arms with a spherical wrist like the PUMA 560; it is fast and exact, unlike the iterative `ikine`. Three things can bite you:

1. **Reachability.** If a pose is outside the arm's reach the solver returns `NaN` for that row and `plot_qtraj` will fail or draw garbage. The PUMA's reach is a little under 0.9 m, and the tool origin here is at $x \approx 0.6$, so letters around $y \in [-0.3, 0.3]$, $z \in [0, 0.6]$ are safe; a 1 m tall letter is not. Check with `any(isnan(q(:)))`.
2. **Multiple solutions.** A 6-axis arm can reach most poses in eight ways (elbow up/down, shoulder left/right, wrist flipped). If the solver switches configuration between two consecutive rows, the animation shows the arm snapping across. Force one configuration with the option string, for example `p560.ikine6s(Tp, 'ru')` (right-handed, elbow up); ⚠️ check the exact option letters in the toolbox manual, since they differ between versions.
3. **Singularities.** Near a singular configuration (for example wrist straight) small Cartesian moves need huge joint moves. If one joint angle shoots off in your plot of `q` against time, move the letter a little.

Plot `q` (six columns) against time in the report: smooth curves mean a healthy trajectory.

---

## 6. Task 4: paths for a micro aerial vehicle

### 6.1 What is asked

Givens: the MAV can move in any direction; its heading always looks at the object; the camera footprint is $1.5 \times 1.5$ m² at 1 m distance (applies to the fountain and the turbine); the MAV keeps a 1 m safety distance to the object (fountain and turbine). ToDo:

- Trajectory from $(-5, 2, 0)$ to $(22, 2, 0)$ avoiding the obstacles of `map1.txt`.
- Trajectory from $(5, 1, 0)$ to $(5, 20, 2)$ avoiding the obstacles of `map2.txt`.
- A flight path, from any start, that inspects the whole LTU fountain with the camera footprint (`fountain.m`).
- The same for the wind turbine (`windturbine.m` loads it, even though the handout text says `fountain.m` for both).
- Optional: several MAVs, collision-free, faster than one MAV.

Visualise the smooth path; optionally animate a frame along it.

### 6.2 What it tests

Via-point trajectories in 3D (`mstraj` again), reading geometry from data, and simple coverage planning. There is no planner algorithm required here; Lecture 7 comes later. You plan by hand and make it smooth.

### 6.3 Read the map files

`Task4/run.m` calls `load_map('maps/map1.txt', 0.1, 2, 0.25)` and `plot_path(map)`. ⚠️ `load_map.p` is p-code; from the call, the three numbers are most likely a grid resolution in $xy$, a resolution in $z$, and an inflation margin around obstacles, but you cannot confirm that. What you *can* read is the text file itself, and it is exact:

```
boundary xmin ymin zmin xmax ymax zmax
block    xmin ymin zmin xmax ymax zmax  r g b
```

**map1** (boundary $x \in [0, 20]$, $y \in [0, 5]$, $z \in [0, 6]$): four **blue** blocks at $x \in [0.1, 0.9], [6.1, 6.9], [12.1, 12.9], [18.1, 18.9]$ spanning the full $y$ and $z \in [0, 3.9]$ (walls standing on the floor), and three **red** blocks at $x \in [3.1, 3.9], [9.1, 9.9], [15.1, 15.9]$ spanning the full $y$ and $z \in [2.1, 6]$ (walls hanging from the ceiling). So it is a vertical slalom: fly *over* every blue wall (above $z = 3.9$) and *under* every red wall (below $z = 2.1$), alternating every 3 m in $x$.

**map2** (boundary $x \in [0, 10]$, $y \in [-5, 20]$, $z \in [0, 6]$): at $y \in [2, 2.5]$ a wall across the full width with a **window** at $x \in [3, 7]$, $z \in [1.5, 4.5]$ (the wall pieces are $z \in [0, 1.5]$, $z \in [4.5, 6]$, and the two side pieces $x \in [0, 3]$ and $[7, 10]$ for $z \in [1.5, 4.5]$). At $y \in [0, 0.5]$ a **panel** at $x \in [3, 7]$, $z \in [2.4, 4.5]$ in front of the upper part of the window. At the far end: a **floor slab** $y \in [15, 20]$, $z \in [0, 1]$; a **wall** on it at $y \in [15, 16]$, $z \in [1, 3.5]$; and a **ceiling beam** at $y \in [18, 19]$, $z \in [4.5, 6]$.

### 6.4 Step-by-step: map1

**Step 1. Via points.** Fly along the centre line $y = 2$. The gap between a blue wall and the next red wall is 2.2 m in $x$, so change altitude in the middle of each gap. A safe pattern (over blue at $z = 5$, under red at $z = 1$):

```matlab
start = [-5 2 0];  goal = [22 2 0];
via = [ 0.5 2 5.0;  2.0 2 3.0;  3.5 2 1.0;  5.0 2 3.0;  6.5 2 5.0;  8.0 2 3.0;
        9.5 2 1.0; 11.0 2 3.0; 12.5 2 5.0; 14.0 2 3.0; 15.5 2 1.0; 17.0 2 3.0;
       18.5 2 5.0;  goal ];
```

**Check one dangerous corner by hand.** Between via points $(2.0, 3.0)$ and $(3.5, 1.0)$ (in $x$–$z$) the path crosses $x = 3.1$, the red wall's leading edge, at $z = 3.0 - 2.0 \cdot (1.1 / 1.5) = 1.53$, which is below the red wall's bottom at $2.1$. Safe, with 0.57 m to spare. Do the same check at $x = 3.9$ (trailing edge) and at the blue walls' top edges. If `load_map` inflates obstacles by 0.25 m, your margins shrink by that much; keep at least 0.5 m.

**Step 2. Smooth it.**

```matlab
traj = mstraj(via, [1 1 1], [], start, 0.1, 0.5);
plot_path(map); hold on; plot3(traj(:,1), traj(:,2), traj(:,3), 'g', 'LineWidth', 2)
```

With `tacc = 0.5` s at 1 m/s the corners round by about half a metre, which pulls the path *towards* the obstacle at the apex of each altitude change. That is why the via points above overshoot (5.0 instead of the minimum 3.9 plus margin). After blending, re-check clearance numerically: for each trajectory point test whether it lies inside any block's $[\min, \max]$ box (expand the box by your margin).

### 6.5 Step-by-step: map2

Start $(5, 1, 0)$ is already between the panel (at $y \le 0.5$) and the window wall (at $y \ge 2$), and the window is at $x \in [3, 7]$, $z \in [1.5, 4.5]$, right in front of you at $x = 5$. So: rise to $z = 2.5$ (the window's lower-middle, clear of its sill at 1.5 and lintel at 4.5), fly through, continue along $x = 5$ to the far end, climb above the wall at $y \in [15, 16]$ (top at $z = 3.5$) but stay below the beam at $y \in [18, 19]$ (bottom at $z = 4.5$), then descend onto the goal $(5, 20, 2)$, which is above the slab (top at $z = 1$).

```matlab
start = [5 1 0];  goal = [5 20 2];
via = [5  1.0 2.5;     % rise in place
       5  3.0 2.5;     % through the window
       5 13.0 2.5;     % cruise
       5 14.0 4.0;     % climb before the wall (top 3.5)
       5 17.0 4.0;     % over the wall, still under the beam (bottom 4.5)
       5 19.5 3.0;     % descend after the beam
       goal];
traj = mstraj(via, [1 1 1], [], start, 0.1, 0.5);
```

The squeeze is between the wall top ($3.5$) and the beam bottom ($4.5$): you have 1 m of vertical room at $z = 4.0$, and only between $y = 16$ and $y = 18$ do both matter. Keep the blend short there or add via points so the rounded path does not dip.

### 6.6 Inspection flights: fountain and wind turbine

**The two numbers that matter.** The footprint is $1.5 \times 1.5$ m² at 1 m, and the safety distance is 1 m, so fly at exactly 1 m from the surface and each image covers a 1.5 m square. For full coverage, successive passes must be no more than 1.5 m apart; use about 1.2 m (20 % overlap) so small position errors do not leave gaps. "Heading looks at the object" means the yaw at each point is

$$ \psi_k = \operatorname{atan2}(y_c - y_k,\ x_c - x_k), $$

where $(x_c, y_c)$ is the object's axis. In words: point the nose at the centre.

**Fountain.** `fountain.m` does `ptCloud = pcread('fountain.ply'); pcshow(ptCloud)`. The cloud (Figure 6a) is a tall central column standing in a wide, flat basin. Get its size from the data:

```matlab
P = ptCloud.Location;             % N x 3
xc = mean(P(:,1)); yc = mean(P(:,2));                 % axis (refine: use only points with z above the basin)
zmin = min(P(:,3)); zmax = max(P(:,3));
r_col = max( hypot(P(P(:,3)>zmin+0.5,1)-xc, P(P(:,3)>zmin+0.5,2)-yc) );  % column radius above the basin
```

Plan a **spiral or stacked rings** around the column at radius $R = r_{col} + 1$ m, with ring spacing 1.2 m from just above the basin to the top. For a column 6 m tall that is $\lceil 6 / 1.2 \rceil = 5$ rings. Generate each ring as, say, 16 via points at angles $0, 22.5°, \dots$; a spiral is simply a ring whose $z$ rises by 1.2 m per turn. Add the yaw as a fourth column and smooth all four with `mstraj`:

```matlab
th = linspace(0, 2*pi*nTurns, 16*nTurns)';
via = [xc + R*cos(th), yc + R*sin(th), linspace(z0, z1, numel(th))', atan2(yc-(yc+R*sin(th)), xc-(xc+R*cos(th)))];
```

Note the yaw column is just $\theta + \pi$ (look inwards) and wraps every turn; use `unwrap` or `angdiff` so `mstraj` does not spin the drone the long way at the $\pm\pi$ seam.

**Wind turbine.** `windturbine.m` loads `Points10.mat`, divides `X`, `Y`, `Z` by 10 and plots. The result (Figure 6b) is a tower about 60 m tall with a nacelle and three blades. A coverage plan has two parts: **vertical passes** around the tower (rings at 1 m from the tower surface, 1.2 m apart, or a few vertical lines 1.5 m apart around the circumference, since the tower is thin), and **blade passes**: for each blade, fly parallel to the blade axis at 1 m offset on the front face and on the back face, in strips 1.2 m apart across the blade's width. Extract each blade's axis from the point cloud by taking points above the nacelle height and fitting a line (or simply by hand from the plot). Heading looks at the blade.

**Multi-MAV (optional).** Give each drone a disjoint set of rings (or blades) at different heights so their paths never cross, and let them fly simultaneously. Total time drops roughly by the number of drones; report path length per drone and the maximum of the per-drone times.

**Common mistakes:** planning the inspection path at the object's surface instead of 1 m away; forgetting that the footprint is 1.5 m only *at* 1 m (closer means a smaller footprint, more passes); letting the yaw wrap at $\pm\pi$ so the drone does a full spin.

---

## 7. The optional book exercises (Chapter 3, exercises 7, 8, 9, 13)

These are marked optional, but they are exactly the understanding the examiner probes, so do at least the arithmetic.

### Exercise 7: `tpoly` overshoot

Run `tpoly(0, 1, 50, v0, vT)` for several initial and final velocities. The quintic is the *only* fifth-order polynomial that satisfies all six boundary conditions, so if you insist on a non-zero velocity at an end, the curve must be *moving* there. With a large positive start velocity, the curve shoots past 1 and comes back to land on 1 with zero speed: **overshoot**. The lecture script's `tpoly(0, 1, 50, 0.5, 0)` is a dramatic case: 0.5 per step over 50 steps is 25 units of "momentum" for a 1 unit trip, so it overshoots by many units. With a negative initial velocity the curve first goes *backwards* below 0 (undershoot). The rule of thumb: overshoot appears when a boundary velocity is comparable to or larger than the average speed $d/T$ (here $1/50 = 0.02$ per step). Try $v_0 = 0.01, 0.02, 0.05, -0.02$ and watch.

### Exercise 8: `lspb` velocity bounds

For `lspb(0, 1, 50, V)`: the distance is $d = 1$ and the duration $T = 50$ steps. If $V \le d/T = 0.02$ the robot cannot cover the distance even at full speed for the whole time, so **$V > 0.02$** is required. If $V > 2d/T = 0.04$, the acceleration and deceleration blends would have to be shorter than zero, so **$V \le 0.04$**. The window is $0.02 < V \le 0.04$. The lecture script's $0.025$ and $0.035$ both work; try $0.015$ and $0.05$ and read the error message. At $V = 0.04$ the profile is a pure triangle with no constant-speed segment.

### Exercise 9: steps needed at a speed limit of 0.025

For the quintic $s(\tau) = 6\tau^5 - 15\tau^4 + 10\tau^3$ the peak velocity is at mid-motion and equals $1.875\, d/T$ (Lecture 3 notes derive it: the mean-to-peak ratio is about 0.523). To keep the peak at or below 0.025 per step, $T \ge 1.875 / 0.025 = 75$ steps. For `lspb` with cruise speed $V = 0.025$, exercise 8 says $d/V < T \le 2d/V$, i.e. $40 < T \le 80$; the fastest legal choice is $T = 41$ steps, where the blends are a single step each. So the trapezoid needs about **41 steps against the quintic's 75**: the trapezoid is faster because it sits at the speed limit almost the whole way, while the quintic only touches it for an instant. Verify: `[s,sd]=tpoly(0,1,75); max(sd)` gives 0.025, and `[s,sd]=lspb(0,1,41,0.025); max(sd)` gives 0.025.

### Exercise 13: `mstraj` with different boundary velocities and acceleration times

(a) `mstraj` has optional `qd0` and `qdf` arguments after `TACC` for initial and final velocity; a non-zero start velocity makes the first blend asymmetric. (b) Increase `TACC` and record the number of rows: the lecture script compared `TACC = 0` with `TACC = 2` and the longer blend produced more rows. Plot `size(traj,1) * dt` against `TACC` for, say, `TACC = 0:0.5:4`; total time grows roughly linearly with `TACC` because each of the $M - 1$ corners adds blend time while the straight parts shrink only slightly. Also note the path gets rounder and cuts corners more.

---

## 8. Report checklist and examiner questions

### 8.1 Report checklist (from the template)

- [ ] Title, author or group number, date.
- [ ] **Objective:** each of the four tasks in your own words; the difficulties you expected before starting.
- [ ] **Methodology:** for each task, the via points you chose and why, the `mstraj` parameters (speed, `dt`, `tacc`) and the formulas you used (pen-tip offset in Task 3, footprint spacing in Task 4). Short code snippets, not whole files.
- [ ] **Results:** the plots the handout asks for in each task (poses and path with animated frame; map plus path; arm, wall and letter; 3D map plus path; point cloud plus inspection path), plus velocity-over-time plots to *prove* smoothness. What turned out to be hard; what you learned.
- [ ] About 1.5 pages of text; figures extra.
- [ ] One zip: report, all code, recorded videos of the animations, named with your name or group.

### 8.2 What the demo examiner is likely to ask

1. Why does `TACC = 0` violate "smooth", and what does raising it cost you? (Acceleration jumps at corners; corners get cut, so you need clearance.)
2. What does `QDMAX` do when the axes have different distances to travel? (The slowest axis sets the segment time; the others slow down to arrive together.)
3. In Task 1, what goes wrong at pitch $= \pi/2$, and why does your approach survive it? (Gimbal lock in roll-pitch-yaw; generating rotations from angles is fine, only the inverse is ambiguous.)
4. Why not `ctraj` per segment in Task 1? (It stops at every via point.)
5. In Task 3, why `SE3(0.6, 0, 0)` and what does `SE3.oa` do? (Puts the tool origin 0.6 m out so a 0.4 m pen reaches $x = 1$; `oa` sets the tool's $z$ to the approach vector and $y$ to the orientation vector.)
6. How did you lift the pen between strokes? (Move the tool origin back in $x$; nothing is drawn off the wall.)
7. Why `ikine6s` and not `ikine`? What does a `NaN` mean? (Closed-form solver for a spherical-wrist 6-axis arm; `NaN` means unreachable.)
8. In Task 4, how did you get 1.2 m from "1.5 × 1.5 m at 1 m"? (Footprint width minus overlap.)
9. Where did your 5 seconds go in the door task? (Either arrive later via `TSEG`/waiting, or route around; show the time stamp at which the path crosses the door line.)
10. Show me the velocity plot. Where are the segment boundaries? (Blends appear as smooth ramps between constant-speed plateaus.)

---

## In a nutshell

Lab 1 is one idea applied four times: choose the places a robot must visit, then let `mstraj` turn them into a smooth, timed trajectory whose speed is capped per axis and whose corners are rounded by the blend time. For the floating object you add orientation to the configuration vector and watch out for the $\pm\pi$ wrap and the pitch-$\pi/2$ singularity of roll-pitch-yaw. For the ground robot you read the walls out of the map matrix, leave room for the blends, and handle the door with timing or a detour. For the PUMA you design the letter in the wall plane, convert tip positions to tool-origin positions with the 0.6 m shift and 0.4 m pen, and let `ikine6s` find the joint angles. For the MAV you read the box worlds from the text files, weave over and under the walls, and build inspection rings or strips 1.2 m apart at 1 m from the surface with the nose pointing at the object. The report is short, but it must show plots of position *and* velocity so the examiner can see the motion really is smooth.

## Key terms

- **Via point:** a configuration the path must pass through (or near, after blending).
- **Trajectory:** a path with timing: configuration as a function of time.
- **Quintic polynomial (`tpoly`):** fifth-order curve matching position, velocity and acceleration at both ends.
- **LSPB / trapezoidal profile (`lspb`):** accelerate, cruise at constant speed, decelerate.
- **`mstraj`:** multi-segment, multi-axis trajectory through via points with blended corners.
- **`QDMAX`:** per-axis speed limit; the slowest axis sets each segment's time.
- **`TACC` (acceleration time):** blend duration at each via point; zero gives acceleration jumps.
- **`ctraj` / `interp`:** smooth interpolation between two poses using slerp for rotation.
- **Roll-pitch-yaw $(\phi, \theta, \psi)$:** ZYX three-angle orientation; singular at pitch $\pm\pi/2$.
- **Gimbal lock:** the loss of one rotational degree of freedom in a three-angle representation at the singularity.
- **Slerp:** spherical linear interpolation of unit quaternions; no gimbal lock, `'shortest'` option.
- **`SE3.oa(o, a)`:** rotation built from an orientation vector (tool $y$) and approach vector (tool $z$).
- **Approach vector:** the direction the tool's $z$ axis points, here towards the wall.
- **Inverse kinematics (`ikine6s`):** joint angles from a desired end-effector pose; closed-form for 6-axis spherical-wrist arms.
- **Reachability / `NaN`:** a pose the arm cannot reach makes the solver return `NaN`.
- **Occupancy map (`map1`):** matrix with 1 for walls and 0 for free space, indexed `map(y, x)`.
- **Box world (`map1.txt`, `map2.txt`):** obstacles given as axis-aligned boxes by min and max corners.
- **Camera footprint:** the area one image covers, $1.5 \times 1.5$ m² at 1 m here.
- **Coverage path:** a route whose footprints together cover the whole object, passes $\le 1.5$ m apart.
- **p-code (`.p` files):** compiled, unreadable MATLAB code; you can call it but not read it.

## Check yourself

1. You run `mstraj` with `QDMAX = [1 1]` on a segment that moves 30 cells in $x$ and 10 in $y$. How long does the segment take, and how fast does $y$ actually move?
2. Why does the handout's "rounded corners" requirement rule out chaining `ctraj` segments, and which `mstraj` argument delivers it?
3. In Task 3 the tool origin is at `traj` row $(0, 0.1, 0.3)$. Where is the pen tip, and is it on the vertical wall?
4. For `lspb(0, 1, 50, V)`, which values of $V$ are legal, and why does $V = 0.05$ fail?
5. Your drone inspects a 6 m tall column with a 1.5 m footprint at 1 m. How many rings do you need with 20 % overlap, and what yaw does the drone hold at the point due east of the column?

<details>
<summary>Answers</summary>

1. The $x$ axis needs 30 s at 1 cell/s and $y$ needs 10 s, so the segment takes 30 s (slowest axis). The $y$ axis is slowed to $10 / 30 = 0.33$ cells/s so both arrive together.
2. Every `ctraj` segment starts and ends at zero velocity, so the object stops at each via point; the corner is a full stop, not a curve. `mstraj` with `TACC > 0` blends the end of one segment into the start of the next, rounding the corner and keeping the speed up.
3. Tip $=$ origin $+ 0.4 \cdot a$ with the shift applied: $x = 0.6 + 0 + 0.398 = 0.998$, $y = 0.1$, $z = 0.3 + 0.040 = 0.34$. With $x \approx 1$ it is on the vertical wall $x = 1$ (within the tolerance `plot_qtraj` uses).
4. $d/T < V \le 2d/T$ with $d = 1$, $T = 50$, so $0.02 < V \le 0.04$. At $V = 0.05$ the robot would cover the distance in 20 steps at cruise speed, leaving no feasible blend shape for a 50-step profile; the blend time would have to be negative.
5. Spacing $1.5 \times 0.8 = 1.2$ m, so $\lceil 6 / 1.2 \rceil = 5$ rings. Due east means the drone is at $(x_c + R, y_c)$; looking at the centre gives $\psi = \operatorname{atan2}(0, -R) = \pi$ (facing west).

</details>
