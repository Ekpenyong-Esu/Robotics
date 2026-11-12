# Lab 2 – Mobile Robot Motion and Control (TurtleBot3 Burger) – Study Notes

> Built from the handout `R7010_lab2.pdf` (8 pages; `R7010_lab2_Virtual_Machine.pdf` is the identical file), the Simulink templates in `Lab2_simulink.zip` (`Lab2_Task_1.slx` to `Lab2_Task_5.slx`), the older `Lab2_VM_Task_1234.slx` and `Lab2_VM_Task_5678.slx`, the Python template `turtle_controller.py`, the `Report Template.docx`, and the Lecture 5 script `TurtleBot3.m`. The hardware, bring-up and ROS topics are covered in detail in the [TurtleBot notes](../TurtleBot/TurtleBot3_Burger_Study_Notes.md); the control laws come from the [Lecture 4 notes](../Lecuture4/Lecture_4_Mobile_Robotics_Study_Notes.md). These notes tell you how to turn those two into a working lab.

## What this lab is about, and why it matters

In Lab 2 you take the four control problems from Lecture 4 (move to a point, move to a pose, follow a line, follow a path) off the slides and onto a real two-wheeled robot, then you add two sensing tasks with the robot's LiDAR. Everything you build produces just two numbers, a forward speed and a turn rate, that you publish to the robot twenty or so times a second. The lab matters because it is where you discover the gap between a clean formula and a robot that overshoots, jitters, drives off in the wrong direction or stops a metre short, and learn to close that gap with gains, saturation and a bit of signal filtering.

## Roadmap: the main topics

1. Aim, prerequisites and the seven operating rules.
2. Simulink for someone who has never opened it.
3. The template model, block by block (including the STOP slider and the Enable ports).
4. Bring-up in one page (details in the TurtleBot notes).
5. Task 1a: move to a point.
6. Task 1b: move to a pose.
7. Task 2: follow a trajectory (square, your group number, a sinusoid).
8. Task 3: follow a line.
9. Task 4: LiDAR data registration.
10. Task 5: keep a distance from a wall.
11. Plots and the report.
12. Debugging checklist, and what the examiner is likely to ask.

---

## 1. Aim, prerequisites and the rules

**Aim.** The handout says it plainly: introduce the basic functions of mobile-robot motion and control, on the TurtleBot3 Burger, with your algorithms written in Simulink. You get an intro to the robot, the command-line tools to drive it from a laptop, and template code to bring it up. The real goal is to *experiment with different control strategies* so the robot follows the motion patterns you ask for.

**Prerequisites.** Automatic control (what a proportional controller is, what a gain does), some maths, the Lab 1 tasks, the TurtleBot manual, and Simulink. If "P controller" means nothing to you yet, read section 5 of these notes first; it is explained from scratch.

**The seven notes from the handout.** Treat these as rules, not suggestions:

1. Always run your algorithms **on your local machine**, connected to the robot over Wi-Fi (section 4). The robot only runs its drivers.
2. Always put the robot **on the ground** for experiments. The teleop screenshot in the handout shows a top speed of 0.22 m/s, which is fast enough to drive off a desk before you react.
3. **Disconnect the battery** when you finish.
4. **Never run on low battery.** The robot beeps a warning.
5. **Never charge the battery while it is connected** to the robot.
6. In an emergency, press the **STOP slider** in the Simulink model (section 3 explains what it does).
7. Unzip `Lab2_simulink.zip` and use the matching file `Lab2_Task_1.slx` to `Lab2_Task_5.slx` for each task.

⚠️ The five task files in the zip are byte-for-byte identical. There is nothing task-specific inside them; they are five copies of one template so that you can keep your five solutions separate. Do not spend time looking for the differences.

---

## 2. Simulink, for a first-timer

**Simulink** is MATLAB's graphical programming environment. Instead of writing lines of code, you drag **blocks** onto a canvas and join them with lines called **signals**. A block takes signals in on its left, computes something, and puts signals out on its right. A Gain block multiplies by a number; a Sum block adds; a Trigonometric Function block computes atan2; a MATLAB Function block lets you type a few lines of ordinary MATLAB when a formula is awkward to draw. The model runs in discrete steps set by the **sample time** (for a robot controller something like 0.05 s, i.e. 20 Hz), and at every step every block updates once. You watch signals with a **Scope** block (a live plot) and save them for your report with a **To Workspace** block, which dumps the signal into a MATLAB variable when the run stops. A **Subsystem** is a box that hides a group of blocks behind named input and output ports, which is how the template keeps the canvas tidy.

**Everyday analogy:** a Simulink model is a kitchen production line. Ingredients (sensor readings) come in on a conveyor, each station (block) does one thing to them, and the finished dish (the velocity command) goes out the other end. The sample time is how often the conveyor advances one step.

The **ROS 2 blocks** are what connect the kitchen to the robot. **Subscribe** listens on a topic (here `/odom` and `/scan`) and outputs the latest message as a **bus**. A bus is Simulink's word for a structured signal with named fields, like a MATLAB struct travelling along one wire: the `/odom` bus contains `pose.pose.position.x`, `pose.pose.orientation.z` and so on. A **Bus Selector** picks the fields you want out of a bus and gives you plain numeric signals. **Blank Message** creates an empty message of a given type (here `geometry_msgs/Twist`), **Bus Assignment** writes your numbers into chosen fields of it (`linear.x` and `angular.z`), and **Publish** sends it on a topic (here `/cmd_vel`). If you have read the Lecture 5 notes, this is exactly the publisher/subscriber pattern drawn with boxes instead of code.

---

## 3. The template model, block by block

Opening `Lab2_Task_1.slx` you find three things on the top level plus two sensor inputs.

**Inputs.** Two `ros2lib/Subscribe` blocks, one on `/odom`, one on `/scan`. Each is followed by a Bus Selector. From `/odom` the template pulls out position x, position y and the orientation. ⚠️ The orientation on `/odom` is a quaternion, not an angle. The template's inport is simply called "Orientation (theta)", so check whether the Bus Selector already converts the quaternion to yaw or whether you have to add that conversion yourself (the formula is in the TurtleBot notes, section 9.2, and the Lecture 1 notes explain why). If you feed the raw `orientation.z` component in as if it were an angle, the robot will turn the wrong amount and you will chase a bug for an hour.

**The Proportional Controller subsystem.** This is where your work goes. It has four inports, `Desired position`, `Position X`, `Position Y` and `Orientation (theta)`, and two outports, `Linear Velocity (v)` and `Angular Velocity (w)`. Inside, the template gives you a skeleton: a `STOP` constant (value 0), a **Slider Switch**, a `Stop` constant and two Switch blocks, one for $v$ and one for $\omega$. The switches route either your computed command or the zero constant to the outport, depending on the slider. When you flip the slider to STOP, both outputs become zero and the robot stops within one sample. That is the emergency stop from rule 6, and it works because it sits *downstream* of everything you build: no matter what your controller is doing, zero wins.

**The Command Velocity Publisher subsystem.** Inports `v` and `w` (and a `header`), then Blank Message → Bus Assignment → Publish on `/cmd_vel`. Your $v$ goes into `linear.x`, your $\omega$ into `angular.z`. You should not need to change anything here.

**Enable ports.** Both subsystems have an **Enable** port. An enabled subsystem only runs while its Enable input is true, and when disabled its outputs hold or reset. The template uses this so that the controller and the publisher run only once valid data is flowing (for example after the first `/odom` message has arrived), and so that you can gate the whole thing with one signal. If your controller seems dead, check that whatever drives Enable is actually true.

**The More Info subsystem** is documentation only.

**The old VM models.** `Lab2_VM_Task_1234.slx` and `Lab2_VM_Task_5678.slx` are the same design built with the ROS 1 `robotlib` blocks (Publish2, one Subscribe on `/odom`, and in the 5678 version an extra Subscribe on `/scan` feeding a `Range` inport). They are from the virtual-machine version of the lab. ⚠️ The handout and the current templates are ROS 2 (Domain ID, `ros2lib`); the VM models and `turtle_controller.py` are ROS 1. Use the ROS 2 templates unless your lab assistant tells you otherwise.

---

## 4. Bring-up in one page

Full detail, including photos of where the buttons are, is in the [TurtleBot notes](../TurtleBot/TurtleBot3_Burger_Study_Notes.md) section 8. The short version:

1. Laptop on Wi-Fi `turtlenet` (password `R7010E_robotics`).
2. Battery in, Raspberry Pi power button on.
3. `ssh turtle@192.168.50.X0` (robot X; password `turtle`). The welcome banner lists the commands: `run`, `teleop`, `switch_network`, `set_multirobot`.
4. `run` in that shell and leave it running. Its log should end with `[diff_drive_controller]: Init Odometry` and `Run!`. If odometry is confused, Ctrl+C and `run` again: odometry restarts from (0, 0, 0).
5. Optional: `teleop` in a second shell to drive by keyboard (`w a d x`, `s` to stop). The screenshot shows the Burger's limits: about **0.22 m/s** linear and **2.84 rad/s** angular. Close teleop before running Simulink, or both will publish on `/cmd_vel`.
6. In Simulink, ROS Network → Domain ID = **3X** for robot X (robot 6 → 36). Everything else in that dialog is ROS 1 leftovers.

**Common mistake:** forgetting that `run` resets the odometry origin to wherever the robot happens to be. Your "point B at (1, 0)" is one metre ahead of *where the robot was when you typed `run`*, not of some mark on the floor.

---

## 5. Task 1a: move to a point

**Goal.** Drive from point A $(x_A, y_A)$ to point B $(x_B, y_B)$; repeat for several B.

### 5.1 The idea, from zero

A **proportional (P) controller** does one thing: it makes the command proportional to the error. Big error, big command; small error, small command; zero error, zero command. Here there are two errors, so there are two P controllers running side by side.

1. **Distance error** $\rho$: how far away is B?
   $$\rho = \sqrt{(x_B - x)^2 + (y_B - y)^2}$$
   In words: Pythagoras on the difference between where you want to be and where you are.
2. **Heading error** $e_\theta$: how far off is the robot's nose from pointing at B?
   $$\theta^* = \operatorname{atan2}(y_B - y,\ x_B - x), \qquad e_\theta = \operatorname{wrap}(\theta^* - \theta)$$
   $\theta^*$ is the direction of the line from the robot to B. `atan2(dy, dx)` is the four-quadrant arctangent: unlike `atan(dy/dx)` it knows the difference between "B is behind-left" and "B is ahead-right", and it does not divide by zero when $dx = 0$.
3. **Control law** (Lecture 4, slide 9):
   $$v = K_v\,\rho, \qquad \omega = K_h\, e_\theta$$

**Everyday analogy:** walking to a door in a dark room. The further away it is, the faster you walk (distance term). If it is off to your left, you turn left, and the more it is off, the harder you turn (heading term). As you line up, you stop turning; as you arrive, you slow down.

### 5.2 Angle wrapping

Angles live on a circle. If $\theta^* = 170°$ and $\theta = -170°$, the naive difference is $340°$, but the robot only needs to turn $-20°$. Always wrap the error into $[-\pi, \pi]$:

```matlab
e = atan2(sin(e), cos(e));   % one-line wrap, works for any e
```

or use the toolbox's `angdiff`. **This is the single most common Task 1 bug.** The symptom is a robot that spins the long way round or oscillates wildly when the target is behind it.

### 5.3 Saturation and gains

The Burger's firmware clips commands to roughly $|v| \le 0.22$ m/s and $|\omega| \le 2.84$ rad/s. Put a **Saturation** block on both outputs yourself so you know what is being sent, and so that your plots show the real command. With $K_v = 0.5$ and a target 2 m away you would ask for 1 m/s, which just saturates; the robot moves at 0.22 m/s until $\rho < 0.44$ m, then slows proportionally. That is fine, and it is in fact a decent speed profile.

For $K_h$, start around 1 to 2. Too small and the robot drifts in a wide arc; too large and, because the loop runs at a finite sample rate and the motors have lag, the heading overshoots and chatters. Tune by watching the yaw-versus-time plot: you want it to settle without ringing.

**Worked example.** Robot at $(0, 0, 0)$ (origin, facing $+x$), target $B = (1, 1)$, $K_v = 0.5$, $K_h = 1.5$. Then $\rho = \sqrt 2 = 1.414$ m, $\theta^* = \operatorname{atan2}(1, 1) = 0.785$ rad (45°), $e_\theta = 0.785$. Commands: $v = 0.707$ m/s, saturated to $0.22$; $\omega = 1.18$ rad/s. The robot drives forward while turning left, and as it comes round to face B the $\omega$ term dies away. A few seconds later at, say, $(0.8, 0.75, 0.9)$: $\rho = 0.32$, $\theta^* = \operatorname{atan2}(0.25, 0.2) = 0.896$, $e_\theta = -0.004$, so $v = 0.16$ m/s and $\omega \approx 0$: a straight, slowing final approach.

### 5.4 Stopping

A P controller never reaches exactly zero error, and odometry noise means $\rho$ never reads exactly 0.000. Two things go wrong at the goal if you do nothing: the robot creeps at a few millimetres per second forever, and because $\theta^*$ is computed from a tiny $dx, dy$ it flips wildly when you are centimetres away, so the robot pirouettes on the spot. The fix is a **dead zone**: when $\rho < \rho_{\min}$ (2 to 5 cm) set both $v$ and $\omega$ to zero (a Switch block on $\rho$). Some people also hold $\theta^*$ at its last value inside the dead zone.

### 5.5 What to plot and report

XY plot of the path with `axis equal` (the handout insists: "correct ratio, no stretching"), with the start and the target marked; $x(t)$, $y(t)$ and $\theta(t)$ against time; optional heading arrows along the path (`quiver(x, y, cos(th), sin(th))`). Repeat for at least two or three different B, including one behind the robot. In the report, give the control law with your gains, explain the wrapping and the dead zone, and comment on overshoot and settling time.

---

## 6. Task 1b: move to a pose

**Goal.** Reach B *and* end up facing $\theta_B$. Think parking: being in the bay is not enough, you have to be straight.

### 6.1 Why the point controller is not enough

The point controller only cares about where B is, not which way you face when you get there. To control the final heading you need a third variable, and the Lecture 4 polar-coordinate law (slides 15–16) does exactly that.

### 6.2 The three polar variables

Work in a goal frame: put the origin at B with its $x$-axis along $\theta_B$. In that frame the robot is at $(\Delta_x, \Delta_y, \theta')$ where $\theta' = \theta - \theta_B$ (wrapped). Then

$$\rho = \sqrt{\Delta_x^2 + \Delta_y^2}, \qquad \alpha = \operatorname{wrap}\big(\operatorname{atan2}(-\Delta_y, -\Delta_x) - \theta'\big), \qquad \beta = \operatorname{wrap}(-\theta' - \alpha).$$

(Here $\Delta_x, \Delta_y$ are robot minus goal, hence the minus signs inside atan2 so that the angle points from robot to goal.) In words: $\rho$ is the distance to the goal; $\alpha$ is the angle between the robot's nose and the line to the goal, exactly the heading error of Task 1a; $\beta$ is the angle between that line and the goal's desired heading, i.e. how much the final heading will be wrong if you drive straight in along that line.

### 6.3 The control law and the stability conditions

$$v = K_\rho\,\rho, \qquad \omega = K_\alpha\,\alpha + K_\beta\,\beta$$

with the conditions from the slides: $K_\rho > 0$, $K_\beta < 0$, $K_\alpha - K_\rho > 0$. Typical starting values: $K_\rho = 0.5$ (or less, you are saturated anyway), $K_\alpha = 2$, $K_\beta = -0.5$. The negative $K_\beta$ is the counter-intuitive part: it makes the robot deliberately swing *away* from the goal heading early so that it can arrive along a curve that ends pointing the right way. **Common mistake:** choosing $K_\beta > 0$ "because negative gains look wrong"; the robot then arrives at the right spot facing the wrong way and spins there.

**Everyday analogy:** reversing into a parallel parking space. You do not aim straight at the gap; you swing out first ($\beta$ term) so that the final motion lands you parallel to the kerb.

### 6.4 The goal behind the robot

The law assumes the goal is roughly in front: $\alpha \in (-\pi/2, \pi/2]$. If $|\alpha| > \pi/2$ the slide says "otherwise $\alpha \to -\alpha$", which in practice means: drive **backwards**. Negate $v$ and recompute $\alpha$ with the robot's heading flipped by $\pi$. Simpler for the lab: if the goal is behind, first rotate on the spot ($v = 0$, $\omega = K_\alpha \alpha$) until $|\alpha| < \pi/2$, then switch to the full law. State in the report which option you chose.

### 6.5 Plots and report

Same plots as 1a plus the heading at arrival. Show at least two poses B, one with $\theta_B$ very different from the arrival direction, so the curved approach is visible. Report the gains and show they satisfy the three conditions.

---

## 7. Task 2: follow a trajectory

**Goal.** Reuse the Task 1 controller to follow (a) a square, (b) the digits of your group number, (c) a curve such as a sinusoid.

### 7.1 Two ways to turn a path into a moving target

A controller that goes to a *point* can follow a *path* if you keep moving the point. There are two standard ways:

1. **Waypoint sequencing.** Store the corners of the shape as a list. Aim at waypoint $k$; when $\rho < r_{\text{switch}}$ (say 0.1 m), increment $k$. In Simulink this is a counter (Unit Delay plus a Switch, or a small MATLAB Function block with a persistent index) that indexes into a constant matrix of waypoints. For the square you wrap $k$ back to 1 so the robot repeats the square, which is what the handout asks you to observe.
2. **Time-parameterised target (the "donkey and carrot").** Define the target as a function of time, $(x^*(t), y^*(t))$, and let the point controller chase it. This is Lecture 4's pure-pursuit idea (slide 13). It is the natural choice for the sinusoid.

**Everyday analogy:** method 1 is a treasure hunt, you go clue to clue; method 2 is following a friend who walks ahead of you along the route.

### 7.2 Building the three shapes

- **Square of side $L$:** waypoints $(0,0), (L,0), (L,L), (0,L)$ and back to $(0,0)$. With $L = 1$ m you stay within a lab mat. The robot will **cut the corners**: as it approaches a corner the switch radius triggers and the heading controller starts turning early, so the path rounds off. The bigger $r_{\text{switch}}$ or the slower the heading loop, the rounder the corners. Say so in the report; it is expected behaviour, not a bug. If you want sharper corners, add a rule "stop and rotate on the spot until $|e_\theta| < 0.1$ before driving to the next waypoint".
- **Your group number:** treat each digit as a seven-segment-style list of waypoints on a grid. A "1" is two points; a "4" is a short polyline; a "0" is a rectangle. Place the digits side by side with a gap, and either lift the "pen" (drive between digits without recording) or just accept the connecting stroke.
- **Sinusoid:** $x^*(t) = v_x t$, $y^*(t) = A\sin(\omega_s t)$, for example $v_x = 0.1$ m/s, $A = 0.3$ m, $\omega_s = 0.5$ rad/s. A **Clock** block gives $t$; two function blocks give $x^*, y^*$. Keep the target's speed well under 0.22 m/s: the carrot's speed is $\sqrt{v_x^2 + (A\omega_s\cos\omega_s t)^2}$, which here peaks at about $0.18$ m/s, leaving the robot a little margin to catch up.

### 7.3 Why a pure point controller lags on a moving target

With $v = K_v \rho$ the robot only moves as fast as the carrot when $\rho = v_{\text{carrot}}/K_v$, so there is a steady-state gap. Lecture 4's pure-pursuit law adds an integral term to remove it. For the lab a steady gap is acceptable; just explain it. If you want to remove it, add $K_i \int (\rho - d^*)\,dt$ as in slide 13, with anti-windup (limit the integrator).

### 7.4 Plots and report

XY plot of the desired path and the actual path together, `axis equal`; $x, y, \theta$ versus time; optional heading arrows. Explain how you generated the targets, your switch radius or carrot speed, and why the corners are rounded.

---

## 8. Task 3: follow a line

**Goal.** Follow $y = ax + b$ for (a) $a = 2, b = 0$ and (b) $a = 0.5, b = 4$.

### 8.1 Put the line in the controller's form

Lecture 4's line follower (slide 11) wants the line as $Ax + By + C = 0$. Move everything to one side of $y = ax + b$:

$$a x - y + b = 0 \quad\Rightarrow\quad A = a,\ B = -1,\ C = b.$$

Line (a): $2x - y = 0$, so $(A, B, C) = (2, -1, 0)$. Line (b): $0.5x - y + 4 = 0$, so $(A, B, C) = (0.5, -1, 4)$.

### 8.2 The two things the controller needs

1. **Signed distance** from the robot $(x, y)$ to the line:
   $$d = \frac{A x + B y + C}{\sqrt{A^2 + B^2}}.$$
   Do **not** take the absolute value. The sign tells the robot which side of the line it is on; without it the controller cannot know whether to turn left or right. ⚠️ The Lecture 4 slide prints this formula with absolute-value bars; for a controller you need the signed version.
2. **Direction of the line:** $\theta^* = \operatorname{atan2}(-A, B)$, which for $B = -1$ gives $\operatorname{atan2}(-a, -1)$; equivalently, since the slope is $a$, $\theta^* = \operatorname{atan2}(a, 1)$ when you want to travel in the $+x$ direction. Line (a): $\theta^* = \operatorname{atan2}(2, 1) = 1.107$ rad (63.4°). Line (b): $\theta^* = \operatorname{atan2}(0.5, 1) = 0.464$ rad (26.6°). Pick the direction of travel deliberately; the other direction is $\theta^* + \pi$.

### 8.3 The control law

$$\omega = -K_d\, d + K_h\,\operatorname{wrap}(\theta^* - \theta), \qquad v = \text{constant (e.g. } 0.1\text{ m/s)}.$$

The first term steers you *toward* the line (if you are on the positive side, $d > 0$, you turn negative), the second term steers you *parallel* to it. You need both: distance alone overshoots and zigzags, heading alone drives parallel but never closes the gap.

**Worked numbers, line (b).** Robot at $(0, 2, 0)$, $K_d = 0.5$, $K_h = 1$. $d = (0.5\cdot 0 - 2 + 4)/\sqrt{1.25} = 2/1.118 = 1.789$ m (robot is below the line, which here counts as positive side). $\theta^* - \theta = 0.464$. $\omega = -0.5\cdot 1.789 + 0.464 = -0.43$ rad/s. Hmm: that turns the robot *right*, away from a line that is above it. The sign of the $K_d$ term depends on which way $B$ was chosen, so **check the sign on the first run**: if the robot steers away from the line, flip the sign of $K_d$ (or equivalently multiply $(A, B, C)$ by $-1$). With the sign fixed, $\omega = +0.89 + 0.464 = 1.36$ rad/s: a firm left turn toward the line, easing off as $d$ shrinks and the heading approaches 26.6°.

**Common mistake:** the robot converges to the line but the plot looks wrong because you forgot that the lab's lines are in *metres* in the odometry frame. $b = 4$ means the line passes 4 m above the origin; with a 2 m lab mat you must start the robot somewhere sensible, for example near $(−6, 1)$ by offsetting the odometry origin, or scale the problem and say so.

### 8.4 Plots and report

Plot the line (`fplot(@(x) a*x + b)` or two points) and the path, `axis equal`; $x, y, \theta$ versus time; optional arrows. Show both lines, from at least two starting poses each. Report $(A, B, C)$, the signed-distance formula, $\theta^*$, the gains, and comment on overshoot.

---

## 9. Task 4: LiDAR data registration

**Goal.** (a) Plot the current scan and find the sensor's minimum and maximum range. (b) Build a point-cloud map while driving the square from Task 2a, and explain why turning distorts it.

### 9.1 What is in a LaserScan message

The `/scan` bus has: `ranges` (an array, one distance per beam, in metres), `angle_min` and `angle_increment` (the angle of beam 0 and the step between beams, in radians, in the sensor frame), `range_min` and `range_max` (the sensor's valid interval), plus intensities and timing fields. For the LDS-01 there are 360 beams, one per degree, starting at the robot's $+x$ axis and going anticlockwise. Beams that saw nothing report 0 or `Inf`.

### 9.2 Polar to Cartesian in the robot frame

For beam $i$: $\phi_i = \texttt{angle\_min} + i\cdot\texttt{angle\_increment}$, and

$$ {}^B x_i = r_i\cos\phi_i, \qquad {}^B y_i = r_i\sin\phi_i. $$

Discard beams with $r_i < \texttt{range\_min}$, $r_i > \texttt{range\_max}$, $r_i = 0$ or $r_i = \infty$ *before* plotting, or your map fills with points at the origin and at infinity. In MATLAB, `rosReadCartesian(scan)` does the conversion and the filtering in one call (see the Lecture 5 script); in Simulink you do it with a MATLAB Function block.

For part (a), drive toward a wall until the points at the front disappear and read off the distance: that is `range_min` in practice (around 0.12 m for this sensor). Then back away in a corridor until the points vanish: `range_max` (around 3.5 m). Report both and compare with the values in the message. ⚠️ Those two numbers are from the sensor's public specification, not from the handout; verify them on your robot.

### 9.3 Registration: moving points into the world frame

A point in the robot frame is only meaningful together with where the robot was when it measured it. The handout's hint, "associate them with the robot pose", is Lecture 1 in one line: apply the robot's pose $(x, y, \theta)$ from `/odom` as a 2D homogeneous transform.

$$ \begin{pmatrix} {}^W x_i \\ {}^W y_i \\ 1 \end{pmatrix} = \begin{pmatrix} \cos\theta & -\sin\theta & x \\ \sin\theta & \cos\theta & y \\ 0 & 0 & 1 \end{pmatrix} \begin{pmatrix} {}^B x_i \\ {}^B y_i \\ 1 \end{pmatrix}. $$

In words: rotate the scan point by the robot's heading, then shift it by the robot's position. **Worked example.** Robot at $(1, 2)$ heading $90°$ ($\theta = \pi/2$); a beam straight ahead ($\phi = 0$) with $r = 0.5$ m gives ${}^B p = (0.5, 0)$. Then ${}^W x = \cos(90°)\cdot 0.5 - \sin(90°)\cdot 0 + 1 = 1$, ${}^W y = \sin(90°)\cdot 0.5 + 0 + 2 = 2.5$. The obstacle is half a metre *north* of the robot, as it should be since the robot faces north. Append every transformed scan to a growing array (a To Workspace block with "append", or a MATLAB Function block with a persistent array) and plot the lot with `axis equal` together with the robot's path.

### 9.4 Why turning distorts the map, and two fixes

One sweep of the LDS-01 takes about 0.2 s (it spins at roughly 5 Hz). While the robot rotates at, say, 1 rad/s, the heading changes by about 0.2 rad, 11°, between the first and the last beam of the sweep. But you stamp the whole sweep with one pose, so the beams measured early and late are rotated by the wrong angle. Straight walls come out as arcs and corners smear. Two remedies:

1. **Register only when the robot is not turning** (gate on $|\omega| < 0.05$ rad/s, or on $|v| < 0.02$ and $|\omega| < 0.02$ to stop-and-scan at each corner of the square).
2. **De-skew**: interpolate the pose for each beam from the odometry at the sweep's start and end (the message carries `time_increment` per beam), and transform each beam with its own pose.

Odometry drift adds a slower distortion on top: after a full square the map's start and end do not quite line up. That is the problem Lecture 10 solves with SLAM; here you only need to point it out.

### 9.5 Plots and report

Part (a): a single scan in the robot frame, `axis equal`, with min and max ranges marked. Part (b): the registered point cloud and the robot's path on one XY plot, `axis equal`; a second plot with and without the turning gate makes the distortion argument visually.

---

## 10. Task 5: keep a distance from a wall

**Goal.** Drive along a wall at constant distance, (a) straight, (b) uneven, and keep working when obstacles are added or removed between runs.

### 10.1 Turn the scan into a wall estimate

1. **Pick a sector.** If you follow the wall on your left, take the beams between about 45° and 135° (left side). On an uneven wall widen or narrow this.
2. **Filter** out invalid ranges as in Task 4.
3. **Estimate the wall.** Two options, from simple to robust:
   - *Nearest point:* the minimum range $d_{\min}$ in the sector and its angle $\phi_{\min}$. The wall is perpendicular to that beam, so the wall's direction in the robot frame is $\phi_{\min} - \pi/2$ (for a wall on the left).
   - *Line fit:* convert the sector's points to Cartesian in the robot frame and fit a line, either through two well-separated points or by least squares (`polyfit` on $x, y$, or a MATLAB Function block with a 2×2 solve). The fit gives you the wall's slope and your perpendicular distance to it directly.

### 10.2 Control: it is the line follower again

Once you have a wall line, Task 5 is Task 3 with a line that is re-estimated every sample and expressed in the robot frame:

$$ e_d = d_{\text{wall}} - d^*, \qquad e_\theta = \theta_{\text{wall}}, \qquad \omega = K_d\, e_d + K_h\, e_\theta, \qquad v = \text{constant}. $$

$d^*$ is the distance you want to keep (say 0.4 m). $\theta_{\text{wall}}$ is the wall's direction *relative to the robot*, so the heading error is the wall angle itself: zero when you are parallel. If you are too far from the wall ($e_d > 0$) the controller turns you toward it; sign conventions depend on which side the wall is, so check on the first run exactly as in Task 3.

**Everyday analogy:** walking along a corridor with your hand on the wall in the dark. Your hand tells you two things, how far the wall is and whether it is angling toward or away from you, and you correct both at once.

### 10.3 Making it survive an uneven wall

- **Look ahead.** Fit the line to a sector slightly in front of the robot (say 60° to 120° instead of 45° to 135°) so the controller reacts to what is coming, not what has passed. The handout's Figure 9 path is smoother than the wall for exactly this reason.
- **Filter.** Low-pass the estimated distance and angle (a Discrete Filter block, or an exponential average $\hat d \leftarrow 0.8\hat d + 0.2 d$) so a single protruding box does not kick the steering.
- **Reject outliers.** Points much closer than the fitted line (an added obstacle) can be dropped from the fit, or you can simply let the nearest-point method react to them, which gives natural obstacle avoidance at the cost of a wobble.
- **Slow down when the wall curves.** Scale $v$ down with $|e_\theta|$.

In Lecture 7 Part I terms, this is the **follow-a-wall** primitive behaviour of a potential field, and it is also what the Bug algorithms do when they hit an obstacle. The handout's requirement to plot "the wall points used by the method, the current control target and the robot's path" is asking you to show the sector points, the fitted line or nearest point, and the path, all on one `axis equal` XY plot in the world frame (transform them with Task 4's method).

---

## 11. Plots and the report

The report template is the same for all four labs: about 1.5 pages of text plus figures, one zip with the report, all code and recorded videos, named with your group number. Sections: Objective (tasks, goals, and the difficulties you *expected*), Methodology (the method and the equations and code you used), Results (plots, descriptions, the difficulties you *actually* met, and what you learned).

Every task asks for an XY plot with **axis equal** ("correct ratio, no stretching"), and the control tasks also ask for $x$, $y$ and yaw against time. Log with To Workspace blocks and plot in a script so the figures are reproducible. Mark starts, targets and lines; label axes in metres and seconds. Record a short video of each demo with your phone; the template asks for videos.

---

## 12. Debugging checklist

| Symptom | Likely cause | Fix |
|---|---|---|
| Simulink sees no topics | wrong Domain ID, or `run` not running | Domain ID = 3X; check the SSH window |
| Robot jerks or stutters | `teleop` still publishing on `/cmd_vel` | Ctrl+C in the teleop window |
| Robot drives away from the target or spins | sign of $\omega$, unwrapped angle, quaternion treated as an angle | wrap the error; convert the quaternion to yaw; check the DYNAMIXEL IDs if forward is backward |
| Robot turns the long way round | no angle wrapping | `atan2(sin e, cos e)` |
| Pirouettes at the goal | $\theta^*$ computed from tiny $dx, dy$ | dead zone on $\rho$ |
| Path looks right but is in the wrong place | odometry origin is where `run` was started | restart `run` with the robot at your reference spot |
| Map drifts or doubles | odometry drift, or registering while turning | gate on $\omega$; restart `run`; accept and explain |
| Controller outputs are frozen | Enable port false, or STOP slider still set | check Enable; flip the slider |
| Commands larger than the robot moves | firmware saturation at 0.22 m/s and 2.84 rad/s | add your own Saturation blocks and lower gains |

### What the demo examiner is likely to ask

- Write down your control law and point to each term in the model.
- What happens to your point controller if the target is directly behind the robot?
- Why is $K_\beta$ negative in the pose controller?
- Why are the corners of your square rounded, and how would you sharpen them?
- Why does the distance to the line have to be signed?
- Show the rotation matrix you used to register the LiDAR points; which frame is which?
- Why does the map smear when you turn, and what did you do about it?
- What is the LiDAR's minimum range and what happens to beams closer than that?
- Where does `/odom` come from, and why does it drift?

---

## In a nutshell

Lab 2 puts Lecture 4's four controllers onto the TurtleBot3 Burger through a Simulink template that subscribes to `/odom` and `/scan`, runs your Proportional Controller subsystem, and publishes a Twist on `/cmd_vel`, with a STOP slider that zeroes everything downstream of your code. Move-to-point is two P loops on distance and wrapped heading error with saturation and a dead zone; move-to-pose adds the polar $\rho, \alpha, \beta$ law with a negative $K_\beta$; trajectories are the point controller chasing a sequenced or time-moving target; line following needs the signed distance and the line's direction; and the two LiDAR tasks are polar-to-Cartesian conversion, a 2D homogeneous transform into the world frame, and a line follower driven by a wall estimate from a sector of the scan. Bring-up is Wi-Fi, SSH, `run`, Domain ID 3X, and no teleop while your controller runs. Every plot needs `axis equal`, and the report is 1.5 pages plus figures, code and videos.

## Key terms

- **Simulink:** MATLAB's block-diagram programming environment.
- **Block / signal / sample time:** a processing step, the wire between steps, and how often the model updates.
- **Bus / Bus Selector / Bus Assignment:** a structured signal, picking fields out of it, writing fields into it.
- **Subscribe / Publish / Blank Message:** the ROS 2 blocks that receive, send and create messages.
- **Enabled subsystem:** a block group that only runs while its Enable input is true.
- **STOP slider:** the template's emergency stop; routes zero to both velocity outputs.
- **P (proportional) controller:** command equals gain times error.
- **atan2:** four-quadrant arctangent; gives the direction of a vector without dividing by zero.
- **Angle wrapping:** forcing an angle error into $[-\pi, \pi]$.
- **Saturation:** clipping a command to the robot's limits (0.22 m/s, 2.84 rad/s).
- **Dead zone:** a small region around the goal where the command is set to zero.
- **ρ, α, β:** distance to goal, heading error to the goal line, angle between that line and the goal heading.
- **Waypoint sequencing / switch radius:** aiming at list entries in turn, advancing when close.
- **Donkey and carrot (pure pursuit):** chasing a target that moves along the path.
- **Signed distance to a line:** $(Ax + By + C)/\sqrt{A^2 + B^2}$ without absolute value.
- **LaserScan:** the `/scan` message; `ranges`, `angle_min`, `angle_increment`, `range_min`, `range_max`.
- **Registration:** transforming scan points into the world frame with the robot's pose.
- **De-skew:** correcting a scan for motion during the sweep.
- **Odometry drift:** slow growth of error in `/odom` from wheel slip and integration.
- **Domain ID:** the ROS 2 "channel" (3X for robot X).

## Check yourself

1. Your robot reaches the target point but then spins on the spot. What is happening and what is the fix?
2. For the line $y = 0.5x + 4$, write the $(A, B, C)$ form, the signed distance of the point $(2, 1)$, and the travel heading $\theta^*$.
3. Why must $K_\beta$ be negative in the move-to-pose law, in plain words?
4. A beam of length 2 m at scan angle 90° is measured with the robot at $(3, 1)$ heading 0. Where is the obstacle in the world frame?
5. Name two reasons a LiDAR map built while driving a square does not close up neatly, and one remedy for each.

<details>
<summary>Answers</summary>

1. Inside a few centimetres of the goal, $\theta^* = \operatorname{atan2}(dy, dx)$ is computed from tiny, noisy $dx, dy$ and flips randomly, so the heading controller chases it. Add a dead zone: when $\rho$ is below a few centimetres, set $v = \omega = 0$ (or freeze $\theta^*$).
2. $0.5x - y + 4 = 0$, so $(A, B, C) = (0.5, -1, 4)$. Signed distance of $(2, 1)$: $(1 - 1 + 4)/\sqrt{1.25} = 4/1.118 = 3.58$ m (positive side). Travel heading $\theta^* = \operatorname{atan2}(0.5, 1) = 0.464$ rad, about 26.6°.
3. $\beta$ measures how wrong the final heading will be if you drive straight in. A negative gain makes the robot steer *away* from the goal heading early so that its approach curves round and arrives pointing the right way, like swinging out before parking. With a positive gain it arrives at the spot facing the wrong way.
4. Robot frame: $(2\cos 90°, 2\sin 90°) = (0, 2)$. Heading 0 means no rotation, so world point $= (3 + 0, 1 + 2) = (3, 3)$. The obstacle is 2 m to the robot's left, which with heading 0 is $+y$.
5. (i) Scans taken while turning are stamped with one pose although the heading changed during the sweep, which smears walls; register only when $\omega \approx 0$, or de-skew each beam with an interpolated pose. (ii) Odometry drifts, so the pose used for later scans is wrong; restart `run` to reset, drive slowly to reduce slip, and note that a full fix needs SLAM (Lecture 10).

</details>
