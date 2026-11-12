# Lecture 7 (Part I) – Robot Navigation: Reactive Navigation

*Study notes written from the slide deck `R7010E_Lecture_7_Navigation_2023.pdf` (slides 1–41) and the MATLAB script `R7010E_Lecture_7_Navigation.m`. Part II (map-based planning, slides 42–89) lives in the folder "Lecture 7 - Navigation II".*

---

## What this lecture is about, and why it matters

Up to now you have learned how to describe where a robot is (pose), how to move it smoothly along a trajectory, and how to control a wheeled robot or a drone. This lecture asks the next natural question: **how does a robot decide where to go when there are obstacles in the way?** That is the problem of *navigation*. In this first half, you look at *reactive* navigation, where the robot has no map and simply reacts to what its sensors tell it right now, using artificial potential fields and the family of "Bug" algorithms. These ideas are the backbone of every real collision-avoidance system, from a TurtleBot in the lab to drones flying in mines.

## Roadmap: the main topics in this part

1. What navigation is, and how it differs from control and trajectory generation
2. The three fundamental goals of navigation and the two big families: reactive and map-based
3. Known maps vs unknown maps
4. Reactive / behaviour-based navigation: the Braitenberg vehicle
5. Artificial Potential Fields (APF): attractive and repulsive potentials, forces as gradients
6. The practical Δx/Δy rules for attraction and repulsion
7. Primitive field types and the behaviours they create
8. The "Lindqvist" APF for LiDAR sensors and its ground-robot control law
9. Bug algorithms: assumptions, hit/leave points, Bug0, Bug1, Bug2, improved Bug2
10. Completeness of a path planner
11. Bug2 in MATLAB and in ROS
12. Range segmentation and the Tangent Bug

---

## 1. What is navigation? (slides 2–4)

Let's start with the simplest picture. The robot is at point A and wants to be at point B. **Navigation** is everything the robot has to do to actually get from A to B through a world that contains obstacles.

The slides open with two questions that you should be able to answer in your own words.

**Question 1: What is the difference between path planning and control?**

Think of a road trip. *Path planning* is deciding the route on the map before (or while) you drive: which streets, which turns, how to get around the closed bridge. *Control* is the act of actually steering the car and pressing the pedals so the car follows that route. In Lecture 4 you built controllers that drive a robot to a point, along a line, or along a path. Those controllers *follow* something. Path planning is what *produces* the thing to follow. Control makes the wheels do it, path planning decides what "it" is.

**Question 2: What is the difference between path planning and the trajectory generation you did in Lecture 3?**

In Lecture 3, trajectory generation meant taking a start and an end (and maybe a few via points) and producing a smooth time-indexed motion between them, with nice velocity and acceleration profiles. Importantly, it *assumed the route was already free of obstacles*. Path planning is the step before: it finds a geometric route that avoids the obstacles in the first place. Afterwards you can turn that route into a smooth trajectory. So the order is: **plan the path → generate a trajectory along it → control the robot to track it.**

### The three fundamental goals (slide 4)

The slides summarise "how a robot moves from A to B" as three sub-tasks:

1. **Find the path from A to B in a map.** This is called **map-based navigation**.
2. **Avoid obstacles encountered along the way.** This is called **reactive navigation**.
3. **Track the path.** This is control, which you already know.

A lot of people think navigation is only step 1. In practice you almost always need both 1 and 2: a global plan from a map plus local reactions to things that are not in the map (a person walking by, a chair that moved).

---

## 2. Environment awareness and map knowledge (slides 5–7)

Humans have automatic perception of the surroundings and an almost built-in path planning ability. You walk around a table without thinking. A robot has none of that. The slides list three ways to give it that ability:

- **Perception** through robot vision and obstacle detection (sensors tell the robot what is around it).
- **Human-provided waypoints** to follow (a person draws the route).
- **Algorithms** that let the robot navigate autonomously (this lecture and the next).

Then the slides separate two situations:

**Known map.** If you already have a map, navigation is comparatively easy: provide a series of waypoints, follow them with a position controller (like the "move to a point" controller of Lecture 4), and add some obstacle avoidance for safety.

**Unknown map.** This is *very challenging*. The robot can only act on its current sensor information. Picture the robot inside a grey fog: it sees only a small bright circle around itself, and everything else is question marks. It has to decide what to do with only that.

---

## 3. Reactive vs map-based navigation (slide 8)

Here is the key comparison. Learn it as a table.

| | **Reactive navigation** | **Map-based navigation** |
|---|---|---|
| Needs a global map? | No | Yes |
| Needs localization (knowing where you are in the map)? | No (or only roughly) | Yes |
| What it is good at | Reacting immediately to sensed obstacles | Finding optimally short global routes |
| Guarantees | Few; it can get stuck | "Guarantees" on finding a path if one exists |
| Coupled with | Perception and control systems, very tightly | A map representation and a search algorithm |

An everyday analogy: reactive navigation is walking through a crowded market by dodging people as they appear. Map-based navigation is looking at the city map, choosing the shortest route, and then walking it. Real robots do both: the map gives the route, the reactive layer dodges the surprises. (You will see this as the "two-layer architecture" at the very end of Part II.)

The slides show two videos: a drone (UAV) flying through 1-metre openings between wall-like obstacles using only reactive avoidance, and a car in a simulated building with a smooth planned corridor. These are the two flavours in action.

### What this lecture covers (slide 11)

- *This lecture:* reactive navigation with Artificial Potential Fields, and the Bug algorithm.
- *Next lecture (Part II):* map-based navigation with occupancy grids, grid search, graph/roadmap methods, and tree-based methods.

---

## 4. Reactive / behaviour-based navigation: the Braitenberg vehicle (slide 12)

Your first reactive robot is the **Braitenberg vehicle**. A Braitenberg vehicle is a toy robot with two light sensors (left and right) wired directly to the steering and speed. There is no map, no planning, no memory. Yet it produces behaviour that looks "intelligent", like driving towards a light and slowing down when it gets there.

### The sensor field

The thing being sensed is a smooth scalar field over the plane. In MATLAB the slides build it like this:

```matlab
xc = 60; yc = 90;
[X, Y] = meshgrid(1:1:100, 1:1:100);
sens = 200 ./ ((X - xc).^2 + (Y - yc).^2 + 200);
surf(X, Y, sens)
```

In words: at every point $(x, y)$ of a 100×100 world, the sensor value is

$$ s(x,y) = \frac{200}{(x - x_c)^2 + (y - y_c)^2 + 200}, \qquad s(x,y) \in [0,1] $$

It equals 1 exactly at the "source" $(x_c, y_c)$ and falls off smoothly the further you go. Think of it as the brightness of a lamp placed at $(x_c, y_c)$. (The `.m` file in the folder uses $x_c = y_c = 50$; the slide uses 60 and 90. Both are just examples.)

### The control law

The robot has a right sensor reading $s_R$ and a left sensor reading $s_L$, each sampled at the position of the respective sensor on the robot. The Simulink model `sl_braitenberg` wires them as:

$$ \text{Velocity: } \quad V = 2 - s_R - s_L $$
$$ \text{Steering: } \quad \gamma = k\,(s_R - s_L) $$

Read these slowly:

- **Velocity.** Far from the source both sensors read near 0, so $V \approx 2$ (full speed). As the robot approaches the source both sensors rise towards 1, so $V \to 0$. The robot *slows down as it arrives*. The constant 2 is the maximum speed, you can see it as the "2" block in the Simulink diagram.
- **Steering.** If the right sensor sees more than the left ($s_R > s_L$), the source is to the right, and $\gamma > 0$ steers right (for this model's sign convention). If the left sees more, it steers left. The gain $k$ (5 in the Simulink model) decides how aggressively it turns. The difference $s_R - s_L$ is zero when the robot is pointing straight at the source.

The result, shown in the slide's bottom-right plot, is a smooth curved path from the bottom-left corner that homes in on the source and stops there.

> **Common mistake.** People think the Braitenberg vehicle "knows" where the goal is. It does not. It only knows two numbers, right now. Everything that looks like planning emerges from the wiring. That is the whole point of *behaviour-based* navigation.

---

## 5. Artificial Potential Fields (APF) (slides 13–14)

The Braitenberg vehicle is attracted to one thing. Real environments have a goal *and* obstacles. **Artificial Potential Fields** (APF) generalise the idea: the robot is treated as a particle in an imaginary force field where the goal pulls and the obstacles push. This is **online collision avoidance**: it runs continuously while the robot moves.

### Configuration space

The slides say we "build potential fields in the C-space". The **configuration space** (**C-space**) is the space of all possible configurations $q$ of the robot. For a point robot in the plane, $q = (x, y)$. Obstacles mapped into this space are called **C-obstacles** (CO). The goal configuration is $q_{goal}$.

### The idea in one sentence

Define a scalar function $U(q)$ (the **potential**) over the C-space so that it is *low at the goal* and *high near obstacles*. Then let the robot "roll downhill".

Picture a landscape: the goal is the bottom of a bowl, every obstacle is a hill. Drop a marble anywhere and it rolls down to the goal, going around the hills.

### The maths (slide 14)

The environment is represented by a potential function $U(x,y)$. The force acting on the robot is the **negative gradient** of the potential:

$$ F(q) = -\nabla U(q), \qquad \nabla U(q) = \begin{bmatrix} \partial U / \partial x \\ \partial U / \partial y \end{bmatrix} $$

The **gradient** $\nabla U$ is the vector that points in the direction of steepest *increase* of $U$ and has length equal to how steep it is. The minus sign makes the force point *downhill*. For a simple point robot we treat the force as the velocity command:

$$ \begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix} = -\nabla U(x, y) $$

This is "incremental path planning": at every instant you compute the force at the current position and move a little in that direction. No global plan is ever computed.

The total potential is the sum of an attractive part and a repulsive part:

$$ U(q) = U_{att}(q) + U_{rep}(q), \qquad \vec F(q) = -\nabla U_{att}(q) - \nabla U_{rep}(q) $$

Because gradients add, the total force is simply the attractive force plus the repulsive force. That is what makes this so easy to implement.

### Attractive potentials

Two standard choices are given on the slide. Here $\xi$ (xi) is a positive gain and $\|q - q_{goal}\|$ is the straight-line distance to the goal.

**Linear (conic) potential:**
$$ U_a(q) = \xi \,\|q - q_{goal}\|, \qquad F_a(q) = -\nabla U_a(q) = -\xi\, \frac{q - q_{goal}}{\|q - q_{goal}\|} $$
The force has *constant magnitude* $\xi$ and always points straight at the goal. Like a tractor beam that pulls equally hard no matter how far you are.

**Quadratic (parabolic) potential:**
$$ U_a(q) = \tfrac{1}{2}\,\xi\, \|q - q_{goal}\|^2, \qquad F_a(q) = -\nabla U_a(q) = -\xi\,(q - q_{goal}) $$
The force grows *linearly with distance*: far away it pulls hard, close to the goal it pulls gently. Like a spring attached between robot and goal.

> **Which to use?** The quadratic version is smooth at the goal (no chattering) but can produce huge forces far away. The linear version is bounded but has a sharp "kink" at the goal. Many practical implementations use quadratic near the goal and linear far away, exactly what the Δx/Δy rules in the next section do.

### Repulsive potential

$$ U_{rep}(q) = \begin{cases} \tfrac{1}{2}\,\eta \left( \dfrac{1}{\rho(q)} - \dfrac{1}{\rho_0} \right) & \text{if } \rho(q) \le \rho_0 \\[2mm] 0 & \text{if } \rho(q) > \rho_0 \end{cases} $$

where

- $\rho(q)$ is the **minimum distance** from the robot to the obstacle,
- $\eta$ (eta) is a positive scaling factor (how strongly obstacles push),
- $\rho_0$ is the **distance of influence**: outside this radius the obstacle is simply ignored.

Read it as: the closer you get ($\rho \to 0$), the term $1/\rho$ blows up, so the potential goes to infinity and the robot is pushed away hard. At exactly $\rho = \rho_0$ the potential is zero, and it stays zero further out. So each obstacle has a "bubble" of radius $\rho_0$ around it, and only inside the bubble does it matter.

> ⚠️ **Check with your lecturer.** The classic Khatib formulation of the repulsive potential has the bracket **squared**: $U_{rep} = \tfrac12 \eta \left(\tfrac{1}{\rho} - \tfrac{1}{\rho_0}\right)^2$. The slide prints it without the square. The squared form is what the referenced source (the Koç University notes linked on the slide) and most textbooks use, and it is the one that gives the standard repulsive force formula. The qualitative behaviour is the same either way.

---

## 6. The practical Δx/Δy rules (slides 15–17)

The potential formulas above are elegant but a bit abstract. The slides then give a very concrete "recipe" version, where at each step you compute a small displacement $(\Delta x, \Delta y)$ that the robot should take. Here are the symbols:

- $(x_G, y_G)$: goal position; $(x_O, y_O)$: position of an obstacle
- $\mathbf v = [x, y]^T$: the robot's current position
- $r$: the radius of the object (goal or obstacle). Treat the goal and the obstacles as circles of radius $r$.
- $s$: the range (spread) of influence beyond the object's radius
- $\alpha > 0$ and $\beta > 0$: gains for attraction and repulsion

### Attractive rule (slide 15)

**Step 1.** Find the distance between robot and goal:
$$ d = \sqrt{(x_G - x)^2 + (y_G - y)^2} $$

**Step 2.** Find the angle from robot to goal, using the two-argument arctangent `atan2` so the quadrant is right:
$$ \theta = \text{atan2}\,(y_G - y,\; x_G - x) $$

> ⚠️ The slide prints $\theta = \tan^{-1}\!\big(\frac{x_G - x}{y_G - y}\big)$ with $x$ on top and $y$ underneath. With the usual convention where $\theta$ is measured from the $x$-axis and $\Delta x$ uses $\cos\theta$, it should be $\text{atan2}(y_G - y,\, x_G - x)$. The slide itself hints at this by writing "atan2" next to the formula. Use `atan2(yG - y, xG - x)` in code.

**Step 3.** Set $\Delta x$ and $\Delta y$ by cases:

| Case | Meaning | $\Delta x$ | $\Delta y$ |
|---|---|---|---|
| $d < r$ | Robot is inside the goal circle. Done. | $0$ | $0$ |
| $r \le d \le s + r$ | Robot is in the "spring zone" | $\alpha\,(d - r)\cos\theta$ | $\alpha\,(d - r)\sin\theta$ |
| $d > s + r$ | Robot is far away | $\alpha\, s \cos\theta$ | $\alpha\, s \sin\theta$ |

In the middle zone the pull grows with distance (that is the quadratic potential). Far away the pull is capped at a constant $\alpha s$ (that is the linear potential). So this recipe *is* the "quadratic near, linear far" combination mentioned above.

**Worked example.** Robot at $(2, 1)$, goal at $(6, 4)$, $r = 0.5$, $s = 2$, $\alpha = 1$.
- $d = \sqrt{(6-2)^2 + (4-1)^2} = \sqrt{16 + 9} = 5$.
- $\theta = \text{atan2}(3, 4) \approx 36.9°$, so $\cos\theta = 0.8$, $\sin\theta = 0.6$.
- $s + r = 2.5$ and $d = 5 > 2.5$, so we are in the far zone: $\Delta x = 1 \cdot 2 \cdot 0.8 = 1.6$, $\Delta y = 1 \cdot 2 \cdot 0.6 = 1.2$.
- The robot moves 1.6 in $x$ and 1.2 in $y$, straight at the goal with step length 2.

### Repulsive rule (slide 16)

Same structure, obstacle instead of goal, and the signs flipped.

**Step 1.** $d = \sqrt{(x_O - x)^2 + (y_O - y)^2}$ (distance robot to obstacle).

> ⚠️ Slide 16 copies the formula from slide 15 and still writes $x_G, y_G$ inside the square root. It should of course be the obstacle coordinates $x_O, y_O$.

**Step 2.** $\theta = \text{atan2}(y_O - y,\; x_O - x)$ (angle from robot towards the obstacle).

**Step 3.** Cases:

| Case | Meaning | $\Delta x$ | $\Delta y$ |
|---|---|---|---|
| $d < r$ | Robot is *inside* the obstacle (collision!) | $-\text{sign}(\cos\theta)\,\infty$ | $-\text{sign}(\sin\theta)\,\infty$ |
| $r \le d \le s + r$ | Robot is in the influence bubble | $-\beta\,(s + r - d)\cos\theta$ | $-\beta\,(s + r - d)\sin\theta$ |
| $d > s + r$ | Obstacle is too far to matter | $0$ | $0$ |

Note the minus signs: $\theta$ points *towards* the obstacle, and we move the *opposite* way. In the bubble, the push is strongest right at the obstacle surface ($d = r$ gives $\beta s$) and fades to zero at the edge of the bubble ($d = s + r$). The infinite push inside the obstacle is a mathematical way of saying "never, ever be here".

**Worked example.** Robot at $(2, 1)$, obstacle at $(3, 1)$, $r = 0.5$, $s = 1$, $\beta = 1$.
- $d = 1$, $\theta = \text{atan2}(0, 1) = 0$, so $\cos\theta = 1$, $\sin\theta = 0$.
- $r \le 1 \le 1.5$: middle zone. $\Delta x = -1 \cdot (1 + 0.5 - 1) \cdot 1 = -0.5$, $\Delta y = 0$.
- The robot is pushed half a unit to the left, directly away from the obstacle.

### Combining them (slide 17)

You simply add the contributions:

$$ \Delta x = \Delta_O x + \Delta_G x, \qquad \Delta y = \Delta_O y + \Delta_G y $$

where subscript $O$ is from the obstacle(s) and $G$ from the goal. With several obstacles you add one repulsive term per obstacle. The slide's figure shows the resulting vector field (arrows swirl around the obstacle and converge on the goal) and the path a robot takes: it bends smoothly around the obstacle and lands on the goal.

Continuing the two worked examples together: $\Delta x = 1.6 + (-0.5) = 1.1$, $\Delta y = 1.2 + 0 = 1.2$. The robot still heads roughly towards the goal, but shifted away from the obstacle.

> ⚠️ **The local-minimum problem (not stated explicitly on the slides, but exam-relevant).** Because the attractive and repulsive forces can cancel, the robot can get stuck at a point that is not the goal, typically right behind a U-shaped obstacle or exactly between two obstacles. At that point $\nabla U = 0$ so the robot simply stops. This is the main weakness of pure potential fields and the reason map-based methods exist. The Bug algorithms in Section 9 are one way to escape: when stuck, follow the wall.

---

## 7. Primitive types of potential fields (slide 18)

Potential fields are not only for "go to goal, avoid obstacle". By choosing simple vector fields you can build a whole library of *behaviours*. The slide lists five primitives:

| Primitive | Field | Behaviour it creates |
|---|---|---|
| **Uniform** | $\Delta x = \pm c$, $\Delta y = 0$: every arrow points the same way | Follow a wall; return to territory |
| **Perpendicular** | $\Delta x = 0$, $\Delta y = \pm c$: arrows point straight away from a line | Avoid a wall; avoid a territory |
| **Attraction** | Arrows converge on a point | Go to a goal |
| **Repulsion** | Arrows diverge from a point | Avoid an object |
| **Tangential** | Arrows circle around a point | Patrol a flag; inspect a structure |

An analogy: these are like Lego bricks. "Follow the corridor and inspect the pillar" is a uniform field along the corridor plus a tangential field around the pillar. Add the fields, roll downhill, done.

---

## 8. The "Lindqvist" APF for LiDAR sensors (slides 19–23)

The textbook potential fields assume you know where each obstacle *is* as a geometric object. A real robot has a **LiDAR** (a laser range scanner that returns a cloud of points where the laser hit something). Björn Lindqvist, one of the course's lab assistants, formulated an APF that works directly on those points. It is based on the classic formulation of Warren (1989).

### The repulsive force

Let each LiDAR point $\rho^i_F$ (a vector from the robot to the $i$-th hit point, expressed in the LiDAR frame) that lies inside a radius $r_F$ generate a repulsive force, and sum them:

$$ F^r = \sum_{i=1}^{N_{\rho_F}} L^r \left( 1 - \frac{\|\rho^i_F\|}{r_F} \right)^2 \frac{-\rho^i_F}{\|\rho^i_F\|} $$

Piece by piece:

- $N_{\rho_F}$: the number of LiDAR points inside the radius $r_F$.
- $L^r$: the repulsive gain (how hard each point pushes).
- $\left(1 - \frac{\|\rho^i_F\|}{r_F}\right)^2$: a weight that is 1 when the point touches the robot and 0 at the edge of the radius. Squaring it makes close points matter much more than far ones.
- $\frac{-\rho^i_F}{\|\rho^i_F\|}$: a unit vector pointing *away* from the hit point.

So: every nearby laser hit pushes the robot away from itself, close hits push hard, and all pushes are summed. Notice this is exactly the "middle zone" repulsive rule from Section 6, but applied to hundreds of points at once and with a squared fall-off.

### The attractive force

The slide defines it "simply as"

$$ F^a = {}^{w}p^{\mathcal B} - \hat p^{\mathcal B} $$

that is, the vector from the robot's estimated position $\hat p$ to the waypoint $p$, expressed in the body frame $\mathcal B$. In plain words: *pull straight towards the waypoint*.

> ⚠️ The slide does not define the symbols $w$, $\mathcal B$, or the hat. The reading above (waypoint minus current estimated position, in the body/LiDAR frame) is the natural one and matches the rest of the slide, but confirm with your lecturer.

### Two ways to use it

**Option A: shift the waypoint and run the normal controller (slide 19).** Rotate the repulsive force from the LiDAR frame into the global frame, then add it to the waypoint:

$$ F^{r,G} = \text{rot2}(F^r, \theta), \qquad p_{ref} = wp + F^{r,G} $$

Here `rot2(·, θ)` is the 2-D rotation by the robot's heading $\theta$ (you know this function from Lecture 1). The repulsive force nudges the reference position away from obstacles, and your existing position controller does the rest. This is elegant because it leaves the controller untouched.

**Option B: a control law for a ground robot (slide 20).** For a unicycle-type robot with forward speed $u_v$ and turn rate $u_w$:

$$ u_v = K_d\, d + F^r_x $$
$$ u_w = K_w\,(\theta_{ref} - \theta) + F^r_y\, \text{sign}(u_v) $$

- $K_d\, d$: drive forward proportional to the distance $d$ to the waypoint (the "move to a point" law of Lecture 4).
- $F^r_x$: the forward/backward component of the repulsive force slows you down (or backs you up) when something is ahead.
- $K_w (\theta_{ref} - \theta)$: turn towards the desired heading.
- $F^r_y\,\text{sign}(u_v)$: the sideways component of the repulsive force makes you turn away from the obstacle. The sign of $u_v$ is there because if you are reversing, "turn away" means the opposite steering direction.

The slides end this section with three videos from the LTU Robotics & AI team: an *adaptive 3-D APF for fail-safe UAV navigation*, a drone *exploring unknown tunnel environments* purely reactively, and an office demo where a drone avoids a chair. The message: a well-tuned APF alone is enough for a drone to fly through an unknown tunnel.

---

## 9. Bug algorithms (slides 24–35)

Potential fields can get stuck. **Bug algorithms** are the simplest planners that provably do not get stuck (at least Bug1 and Bug2). They are called "bug" because they behave like an insect crawling along walls.

### Assumptions (slide 24)

- Only **local sensing**: the robot senses obstacles only when it touches them (or, for Tangent Bug, within a short range).
- A **global goal** is known, and so is the **direction to the goal** from anywhere (think of a compass that always points at the goal).
- The workspace is $\mathbb R^2$ or $\mathbb R^3$, and can be infinite (open) or bounded (closed).

### The only two behaviours

Every Bug algorithm combines exactly two primitive behaviours:

1. **Move in a straight line towards the goal.**
2. **Follow a wall** (and you must decide beforehand whether to turn left or right when you hit one; the slides assume a left-turning robot).

### Two important points

- **Hit point** $q^H_i$: the place where the robot first touches the $i$-th obstacle.
- **Leave point** $q^L_i$: the place where the robot leaves the $i$-th obstacle to head for the goal again.

### Bug0 (slides 24, 26–27)

1. Head towards the goal.
2. When you hit an obstacle, follow its boundary until you can head towards the goal again (i.e. the goal direction is free).
3. Repeat.

This is the most naive version, and **it can fail**. Slides 26–27 show a spiral-shaped (C-shaped) obstacle with the goal inside. Bug0 hits the outer wall, follows it, sees "the goal direction is free again" as soon as it rounds a corner, heads off, hits the next wall, and ends up circling the *outside* of the obstacle forever without ever entering the opening. Because it has no memory, it can loop infinitely. Bug0 is therefore **not complete**.

### Completeness (slide 25)

> A path planning algorithm is **complete** if, in finite time, it finds a path if such a path exists, or terminates with failure if it does not.

Both halves matter. A complete planner never runs forever, and it never gives up when a solution exists. Bug0 fails the first half (it may run forever). Bug1 and Bug2 are complete.

### Bug1 (slide 28)

1. Head towards the goal.
2. If you hit an obstacle, **circumnavigate it completely** (go all the way around, back to the hit point) and remember the point on the boundary that was **closest to the goal**.
3. Return to that closest point (by wall following) and leave from there. Repeat.

Here is the slide's pseudocode in plain English:

```
Let q_L0 = q_start, i = 1
repeat
    repeat
        from q_L(i-1), move toward q_goal
    until goal is reached or an obstacle is encountered at q_Hi
    if goal is reached: exit (success)
    repeat
        follow the boundary, recording the point q_Li with the shortest distance to goal
    until q_goal is reached or q_Hi is re-encountered   (full loop around)
    if goal is reached: exit
    go to q_Li (by wall following)
    if moving toward q_goal from q_Li would enter the obstacle:
        exit with FAILURE (goal is unreachable)
    else
        i = i + 1, continue
```

The failure test is what makes Bug1 complete: after a full circumnavigation you know *everything* about that obstacle's boundary. If even the best point on it cannot step towards the goal, no path exists.

**Analogy.** You are blindfolded in a room, holding a string pulled towards the exit. You bump into a table. Bug1 says: walk all the way around the table, feel where the string pulls hardest (closest to the exit), walk back to that spot, and only then let go of the table. Slow, but you never get lost.

Slide 29 is a blank worked example (two blob-shaped obstacles between $q_{start}$ and $q_{goal}$) for you to draw yourself. Try it: head right, hit the first blob, trace it fully, go back to its rightmost point, head right again, hit the second blob, trace it fully, leave from its point closest to the goal.

### Bug2 (slide 30)

Bug2 introduces the **m-line**: the straight line from $q_{start}$ to $q_{goal}$, fixed once at the beginning.

1. Head towards the goal **along the m-line**.
2. If an obstacle is in the way, follow its boundary **until you encounter the m-line again**.
3. Leave the obstacle and continue towards the goal along the m-line.

The m-line acts as a rail. You always return to the same rail, so you never wander off. In most "nice" environments Bug2 is much shorter than Bug1 because you do not have to circle every obstacle completely.

### Bug2 improved (slide 31)

Plain Bug2 has a subtle problem: you might re-meet the m-line at a point that is *further from the goal* than where you hit the obstacle (slide 30's middle picture, where the start is inside a cup-shaped obstacle, shows the kind of geometry where this matters). The improved version adds a condition in red:

2'. Follow the obstacle until you encounter the m-line again **closer to the goal**, at a point that is not the hit point, and from which the way to the goal is unobstructed.

In pseudocode form (from the slide):

```
Let q_L0 = q_start, i = 1
repeat
    repeat
        from q_L(i-1), move toward q_goal along the m-line
    until goal reached or obstacle encountered at q_Hi
    if goal reached: exit
    repeat
        follow boundary
    until goal reached, or q_Hi re-encountered, or
          the m-line is re-encountered at a point x such that
              x ≠ q_Hi,  d(x, q_goal) < d(q_Hi, q_goal),  and the way to the goal is unimpeded
    if goal reached: exit
    if q_Hi reached: return FAILURE
    else: q_Li = x, i = i + 1, continue
```

Again the failure test (you came all the way back to the hit point without finding a better m-line crossing) makes it complete.

### Bug1 vs Bug2 (slide 33)

The comparison figure shows the L-shaped and bar obstacles with Bug1 in orange and Bug2 in red. Bug2 cuts straight through along the m-line, dodging each obstacle just enough; Bug1 traces every obstacle fully and is much longer.

A quick summary for the exam:

| | **Bug1** | **Bug2** |
|---|---|---|
| Strategy | Exhaustive: circle each obstacle fully, leave at the best point | Greedy: leave as soon as you re-find the m-line (closer to goal) |
| Memory needed | Closest point on current boundary | The m-line (start and goal) |
| Typical path length | Longer | Shorter |
| Worst case | Predictable | Can be worse than Bug1 on obstacles the m-line crosses many times |
| Complete? | Yes | Yes |

*Background (not on the slides, but standard):* with $d$ the start–goal distance and $p_i$ the perimeter of obstacle $i$, Bug1's path is at most $d + 1.5\sum_i p_i$, while Bug2's is at most $d + \tfrac12\sum_i n_i p_i$ where $n_i$ is the number of times the m-line crosses obstacle $i$. So for simple convex obstacles Bug2 wins, but for a twisty obstacle that the m-line slices many times, Bug1 can be better.

### Bug2 in MATLAB (slides 34–35, and the `.m` file)

```matlab
load map1                  % a 100x100 occupancy map with red obstacles
bug = Bug2(map);           % create a Bug2 planner on this map
bug.plot()                 % draw the map
start = [20, 10];
goal  = [50, 35];
bug.query(start, goal, 'animate');   % animate the robot following the m-line
bug.display
p = bug.query(start, goal);          % or return the path as an N x 2 matrix
```

and on the house map from the textbook:

```matlab
load house
place                      % a structure of named places: place.br3, place.kitchen, ...
bug = Bug2(house);
bug.plot();
bug.query(place.br3, place.kitchen, 'animate');
```

Slide 35 shows the result: the dashed m-line runs from bedroom 3 to the kitchen, and the green path hugs the walls whenever the m-line is blocked. The slide's bullet points sum up Bug2: *no map is needed* (the map is only used by the simulator to tell the robot when it bumps), *start and goal are known a priori*, and *create a line and follow it towards the goal*.

### Bug algorithms in ROS (slides 36–37)

The repository `github.com/millere/bugs` is a ROS package that implements Bug0, Bug1 and Bug2 in Python for the Stage simulator. You run it with

```
rosrun bugs bug.py bug[0|1|2] destX destY
```

where the number picks the algorithm and `destX destY` is the goal. Slide 37 is a video of Bug0 running. The point for you: the same abstract algorithm can be written in MATLAB or ROS, the concepts are identical.

---

## 10. Range segmentation and the Tangent Bug (slides 38–41)

The plain Bug algorithms "see" an obstacle only when they touch it. Real robots have range sensors (LiDAR, sonar) that see some distance ahead. **Tangent Bug** uses this to make smarter decisions.

### Range segmentation (slide 38)

A **range scan** $\rho(x, \theta)$ is the sensed distance along the ray at angle $\theta$ from the robot at position $x$, up to a maximum range $R$. If you sweep $\theta$ around 360°, you get a function of angle. Where the ray jumps from one obstacle to another, or from an obstacle to "nothing within range", the function has a **discontinuity**. The set of discontinuity points $\{O_i\}$ (the slide also calls them $B_i$) are exactly the *visible edges* of the obstacles. These points segment the scan into intervals of continuity: each interval is either one obstacle surface or free space.

Imagine shining a torch around you in a dark room. The edges of the shadows are the $O_i$. Those edges are where you could slip past an obstacle.

### Tangent Bug (slides 39–41)

Tangent Bug relies on the range sensor $\rho_R$ to compute the endpoints of finite continuous segments on the obstacle boundaries. These segments are its *local model of the world*.

**Motion-to-goal behaviour.** At each step, consider the candidate points $n \in \{\text{goal}, B_1, B_2, \dots\}$. If the straight line to the goal is free, head to the goal. Otherwise, choose the discontinuity point $B_i$ that minimises the heuristic

$$ h(x, n) = d(x, n) + d(n, \text{goal}) $$

that is, the distance from the robot to the edge point plus the distance from the edge point to the goal. This is "which corner should I aim for so that my detour is shortest?" Slide 40 shows the two candidate detours around an elliptical obstacle, $d(x, O_2) + d(O_2, q_{goal})$ versus $d(x, O_4) + d(O_4, q_{goal})$, and the robot picks the smaller sum.

**When to switch to boundary following.** Keep moving towards your chosen $n$ while $h(x, n)$ keeps *decreasing*. The moment it starts *increasing*, you have reached a local minimum of the heuristic: the obstacle is in the way and no visible corner helps. Then switch to behaviour 2.

**Boundary-following behaviour.** Follow the boundary in the same direction as before, and keep updating two distances:

- $d_{followed}$: the shortest distance between the *sensed boundary you are following* and the goal.
- $d_{reach}$: the shortest distance between the *blocking obstacle* and the goal (or simply the distance to the goal if nothing visible blocks it).

Stop following the boundary when one of these happens:
- the goal is reached, or
- a complete cycle around the obstacle has been performed (then the goal is unreachable: failure), or
- $d_{reach} < d_{followed}$, which means "I can now see a way to get closer to the goal than any point on the wall I am following", so go back to motion-to-goal.

The full algorithm from slide 41:

```
repeat until goal is reached
    1. repeat
           take sensor-range reading and compute continuous range segments
           move toward point n in {goal, B1, B2, ...} that minimises h(x,n) = d(x,n) + d(n,goal)
       until goal is reached, or the value of h(x,n) begins to increase
    2. follow boundary continuing in the same direction as before, repeating
           update discontinuity points {B1, B2, ...}, d_reach, d_followed
       until goal is reached, or a complete cycle is performed (goal unreachable), or d_reach < d_followed
```

Summary bullets from slide 39: no map needed, start and goal known a priori, create a line and follow it towards the goal, **and use the onboard sensors**. Compared with Bug2, Tangent Bug reacts before touching anything and usually finds much shorter paths, because it can "see" the corners.

> **Common mistake.** $d_{reach}$ and $d_{followed}$ are easy to swap. Remember: *followed* is about the wall you are *currently following*; *reach* is about what you could *reach* if you let go now. You let go when reaching beats following.

---

## In a nutshell

Navigation is how a robot gets from A to B among obstacles, and it splits into finding a path (map-based), avoiding obstacles on the fly (reactive), and tracking the path (control). Reactive methods need no map and no localization: the Braitenberg vehicle wires two sensors straight to speed and steering, and artificial potential fields treat the robot as a particle pulled by the goal and pushed by obstacles, with the force equal to the negative gradient of a potential. In practice you compute a small step from a distance $d$ and an angle $\theta$ to each goal/obstacle, with gains $\alpha$ and $\beta$, add everything up, and move. Potential fields can get stuck in local minima, which is where Bug algorithms come in: Bug0 is naive and can loop forever, Bug1 circles each obstacle and leaves at the closest point, Bug2 follows a fixed m-line, and both Bug1 and Bug2 are complete. Tangent Bug adds a range sensor, aims at the obstacle corner that minimises distance-to-corner plus corner-to-goal, and follows walls only when that heuristic stops improving.

## Key terms

- **Navigation**: getting a robot from A to B through an environment with obstacles.
- **Path planning**: computing a collision-free route; distinct from control (tracking it) and trajectory generation (making it smooth in time).
- **Map-based navigation**: planning on a known map; aims for optimal routes and guarantees.
- **Reactive navigation**: acting on current sensor data only; no map, no localization.
- **Braitenberg vehicle**: a robot whose sensors are wired directly to its actuators, giving emergent goal-seeking behaviour.
- **Configuration space (C-space)**: the space of all robot configurations $q$; obstacles mapped into it are C-obstacles.
- **Artificial Potential Field (APF)**: a scalar function $U(q)$, low at the goal and high near obstacles, that the robot descends.
- **Gradient $\nabla U$**: the vector of partial derivatives pointing uphill; the robot moves along $-\nabla U$.
- **Attractive potential**: pulls towards the goal; linear (constant force) or quadratic (spring-like force).
- **Repulsive potential**: pushes away from an obstacle, active only within the distance of influence $\rho_0$, with gain $\eta$.
- **Local minimum**: a point where attractive and repulsive forces cancel so the robot stops short of the goal.
- **Hit point / leave point**: where a Bug robot first touches an obstacle / where it leaves it.
- **Completeness**: a planner finds a path in finite time if one exists, and reports failure if none does.
- **m-line**: the straight line from start to goal used by Bug2.
- **Range segmentation**: splitting a 360° range scan at its discontinuities into obstacle segments and free intervals.
- **Tangent Bug**: a Bug variant that uses a range sensor and the heuristic $d(x, n) + d(n, \text{goal})$.
- **$d_{followed}$ / $d_{reach}$**: shortest distance from the followed boundary to the goal / from the blocking obstacle (or robot, if clear) to the goal.

## Check yourself

1. In the Braitenberg controller $V = 2 - s_R - s_L$, $\gamma = k(s_R - s_L)$, what happens to the speed and steering as the robot arrives exactly on top of the source?
2. Write the force produced by the quadratic attractive potential $U_a = \tfrac12 \xi \|q - q_{goal}\|^2$ and explain in words how it differs from the force of the linear potential.
3. A robot is at $(0, 0)$, an obstacle of radius $r = 1$ is at $(0, 3)$, and the range of influence is $s = 3$ with $\beta = 2$. Compute the repulsive $(\Delta x, \Delta y)$.
4. Why is Bug0 not complete, and what single change does Bug2 make that fixes it?
5. In Tangent Bug, when does the robot switch from boundary following back to motion-to-goal?

<details>
<summary>Answers</summary>

1. Both sensors read 1 at the source, so $V = 2 - 1 - 1 = 0$: the robot stops. The difference $s_R - s_L$ is 0, so $\gamma = 0$: no steering. The robot parks on the source.

2. $F_a = -\nabla U_a = -\xi (q - q_{goal})$. Its magnitude is $\xi$ times the distance to the goal, so it is strong far away and gentle near the goal (like a spring). The linear potential gives $F_a = -\xi \frac{q - q_{goal}}{\|q - q_{goal}\|}$, whose magnitude is always $\xi$ regardless of distance (like a constant tractor beam).

3. $d = 3$, $\theta = \text{atan2}(3, 0) = 90°$, so $\cos\theta = 0$, $\sin\theta = 1$. Since $r = 1 \le 3 \le s + r = 4$, we are in the influence zone: $\Delta x = -2(3 + 1 - 3)\cdot 0 = 0$, $\Delta y = -2(4 - 3)\cdot 1 = -2$. The robot is pushed 2 units in the negative $y$ direction, directly away from the obstacle.

4. Bug0 has no memory: it leaves an obstacle as soon as the goal direction looks free, so on a C-shaped obstacle it can circle the outside forever and never terminate. Bug2 fixes this by fixing the m-line from start to goal and only leaving an obstacle when it re-encounters that m-line (closer to the goal, in the improved version). Because you always return to the same line and make progress along it, you either reach the goal or come back to the hit point and can declare failure.

5. When $d_{reach} < d_{followed}$: the shortest distance from the currently visible blocking obstacle (or, if nothing blocks, from the robot) to the goal becomes smaller than the shortest distance from the wall being followed to the goal. In other words, letting go of the wall now gets you closer to the goal than any point on that wall could.

</details>
