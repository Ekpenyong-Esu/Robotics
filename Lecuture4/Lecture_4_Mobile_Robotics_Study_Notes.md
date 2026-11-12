# Lecture 4 – Intro to Mobile Robotics: Study Notes

*Course R7010E Robotics (LTU). Source: `R7010E_Lecture_4_Mobile_Robotics.pdf` (26 slides) and the MATLAB/Simulink demo script `Chapter4.m` / `Chapter4.mlx`, which run the Corke "Robotics, Vision and Control" models `sl_lanechange`, `sl_drivepoint`, `sl_driveline`, `sl_pursuit`, `sl_drivepose`.*

---

## What this lecture is about, and why it matters

Up to now you have learned how to describe *where* a robot is (pose) and how to make smooth *trajectories* in time. This lecture is where the robot finally gets wheels. You will learn how a wheeled robot actually moves when you turn its steering wheel or spin its two wheels at different speeds (that is the "kinematic model"), and then how to write simple controllers that make it drive to a point, follow a line, follow a path, and park at a desired pose. These are exactly the controllers you will build on the TurtleBot3 in Lab 2, and they are the foundation for everything in the navigation lectures that follow.

## Roadmap: the main topics in this lecture

1. Configuration, degrees of freedom and actuators for different vehicles
2. Holonomic versus non-holonomic vehicles
3. Wheel configurations
4. The unicycle model (the simplest wheeled robot)
5. The bicycle (car-like, Ackermann) model: turning radius, steering angle, constraints, singularity
6. The bicycle model with slip angle β
7. Simulink demo: the Bicycle block and `sl_lanechange`
8. Control problem 1: moving to a point (`sl_drivepoint`)
9. Control problem 2: following a line (`sl_driveline`)
10. Control problem 3: following a path, "pure pursuit" (`sl_pursuit`)
11. Control problem 4: moving to a pose with polar coordinates ρ, α, β (`sl_drivepose`)
12. The differential-steered robot (TurtleBot-style), converting between wheel speeds and (V, ω)
13. Error dynamics expressed in the robot's own frame
14. Linearization, the Jacobian, and checking stability with eigenvalues
15. A real control architecture for a differential-drive adhesion robot
16. How this connects to Lab 2

---

## 1. Configuration, degrees of freedom and actuators

Let's start with vocabulary you will use for the whole course.

The **configuration** of a robot, written **q**, is the smallest list of numbers that completely tells you where the robot is and how it is oriented. Think of it as the robot's "address": once you know q, you know everything about its placement.

The number of entries in q is the number of **degrees of freedom (DoF)**. Each DoF is one independent way the robot can be placed.

An **actuator** is anything that produces motion: a motor, a thruster, a steering servo. The number of actuators is how many independent "knobs" you can turn.

Slide 2 lines up six vehicles and gives their configuration and actuators:

| Vehicle | q (configuration) | DoF | Actuators |
|---|---|---|---|
| Train | $q = x$ (how far along the track) | 1 | speed |
| Hovercraft | $q = (x, y, \theta)$ | 3 | thrust, yaw |
| Helicopter | $q = (x, y, z, \theta_r, \theta_p, \theta_y)$ | 6 | main thrust, lateral cyclic pitch, longitudinal cyclic pitch, tail yaw |
| Fixed-wing aircraft | $q = (x, y, z, \theta_r, \theta_p, \theta_y)$ | 6 | forward thrust, ailerons, elevator, rudder |
| Quadrotor | $q = (x, y, z, \theta_r, \theta_p, \theta_y)$ | 6 | 4 motors |
| Car | $q = (x, y, \theta)$ | 3 | steering, throttle |

Here $\theta_r, \theta_p, \theta_y$ are **roll, pitch and yaw**, the three rotation angles you met in Lecture 1 (roll = tilting side to side, pitch = nose up or down, yaw = turning left or right like a compass heading).

Notice the pattern: the train has one DoF and one actuator, so it can do exactly what it wants. The car has three DoF (position x, position y, heading θ) but only two actuators (steering and throttle). That gap, "fewer actuators than DoF", is the whole reason mobile robot control is interesting. It means there are directions the car simply cannot move in *right now*, even though it can eventually reach any (x, y, θ).

**Everyday example:** parallel parking. The parking spot is only one metre to your right. You cannot slide sideways into it. You have to drive forward and backward with the wheels turned, a sequence of allowed motions, to achieve a result (sideways displacement) that no single allowed motion can produce.

---

## 2. Holonomic versus non-holonomic

This is a term that confuses many people, so let's build it carefully.

A vehicle is **holonomic** if it can move instantly in any direction of its configuration space. In plain words: every DoF can be changed directly and independently at any moment. A vehicle is **non-holonomic** if there are velocity constraints: some directions of motion are forbidden at any given instant, even though the robot can reach those configurations eventually through a sequence of moves.

Slide 3 gives this table:

| Vehicle | DoF | Number of actuators | Holonomic? |
|---|---|---|---|
| Train | 1 | 1 | YES |
| Hovercraft | 3 | 2 | NO |
| Helicopter | 6 | 4 | NO |
| Fixed wing | 6 | 4 | NO |
| Quadrotor | 6 | 4 | YES |
| Car | 3 | 2 | NO |

The rule of thumb behind the table: if the number of actuators is at least the number of DoF, the vehicle can be holonomic. If there are fewer actuators than DoF, it is (usually) non-holonomic. Train: 1 = 1, yes. Car: 2 < 3, no. Hovercraft: it can push forward and rotate, but it cannot push sideways, so no. Fixed-wing: it can never fly sideways or hover, so no.

**Common mistake:** "Holonomic means it can go anywhere." No. The car can go anywhere in (x, y, θ) eventually. Holonomic is about *instantaneous* freedom: can it go in every direction *right now*? The car cannot go sideways right now, so it is non-holonomic.

**Everyday example:** a shopping trolley with four swivel castors is (roughly) holonomic: you can push it sideways, forward or spin it on the spot. An office chair on castors is the same. A bicycle is non-holonomic: it goes where its front wheel points, full stop.

> ⚠️ **The quadrotor row is a judgement call that the slide does not explain.** A quadrotor has 6 DoF but only 4 motors, so by the "actuators ≥ DoF" rule you would expect NO, and in the strict sense a quadrotor is indeed *under-actuated*: it must tilt (roll or pitch) in order to accelerate sideways, so it cannot change all six coordinates independently at the same instant. The slide says YES, and the most likely reasoning is that in the *position* sense a quadrotor can hover and move in any of x, y, z and rotate in yaw at will, unlike a car or aeroplane, so for navigation purposes it behaves like a holonomic vehicle. If this comes up, be ready to argue both sides, and check with your lecturer which answer they expect.

---

## 3. Wheel configurations (slide 4)

Slide 4 is a picture gallery of how wheels can be arranged. You do not need equations here, just recognition:

- **Two wheels:** one driven wheel plus one steered wheel (like a bicycle), or two side-by-side driven wheels (a differential drive, like a Segway or the TurtleBot).
- **Three wheels:** two driven wheels plus a passive castor for balance (the classic differential-drive layout, exactly the TurtleBot3 Burger with its ball castor), or two fixed wheels plus one steered wheel (a tricycle).
- **Omnidirectional drive:** three or more special wheels (e.g. "omni" or "mecanum" wheels with rollers around the rim) placed at angles so the robot can move in any direction instantly. This is the holonomic case.
- **Synchro drive:** all wheels steer together and drive together, linked by belts or chains, so the body keeps its orientation while the wheels point wherever they like.

The take-away: the arrangement of wheels decides which kinematic model (below) applies and whether the robot is holonomic.

---

## 4. The unicycle model (slide 5)

This is the simplest wheeled model and the one everything else reduces to. Picture a single wheel rolling on the ground, pointing in direction $\varphi$ and rolling with speed $V$.

$$
\dot x = V\cos\varphi, \qquad \dot y = V\sin\varphi, \qquad \dot\varphi = \omega
$$

In words: the wheel's x-position grows at a rate equal to the speed times the cosine of the heading; the y-position grows at speed times the sine of the heading; and the heading itself changes at the turn rate $\omega$.

Meaning of each symbol:
- $(x, y)$: position of the wheel contact point in the world frame.
- $\varphi$ (often written $\theta$ later): heading angle, measured from the world x-axis, counter-clockwise positive.
- $V$: forward (rolling) speed, in m/s. This is a **control input**.
- $\omega$: angular velocity (rate of turning), in rad/s. This is the other **control input**.
- The dot on top means "rate of change with time", i.e. a time derivative.

Why cos and sin? Because the velocity vector has length $V$ and points along the heading; projecting it onto the x and y axes gives $V\cos\varphi$ and $V\sin\varphi$. That is just trigonometry on a right-angled triangle.

**Worked example.** The robot is at $(x, y, \varphi) = (0, 0, 0)$, so facing along +x. You command $V = 1$ m/s and $\omega = 0.5$ rad/s. Over a small time step of $\Delta t = 0.1$ s:
1. $\dot x = 1\cdot\cos 0 = 1$, so $x$ becomes $0 + 1\cdot 0.1 = 0.1$ m.
2. $\dot y = 1\cdot\sin 0 = 0$, so $y$ stays at 0.
3. $\dot\varphi = 0.5$, so $\varphi$ becomes $0.05$ rad (about 2.9°).
On the next step the heading is no longer zero, so a little of the speed starts going into y. Repeat many times and you trace out a circle of radius $V/\omega = 2$ m.

Note the constraint hiding here: there is no way to make $\dot y \neq 0$ while $\varphi = 0$. The unicycle can never slide sideways. That is the non-holonomic constraint in its purest form.

---

## 5. The bicycle model (car-like robot, Ackermann steering) – slide 6

A real car has four wheels, but for kinematics we collapse the two front wheels into one steered wheel and the two rear wheels into one fixed wheel. The result is the **bicycle model**. "Ackermann steering" is the name of the real-car mechanism that turns the inner front wheel a bit more than the outer one so that all wheels roll around the same centre without scrubbing; the bicycle model is its idealisation.

### 5.1 Setting the scene

- Body frame $\{B\}$ sits at the **rear axle**, with $x_B$ pointing forward. Configuration is $q = (x, y, \theta)$.
- $L$ is the **wheelbase**: the distance between the rear axle and the front axle.
- $\gamma$ (gamma) is the **steering angle**: how far the front wheel is turned relative to the body.
- $v$ is the speed of the rear axle point along $x_B$.
- Because the wheel cannot slip sideways, the body-frame velocity is $v_{\dot x} = v$ and $v_{\dot y} = 0$: all motion is forward, none sideways.

### 5.2 Instantaneous centre of rotation and turning radius

Draw a line through the rear axle perpendicular to the body, and a line through the front wheel perpendicular to where it is pointing. They meet at a point called the **instantaneous centre of rotation (ICR)**: the point the whole car is pivoting around *at this instant*. Both wheels move on circles around the ICR.

From the right-angled triangle formed by the rear axle, the front axle and the ICR, the rear wheel's turning radius is

$$
R_B = \frac{L}{\tan\gamma}
$$

In words: the turning radius equals the wheelbase divided by the tangent of the steering angle. The slide's arrows say it compactly: **γ up ⇒ R down.** Turn the wheel more and the circle you drive gets tighter.

The car's rate of turning is then speed over radius:

$$
\dot\theta = \frac{v}{R_B} = \frac{v}{L}\tan\gamma
$$

### 5.3 The full bicycle model

$$
\dot x = v\cos\theta, \qquad \dot y = v\sin\theta, \qquad \dot\theta = \frac{v}{L}\tan\gamma
$$

Compare with the unicycle: the first two lines are identical. The only difference is that the turn rate is no longer a free input $\omega$; it is produced by the steering angle and the speed together, $\omega = (v/L)\tan\gamma$. The control inputs of the bicycle are $v$ and $\gamma$.

**Worked example.** A car with wheelbase $L = 2.5$ m drives at $v = 5$ m/s with steering angle $\gamma = 10°$.
1. $\tan 10° = 0.176$.
2. Turning radius: $R_B = 2.5 / 0.176 = 14.2$ m.
3. Turn rate: $\dot\theta = 5 / 14.2 = 0.35$ rad/s, which is about 20° per second.
4. Time for a full circle: $2\pi / 0.35 \approx 18$ s.
If you double the steering angle to 20°, $\tan 20° = 0.364$, so $R_B = 6.9$ m: more than twice as tight. Steering is not linear.

### 5.4 Two things that can go wrong

The slide notes two special situations:

- **If the car is not moving, it cannot turn.** The slide writes "$\dot v = 0 \Rightarrow \dot\theta = 0$". Look at $\dot\theta = (v/L)\tan\gamma$: if $v = 0$ then $\dot\theta = 0$ no matter how you turn the wheel. A parked car with its wheels cranked still points the same way.
  > ⚠️ The slide literally writes $\dot v = 0$ (zero *acceleration*), but the conclusion $\dot\theta = 0$ only follows from $v = 0$ (zero *speed*). Read it as "speed zero ⇒ no turning". Check with your lecturer if you want the exact wording they intend.
- **Singularity at $\gamma = \pm\pi/2$.** If the front wheel is turned a full 90°, $\tan\gamma \to \infty$ and $R_B \to 0$: the model breaks down (the ICR lands on the rear axle). Real cars limit the steering angle well below this.
  > ⚠️ The slide writes "$\theta = \pi/2 \Rightarrow$ singularity". In the bicycle model the troublesome angle is the *steering* angle $\gamma$, not the heading $\theta$: a heading of 90° (driving straight north) is perfectly fine. Treat this as a typo and remember: the singularity is **steering angle = 90°**.

### 5.5 The non-holonomic constraints

The slide lists, under the heading "Holonomic Constraints":

$$
\dot x\sin\theta - \dot y\cos\theta = 0 \quad\text{(rear wheel)}
$$
$$
\dot x\sin(\theta+\gamma) - \dot y\cos(\theta+\gamma) = 0 \quad\text{(front wheel)}
$$

What do these say? Take the first one and substitute the model: $v\cos\theta\sin\theta - v\sin\theta\cos\theta = 0$. It is always true. It is the mathematical statement "the velocity has no component perpendicular to the wheel", i.e. **no sideways slip**. The second line says the same for the front wheel, which points in direction $\theta + \gamma$.

> ⚠️ These are **non-holonomic** constraints (constraints on velocities that cannot be integrated into a constraint on positions), even though the slide title says "Holonomic Constraints". This is a known naming slip; in an exam, call them the **non-holonomic (rolling-without-slipping) constraints** of the bicycle model.

**Common mistake:** confusing "constraint equals zero" with "robot cannot move". The constraint does not stop the robot; it only forbids one direction (sideways) at each instant.

---

## 6. The bicycle model with slip angle β (slide 7)

Real tyres do slip a little, and some vehicles can steer both front and rear wheels. Slide 7 takes a model from Rajamani's *Vehicle Dynamics and Control* (page 20) that handles both.

New symbols:
- $C$: the centre of gravity (CG), which is now the reference point instead of the rear axle.
- $l_f$, $l_r$: distances from the CG to the front and rear axles.
- $\delta_f$, $\delta_r$: front and rear steering angles.
- $\psi$ (psi): the heading (yaw) of the body.
- $\beta$ (beta): the **slip angle**, the angle between where the body points ($\psi$) and where the CG is actually travelling. If $\beta = 0$ the car moves exactly where it points.
- $V$: speed of the CG; $a$: acceleration.

The model:

$$
\dot X = V\cos(\psi+\beta), \qquad \dot Y = V\sin(\psi+\beta)
$$
$$
\dot\psi = \frac{V\cos\beta}{l_f + l_r}\left(\tan\delta_f + \tan\delta_r\right)
$$
$$
\beta = \tan^{-1}\!\left(\frac{l_r}{l_f + l_r}\tan\delta_f\right), \qquad \dot v = a
$$

In words: the CG moves in direction heading-plus-slip; the yaw rate depends on speed, on the total wheelbase $l_f + l_r$ and on both steering angles; and the slip angle is set by the geometry and the front steering. Control signals are $V$, $\delta_f$, $\delta_r$.

Sanity check: set $\delta_r = 0$ and $\beta \approx 0$ (small angles) and you recover the ordinary bicycle model with $L = l_f + l_r$. So this is a generalisation, not a different animal.

> ⚠️ Two small issues on this slide. (1) The denominator of the $\beta$ formula is printed as $l_f + l_f$; it should be $l_f + l_r$ (the total wheelbase), which is what Rajamani writes. (2) Rajamani's full expression for a car with front *and* rear steering is $\beta = \tan^{-1}\!\big(\frac{l_f\tan\delta_r + l_r\tan\delta_f}{l_f + l_r}\big)$; the slide shows the special case with $\delta_r = 0$. For the exam, the simplified version on the slide is what you are expected to know, but be aware of the typo.

---

## 7. MATLAB time: the Bicycle block and `sl_lanechange` (slide 8)

The Simulink **Bicycle** block implements exactly the model of Section 5.3. Reading the diagram: input 1 is $v$, input 2 is $\gamma$. The speed passes through a *velocity limit* and an *acceleration limit* (saturation blocks, so the simulated car cannot do impossible things). The steering angle passes through a *steering angle limit* (that is how the $\gamma = 90°$ singularity is avoided) and then a $\tan(u)/L$ block, which is the $\tan\gamma / L$ factor. Multiplying by $v$ gives $\dot\theta$. The $\cos$ and $\sin$ blocks build $\dot x, \dot y$. A single integrator $1/s$ turns the three rates into $(x, y, \theta)$. There is even a "handbrake" input that resets the integrator.

`sl_lanechange` feeds the block a constant speed of 1 and a steering **pulse**: turn left briefly, then right briefly. The XY plot shows the result: the car moves from $y = 0$ to $y \approx 1$ and then continues straight, i.e. a lane change. The script does

```matlab
out = sim('sl_lanechange');
t = out.get('t'); q = out.get('y');
mplot(t, q(:,2:4))      % x, y, theta against time
plot(q(:,1), q(:,2))    % the path in the XY plane
```

The lesson: with a non-holonomic car you cannot just "step sideways" one lane; you need a sequence of steering actions.

---

## 8. Control problem 1: moving to a point (slide 9, `sl_drivepoint`)

Now we start controlling. The task: from the current position $(x^o, y^o)$ with heading $\theta$, drive to a goal point $(x^*, y^*)$. We do not care about the final heading.

The idea is two separate, very intuitive rules:

**Step 1. Drive faster when you are far away, slower when close.**

$$
v^* = K_v\sqrt{(x^*-x^o)^2 + (y^*-y^o)^2}
$$

The square root is just the straight-line **distance to the target** (Pythagoras). $K_v$ is a positive gain you choose. The star on $v^*$ means "desired".

**Step 2. Point towards the goal.** The direction to the goal is

$$
\theta^* = \tan^{-1}\frac{y^*-y^o}{x^*-x^o}
$$

and the steering is proportional to the **heading error**, the difference between where you should point and where you do point:

$$
\gamma = K_h(\theta^* - \theta)
$$

with $K_h$ another positive gain.

**Important practical detail: use `atan2`, not `atan`.** The plain inverse tangent only returns angles between −90° and +90°, so it cannot tell "goal is behind me to the left" from "goal is ahead to the right". The two-argument version `atan2(dy, dx)` looks at the signs of both inputs and returns the correct angle over the full circle. The Simulink model on slide 10 uses `atan2(u(2), u(1))` for exactly this reason. Also, the heading error must be wrapped into $[-\pi, \pi]$ (Simulink's `angdiff` block does this), otherwise an error of 350° would make the robot spin the long way round instead of turning 10° the other way.

**Worked example** with the slide's numbers: goal $[5, 5]$, start $x_0 = [8, 5, \pi/2]$ (at $(8,5)$, facing +y), gains $K_v = 0.5$, $K_h = 4$.
1. Distance: $\sqrt{(5-8)^2 + (5-5)^2} = \sqrt{9} = 3$ m.
2. Desired speed: $v^* = 0.5 \cdot 3 = 1.5$ m/s.
3. Desired heading: $\theta^* = \text{atan2}(0, -3) = \pi$ (the goal is straight to the left, i.e. in the −x direction). Note that plain `atan(0/−3) = 0` would wrongly say "straight ahead in +x".
4. Heading error: $\pi - \pi/2 = \pi/2$ (turn left by 90°).
5. Steering: $\gamma = 4 \cdot \pi/2 = 2\pi$ rad, which is absurd; the steering limit in the Bicycle block saturates it to the maximum allowed angle. So the car turns as hard as it can to the left, and as the error shrinks the steering relaxes.

The bottom-right plot on slide 10 shows the same controller run from eight different starting poses around the goal: every path curls smoothly into the point. The middle plot shows the speed decaying towards zero as the distance shrinks.

**Common mistake:** expecting a straight-line path. The car cannot turn on the spot, so it drives a curve while rotating towards the goal. Also note the controller never cares about the arrival heading; it stops at the point pointing wherever it happens to point.

> Small bookkeeping note: the slide uses $x_0 = [8\ 5\ \pi/2]$ but the provided `Chapter4.m` uses $x_0 = [4\ 5\ \pi/2]$. Either is fine; the only difference is which side of the goal the robot starts on.

---

## 9. Control problem 2: following a line (slide 11, `sl_driveline`)

Task: drive along an infinite straight line. The line is written in the general form

$$
ax + by + c = 0
$$

(e.g. $x - 2y + 4 = 0$ is the line $y = x/2 + 2$). Writing it this way, instead of $y = mx + k$, has a big advantage: vertical lines work too, and the distance formula below is clean.

**Step 1. How far am I from the line?**

$$
d = \frac{a x^o + b y^o + c}{\sqrt{a^2 + b^2}}
$$

This is the standard perpendicular distance from a point to a line. The sign of $d$ tells you which side of the line you are on (positive on one side, negative on the other). That sign is essential: the controller must know *which way* to steer.

> ⚠️ The slide prints the numerator inside absolute-value bars, $|ax^o+by^o+c|$. With the absolute value you lose the side information and the controller cannot work. Corke's book and the Simulink model use the **signed** distance (no absolute value). Use the signed version.

**Step 2. What heading is parallel to the line?** The line's direction has slope $-a/b$, so

$$
\theta^* = \tan^{-1}\frac{-a}{b}
$$

(The Simulink model computes `atan2(-L(1), L(2))`; either of the two opposite directions along the line is "parallel", and `atan2` picks one consistently.)

**Step 3. Combine two steering terms.**

- Turn *towards* the line, proportional to how far away you are: $a_d = -K_d\, d$.
- Turn to become *parallel* to the line: $a_h = K_h(\theta^* - \theta)$.

Overall steering law:

$$
\gamma = -K_d\, d + K_h(\theta^* - \theta)
$$

Why two terms? With only the distance term the robot would reach the line and then overshoot because nothing tells it to straighten up; with only the heading term it would drive parallel to the line but never close the gap. Together: the distance term pulls you in, and the heading term settles you along the line. The speed is simply held constant (the Simulink model uses speed = 1).

**Everyday analogy:** walking along a painted line in a car park while blindfolded with a friend giving instructions. "You're a metre to the left" (distance term) and "you're angled too far right" (heading term). You need both to walk neatly along it.

**Worked example** with the slide's numbers: $L = [1, -2, 4]$ (so $a = 1$, $b = -2$, $c = 4$), robot at $x_0 = [8, 5, \pi/2]$, gains $K_d = 0.5$, $K_h = 1$.
1. Distance: $d = (1\cdot 8 + (-2)\cdot 5 + 4)/\sqrt{1 + 4} = (8 - 10 + 4)/\sqrt 5 = 2/2.236 = 0.894$. Check: at $x = 8$ the line is at $y = 8/2 + 2 = 6$, and the robot is at $y = 5$, one unit below it, which measured perpendicular to the line is indeed $0.894$.
2. Parallel heading: $\theta^* = \tan^{-1}(-1/-2) = \tan^{-1}(0.5) = 0.464$ rad (26.6°). That is the slope of $y = x/2 + 2$, as expected.
3. Heading error: $0.464 - 1.571 = -1.107$ rad (robot is pointing too far counter-clockwise, must turn right).
4. Steering: $\gamma = -0.5\cdot 0.894 + 1\cdot(-1.107) = -0.447 - 1.107 = -1.55$ rad, saturated by the steering limit. The robot turns right hard, the heading term dominates at first, and as it comes parallel the distance term gently brings it onto the line.

The slide's plots show the heading converging to about 0.46 rad and $d$ converging to zero, with a small overshoot. The right-hand plot shows three different starting poses all ending up on the dashed line.

---

## 10. Control problem 3: following a path, "pure pursuit" (slide 13, `sl_pursuit`)

Now the target is not a fixed point but a **moving point** that slides along a desired path ahead of the robot. The slide calls it the **donkey and carrot** method: hang a carrot a fixed distance in front of the donkey, and the donkey follows wherever the carrot goes. In the literature this is called **pure pursuit**.

**Step 1. Distance error to the carrot.**

$$
e = \sqrt{(x^*-x^o)^2 + (y^*-y^o)^2} - d^*
$$

Here $(x^*, y^*)$ is the moving target point on the path and $d^*$ is the desired **look-ahead distance**: how far ahead we want the carrot to be. If $e > 0$ the robot has fallen behind; if $e < 0$ it is too close.

**Step 2. Speed control with a PI controller.**

$$
v^* = K_v\, e + K_i\int e\,dt
$$

The proportional part $K_v e$ speeds up when behind. The integral part $K_i\int e\,dt$ accumulates any persistent error and removes it. Why is the integral needed here but not in "move to point"? Because the target is *moving*: with P-only control the robot would settle at a steady distance error (it needs a non-zero error just to generate the speed to keep up). The integral term lets the robot match the carrot's speed with zero steady-state error. In the Simulink model this is the red PID block.

**Step 3. Steering exactly as before.**

$$
\theta^* = \tan^{-1}\frac{y^*-y^o}{x^*-x^o}, \qquad a = K_h(\theta^* - \theta)
$$

(The slide writes the steering output as $a$; it plays the role of $\gamma$.)

The Simulink model `sl_pursuit` has a *trajectory generator* block that produces the moving target; you edit its properties to change the path, and it creates `P` (way-point list), `dt`, `vseg`, `robotpath` and `tseg` in the workspace. The example path is a circle; the XY plot shows the robot starting off-circle, catching up, and then orbiting on top of the path. The speed plot shows a transient then settling at the path speed; the heading plot shows a spike while it catches up and then steady rotation.

**Common mistake:** choosing the look-ahead $d^*$ badly. Too small and the robot oscillates (it chases every wiggle); too large and it cuts corners (it smooths the path). This is the knob you will play with in Lab 2 Task 2.

---

## 11. Control problem 4: moving to a pose (slides 15–16, `sl_drivepose`)

This is the hardest of the four and the one most likely to appear on an exam. Task: reach a goal position **and** a goal heading. Think parking: the position of the bay is not enough; you also need to be pointing the right way.

### 11.1 Start with a unicycle view of the car

Write the car as a unicycle with inputs $(v, \omega)$:

$$
\begin{pmatrix}\dot x\\ \dot y\\ \dot\theta\end{pmatrix} =
\begin{pmatrix}\cos\theta & 0\\ \sin\theta & 0\\ 0 & 1\end{pmatrix}
\begin{pmatrix} v\\ \omega\end{pmatrix}
$$

and afterwards convert the desired turn rate into a steering angle for the bicycle with

$$
\gamma = \tan^{-1}\frac{\omega L}{v}
$$

(which is just $\omega = (v/L)\tan\gamma$ rearranged). The Simulink model calls this a "virtual unicycle" wrapped around the Bicycle block.

### 11.2 Change to polar coordinates centred on the goal

Put the goal frame $\{G\}$ at the origin with its x-axis as the desired final heading (we will handle a general goal later). Let $\Delta_x, \Delta_y$ be the goal position minus the robot position. Define three new variables:

$$
\rho = \sqrt{\Delta_x^2 + \Delta_y^2}, \qquad
\alpha = \tan^{-1}\frac{\Delta_y}{\Delta_x} - \theta, \qquad
\beta = -\theta - \alpha
$$

What each one means:
- $\rho$ (rho): straight-line **distance** to the goal.
- $\alpha$ (alpha): the angle between the robot's heading and the line to the goal. "How far off am I pointing from the goal?" If $\alpha = 0$ the goal is dead ahead.
- $\beta$ (beta): the angle between the line to the goal and the goal's own x-axis. "Once I arrive along this line, how far off will my heading be from the required final heading?" It is defined so that $\alpha + \beta = -\theta$; if you work out $\theta$ for $\rho \to 0$ with $\alpha = \beta = 0$ you get $\theta = 0$, i.e. the goal heading.

Reaching the goal pose means $(\rho, \alpha, \beta) = (0, 0, 0)$. That is the point of the transformation: three awkward world coordinates become three errors that should all go to zero.

**Everyday analogy:** docking a boat at a jetty. $\rho$ is how far you still have to go, $\alpha$ is whether your bow is pointed at the berth, and $\beta$ is whether, when you get there, you will be lying alongside the jetty or sticking out at an angle.

### 11.3 How do ρ, α, β change when the robot moves?

Differentiating (the slide just states the result):

$$
\begin{pmatrix}\dot\rho\\ \dot\alpha\\ \dot\beta\end{pmatrix} =
\begin{pmatrix}-\cos\alpha & 0\\ \dfrac{\sin\alpha}{\rho} & -1\\ -\dfrac{\sin\alpha}{\rho} & 0\end{pmatrix}
\begin{pmatrix} v\\ \omega\end{pmatrix}
\qquad\text{if } \alpha \in \left(-\tfrac{\pi}{2}, \tfrac{\pi}{2}\right]
$$

Reading it row by row: moving forward with the goal ahead ($\cos\alpha > 0$) shrinks the distance; turning ($\omega$) directly reduces $\alpha$; and moving forward when the goal is off to one side swings both $\alpha$ and $\beta$.

The condition $\alpha \in (-\pi/2, \pi/2]$ means "the goal is in the front half-plane". If instead the goal is *behind* the robot, it is silly to make a giant U-turn; it is better to **drive backwards**. The slide writes "Otherwise A → −A", a cryptic shorthand meaning: if $\alpha$ is outside that range, flip the direction of motion (use negative $v$) and redefine the angles for the reversed heading. In the Simulink model this is the "direction of motion: +1 or −1" signal that multiplies the velocity command.

> ⚠️ "Otherwise A → −A" is not explained on the slide. The intent (from Corke's book, which the model comes from) is: when the goal lies behind the robot, reverse the vehicle. Check with your lecturer if you need the exact rule for the exam.

### 11.4 The control law

$$
v = k_\rho\,\rho, \qquad \omega = k_\alpha\,\alpha + k_\beta\,\beta
$$

In words: drive forward at a speed proportional to the remaining distance, and turn at a rate that corrects both the pointing error $\alpha$ and the final-heading error $\beta$. Three gains: $k_\rho, k_\alpha, k_\beta$.

### 11.5 Closed-loop behaviour and stability (slide 16)

Substitute the control law into the polar dynamics:

$$
\begin{pmatrix}\dot\rho\\ \dot\alpha\\ \dot\beta\end{pmatrix} =
\begin{pmatrix}-k_\rho\,\rho\cos\alpha\\ k_\rho\sin\alpha - k_\alpha\alpha - k_\beta\beta\\ -k_\rho\sin\alpha\end{pmatrix}
$$

This is nonlinear (because of the sines and cosines), but the slide states the conditions for it to be **stable**, meaning all three errors decay to zero:

$$
k_\rho > 0, \qquad k_\beta < 0, \qquad k_\alpha - k_\rho > 0
$$

Equivalently, after linearising (Section 14), all eigenvalues of the system matrix $A$ have negative real part: $\mathrm{Re}(\mathrm{eig}(A)) < 0$.

**Why is $k_\beta$ negative?** Many people trip here. Look at the $\dot\beta$ row: $\beta$ only changes through $-k_\rho\sin\alpha$, so to drive $\beta$ to zero the controller must create a small $\alpha$ of the *right sign* via the $k_\beta\beta$ term in $\omega$, and the algebra works out that this requires $k_\beta < 0$. Intuitively: to end up parallel to the jetty, you first steer slightly *away* from the final heading so that you approach on a curve that ends tangent to it.

### 11.6 A general goal pose

So far the goal sat at the origin with heading 0. For any goal $(x^*, y^*, \theta^*)$, simply shift and rotate the coordinates:

$$
x' = x - x^*, \qquad y' = y - y^*, \qquad \theta' = \theta, \qquad \beta' = \beta - \theta^*
$$

In words: measure positions relative to the goal, keep the robot's heading as is, and subtract the goal heading from $\beta$ so that "$\beta' = 0$" means "pointing the way the goal wants".

### 11.7 Worked example

Robot at $(1, 1)$ with heading $\theta = 0$. Goal at $(5, 4)$ with heading $\theta^* = 0$. Gains from the Simulink model: $k_\rho = 1$, $k_\alpha = 5$, $k_\beta = -2$. Wheelbase $L = 1$.
1. $\Delta_x = 5 - 1 = 4$, $\Delta_y = 4 - 1 = 3$.
2. $\rho = \sqrt{16 + 9} = 5$ m.
3. $\alpha = \text{atan2}(3, 4) - 0 = 0.644$ rad (36.9°). It is inside $(-\pi/2, \pi/2]$, so drive forward.
4. $\beta = -\theta - \alpha = -0 - 0.644 = -0.644$ rad; with $\theta^* = 0$, $\beta' = \beta$.
5. $v = 1 \cdot 5 = 5$ m/s.
6. $\omega = 5\cdot 0.644 + (-2)\cdot(-0.644) = 3.22 + 1.29 = 4.51$ rad/s.
7. Steering: $\gamma = \tan^{-1}(4.51\cdot 1 / 5) = \tan^{-1}(0.90) = 0.73$ rad (42°), left.

Now repeat with the goal heading $\theta^* = \pi/2$ (arrive pointing +y). Only $\beta'$ changes: $\beta' = -0.644 - 1.571 = -2.21$ rad, so $\omega = 3.22 + (-2)(-2.21) = 3.22 + 4.43 = 7.65$ rad/s. The robot turns much harder because it has to swing around to finish pointing north. The gains are large here only for illustration; in the simulation the speed and steering limits in the Bicycle block clip them.

Check the stability conditions: $k_\rho = 1 > 0$ ✓, $k_\beta = -2 < 0$ ✓, $k_\alpha - k_\rho = 4 > 0$ ✓.

### 11.8 The Simulink model `sl_drivepose` (slides 17–18)

Reading the diagram: the current $(x, y, \theta)$ is subtracted from the desired position `[xg(1:2) 0]` to form the pose error, which goes into a **to polar** block producing $\rho$, $\alpha$, $\beta$ and the direction flag. Gains `krho = 1`, `kalpha = 5`, `kbeta = −2` build $v$ and $\omega$. The desired heading `xg(3)` is added to $\beta$ (the change of coordinates above). The $\omega$ is divided by $|v|$ and passed through `atan(u)` to get the steering angle for the Bicycle block. An "are we there yet?" function stops the simulation when $\rho$ is small.

Parameters: `x0 = [9 5 0]` (start at (9,5) facing +x) and `xg = [5 5 pi/2]` (finish at (5,5) facing +y). Note the goal is *behind* the robot (it is at smaller x while the robot faces +x), so $\alpha = \pi$: this is exactly the "otherwise" case, and the vehicle reverses. Slide 18 shows the same controller from eight starting poses, each curving in and arriving at $(5,5)$ pointing straight up.

**"How do we implement it in the real world?"** (the lightning-bolt question on slide 16). The slides pose it without answering; the answer you should have ready is: on a real robot you do not have $(x, y, \theta)$ for free. You need odometry (wheel encoders), a motion-capture system, or a localisation algorithm (Lecture 10) to estimate the pose, you must respect velocity and steering limits, the control loop runs at a fixed sample rate rather than continuously, and your robot is probably a differential drive, so you need the conversion in the next section.

---

## 12. The differential-steered robot (slides 19–20)

The TurtleBot3 does not have a steering wheel. It has two independently driven wheels, left and right, plus a castor. Steering happens by spinning the wheels at different speeds. This is a **differential drive**.

### 12.1 Geometry (slide 19)

- $v_L$, $v_R$: linear speeds of the left and right wheel contact points (m/s).
- $W$: the distance between the two wheels (the slide calls it "wheel width"; it is often called the track or wheel separation).
- Both wheels rotate about the same ICR, with radii $R_L$ and $R_R = R_L + W$.

Because both wheels share the same angular velocity about the ICR:

$$
\dot\theta = \frac{v_L}{R_L} = \frac{v_R}{R_R}
$$

Subtracting the two radii relations gives the key result:

$$
\dot\theta = \frac{v_R - v_L}{W}
$$

In words: the turn rate is the **difference** of the wheel speeds divided by the wheel separation. Equal speeds ⇒ straight line. Right faster than left ⇒ turn left (counter-clockwise). The slide writes the difference as $u_\Delta = u_R - u_L$.

The slide also marks a point $\{B'\}$ a distance $a$ ahead of the axle midpoint. Controlling that offset point instead of the axle centre is a common trick that removes the "cannot move sideways" problem for control design (the point $B'$ *can* move sideways when the robot turns). The slide does not elaborate on it.

> ⚠️ The boxed equations on slide 19 read $\dot x = \frac{u_\Delta}{2}\cos\theta$, $\dot y = \frac{u_\Delta}{2}\sin\theta$, $\dot\theta = \frac{u_\Delta}{W}$. The third line is correct, but the first two should use the **sum** (average) of the wheel speeds, $\frac{v_R + v_L}{2}$, not the difference. With the difference, a robot driving straight (equal wheel speeds) would have zero forward speed, which is clearly wrong. Slide 20 has the correct version. Use slide 20.

### 12.2 Wheel angular velocities and the unicycle equivalence (slide 20)

Slide 20 switches to a slightly different notation:
- $v_r$, $v_l$: **angular** velocities of the right and left wheels (rad/s).
- $R$: wheel radius, so the linear speed of each wheel rim is $R v_r$ and $R v_l$.
- $L$: wheel separation (same thing as $W$ on slide 19).

Differential drive model:

$$
\dot x = \frac{R}{2}(v_r + v_l)\cos\theta, \qquad
\dot y = \frac{R}{2}(v_r + v_l)\sin\theta, \qquad
\dot\theta = \frac{R}{L}(v_r - v_l)
$$

Unicycle model: $\dot x = V\cos\theta$, $\dot y = V\sin\theta$, $\dot\theta = \omega$.

Match them term by term:

$$
V = \frac{R}{2}(v_r + v_l), \qquad \omega = \frac{R}{L}(v_r - v_l)
$$

and invert to get what you actually send to the motors:

$$
v_r = \frac{2V + \omega L}{2R}, \qquad v_l = \frac{2V - \omega L}{2R}
$$

This is enormously useful: **every controller in Sections 8–11 outputs $(V, \omega)$ for a unicycle, and these two formulas convert that into left and right wheel commands.** On the TurtleBot you publish $(V, \omega)$ as a `cmd_vel` message and the robot's firmware does this conversion for you.

> ⚠️ Notation clash to watch: $L$ means **wheelbase** (front-to-rear) in the bicycle model but **wheel separation** (left-to-right) in the differential-drive model on slide 20, where slide 19 calls the same quantity $W$. Also, slide 20's footer says "Lecture 3" (a copy-paste slip). Finally, $v_r, v_l$ on slide 20 are angular velocities in rad/s, while $v_R, v_L$ on slide 19 are linear speeds in m/s.

**Worked example** with TurtleBot3 Burger numbers (wheel radius $R = 0.033$ m, wheel separation $L = 0.160$ m). You want $V = 0.2$ m/s and $\omega = 0.5$ rad/s (a gentle left turn).
1. $2V = 0.4$; $\omega L = 0.5 \cdot 0.16 = 0.08$; $2R = 0.066$.
2. $v_r = (0.4 + 0.08)/0.066 = 7.27$ rad/s.
3. $v_l = (0.4 - 0.08)/0.066 = 4.85$ rad/s.
4. Check forwards: $V = 0.0165\cdot(7.27 + 4.85) = 0.0165 \cdot 12.12 = 0.200$ m/s ✓; $\omega = (0.033/0.16)(7.27 - 4.85) = 0.206\cdot 2.42 = 0.50$ rad/s ✓.
The right wheel spins faster, so the robot curves left, as it should.

**Common mistake:** forgetting the factor $\frac12$ in $V$, or using the wheel's angular velocity where a linear speed is needed. Always ask: is this number in rad/s (wheel spin) or m/s (ground speed)? The wheel radius $R$ converts between them.

---

## 13. Error dynamics in the robot frame (slide 21)

When you track a path, it is natural to compare your pose with a **reference robot**: an imaginary robot that is exactly where you should be, at $(x_r, y_r, \theta_r)$. The raw difference $(x_r - x,\ y_r - y,\ \theta_r - \theta)$ is in world coordinates, which is awkward: "two metres north" means something different depending on which way you are facing.

So rotate the error into the real robot's own frame:

$$
\begin{pmatrix} e_1\\ e_2\\ e_3\end{pmatrix} =
\begin{pmatrix}\cos\theta & \sin\theta & 0\\ -\sin\theta & \cos\theta & 0\\ 0 & 0 & 1\end{pmatrix}
\begin{pmatrix} x_r - x\\ y_r - y\\ \theta_r - \theta\end{pmatrix}
$$

The $2\times2$ block is the transpose (inverse) of the 2D rotation matrix from Lecture 1, so this is "undo the robot's rotation". The result:
- $e_1$: **longitudinal** error, how far ahead (+) or behind (−) the reference is, along your heading.
- $e_2$: **lateral** error, how far to your left (+) or right (−) the reference is.
- $e_3$: heading error.

Why is this useful? Because the controller now sees errors that match its actuators: $e_1$ is fixed by speeding up or slowing down, $e_2$ and $e_3$ by steering. The "move to point", "follow line" and "move to pose" laws are all special cases of acting on errors like these.

**Worked example.** Reference at $(5, 5, \pi/2)$, real robot at $(4, 3, \pi/2)$ (both facing +y).
1. World differences: $x_r - x = 1$, $y_r - y = 2$, $\theta_r - \theta = 0$.
2. $e_1 = \cos(\pi/2)\cdot 1 + \sin(\pi/2)\cdot 2 = 0 + 2 = 2$: the reference is 2 m ahead.
3. $e_2 = -\sin(\pi/2)\cdot 1 + \cos(\pi/2)\cdot 2 = -1 + 0 = -1$: the reference is 1 m to the right (negative = right, because the robot's y-axis points left).
4. $e_3 = 0$.
So: speed up and steer a little right. Much easier to act on than "(1, 2) in world coordinates".

---

## 14. Linearization and checking stability (slides 22–24)

### 14.1 The problem

Our models are **nonlinear**: $\dot x = v\cos\theta$ contains a cosine of a state. Nonlinear systems are hard to analyse in general. The slide writes the general form

$$
\dot{\mathbf x} = f(\mathbf x, \mathbf u), \quad\text{here}\quad
\dot x = f_1(x,y,\theta,v,\omega),\ \dot y = f_2(\dots),\ \dot\theta = f_3(\dots)
$$

### 14.2 The Jacobian (slide 23)

**Linearization** means replacing a curved function by its tangent (straight-line approximation) near a chosen operating point $\mathbf x_0$. For a vector function the "slope" is a matrix called the **Jacobian**, whose entry in row $i$, column $j$ is the partial derivative of the $i$-th output with respect to the $j$-th input:

$$
J_F = \begin{pmatrix}\dfrac{\partial f_1}{\partial x_1} & \cdots & \dfrac{\partial f_1}{\partial x_n}\\ \vdots & & \vdots \\ \dfrac{\partial f_m}{\partial x_1} & \cdots & \dfrac{\partial f_m}{\partial x_n}\end{pmatrix}
$$

(The slide builds it from the gradient operator $\nabla_{\mathbf x} = [\partial/\partial x_1, \dots, \partial/\partial x_n]^T$ applied to each $f_i$.) The linear approximation is a Taylor expansion cut after the first term:

$$
F(\mathbf x) \approx F(\mathbf x_0) + J_F(\mathbf x_0)\,d\mathbf x
$$

so that $\dot{\mathbf x} \approx J_F(\mathbf x_0)\,d\mathbf x + F(\mathbf x_0)$. Near an equilibrium (where $F(\mathbf x_0) = 0$) this is simply $\dot{\mathbf x} \approx A\,\mathbf x$ with $A = J_F(\mathbf x_0)$, a linear system you know how to analyse.

**Everyday analogy:** zoom in far enough on any smooth curve and it looks like a straight line. The Jacobian is the slope of that line, one slope per input–output pair.

You will meet the Jacobian again in Lecture 10, where the Extended Kalman Filter linearises the motion and sensor models in exactly this way.

### 14.3 Applying it to the move-to-pose controller (slide 24)

Take the closed-loop polar equations from Section 11.5 and assume $\alpha$ is small, so $\cos\alpha \approx 1$ and $\sin\alpha \approx \alpha$ (this is the shortcut the slide uses instead of writing out every partial derivative; it gives the same Jacobian at the equilibrium):

$$
\begin{pmatrix}\dot\rho\\ \dot\alpha\\ \dot\beta\end{pmatrix} =
\begin{pmatrix}-k_\rho & 0 & 0\\ 0 & -(k_\alpha - k_\rho) & -k_\beta\\ 0 & -k_\rho & 0\end{pmatrix}
\begin{pmatrix}\rho\\ \alpha\\ \beta\end{pmatrix}
$$

Check the middle row: $k_\rho\sin\alpha - k_\alpha\alpha - k_\beta\beta \approx k_\rho\alpha - k_\alpha\alpha - k_\beta\beta = -(k_\alpha - k_\rho)\alpha - k_\beta\beta$ ✓.

### 14.4 Eigenvalues and the stability conditions

A linear system $\dot{\mathbf x} = A\mathbf x$ is stable (all errors decay to zero) exactly when every **eigenvalue** of $A$ has negative real part: $\mathrm{Re}(\mathrm{eig}(A)) < 0$. Eigenvalues are the numbers $\lambda$ with $\det(A - \lambda I) = 0$; each one is a "decay rate" of one mode of the system. Negative means decay, positive means blow-up.

Our $A$ is block-triangular, so its eigenvalues are:
1. From the top-left entry: $\lambda_1 = -k_\rho$. Negative iff $k_\rho > 0$.
2. From the lower $2\times2$ block $\begin{pmatrix}-(k_\alpha-k_\rho) & -k_\beta\\ -k_\rho & 0\end{pmatrix}$, whose characteristic equation is $\lambda^2 + (k_\alpha - k_\rho)\lambda - k_\rho k_\beta = 0$. A quadratic has both roots in the left half-plane iff both coefficients are positive: $k_\alpha - k_\rho > 0$ and $-k_\rho k_\beta > 0$, i.e. (given $k_\rho > 0$) $k_\beta < 0$.

That is exactly the slide's condition set: $k_\rho > 0,\ k_\beta < 0,\ k_\alpha - k_\rho > 0$. Now you know where it comes from.

**Worked example** with the Simulink gains $k_\rho = 1$, $k_\alpha = 5$, $k_\beta = -2$.
1. $\lambda_1 = -1$.
2. Quadratic: $\lambda^2 + (5 - 1)\lambda - (1)(-2) = \lambda^2 + 4\lambda + 2 = 0$.
3. $\lambda = \frac{-4 \pm \sqrt{16 - 8}}{2} = -2 \pm 1.41$, so $\lambda_2 = -0.59$ and $\lambda_3 = -3.41$.
All three are negative: the controller is stable, and the slowest mode ($-0.59$) sets how quickly the last bit of error dies away. In MATLAB: `A = [-1 0 0; 0 -4 2; 0 -1 0]; eig(A)`.

**Common mistake:** concluding stability from the *nonlinear* equations by eye. Always linearise, form $A$, and check eigenvalues (or use the three conditions, which are the same thing).

---

## 15. Control architecture for a differential-drive adhesion robot (slides 25–26)

The last two slides show a research robot rather than textbook theory: a four-wheeled differential-drive robot that can climb or stick to surfaces using an electric ducted fan producing an adhesion force $F_{EDF}$ (it is marked on the CAD drawing together with the CG, wheel speeds $\omega_1 \dots \omega_4$, the body frame $\{G\}$ and a slip angle $\beta$).

The block diagram has three parallel control loops feeding a block labelled "DVCR Setup" (the robot's low-level drive):

1. **Heading / slip loop:** reference $\beta_{ref}$ minus measured $\beta$ gives the error $e_\beta$, which passes through a **dead-zone** block (the symbol with flat middle; small errors are ignored to avoid jitter) and a **P** controller with gain $K_{P,1}$, producing $U_\beta$.
2. **Speed loop:** reference $v_{ref}$ minus measured $v$ gives $e_v$, handled by a **PI** controller (gains $K_{P,2}$, $K_{I,2}$), producing $U_v$. The integral term removes steady-state speed error, as in Section 10.
3. **Adhesion force loop:** reference $F_{ref}$ minus the **filtered** measured force gives $e_F$, handled by a **PID** controller (gains $K_{P,3}$, $K_{I,3}$, $K_{D,3}$), producing the thrust command $U_T$.

The measured $\beta$ and $F$ are passed through **filters** before being fed back, because raw sensor signals are noisy and a derivative term (the D in PID) would amplify that noise.

The message of the slide: real robots are controlled by several simple loops (P, PI, PID) running side by side, each in charge of one quantity, which is exactly the "divide and conquer" idea behind the four controllers in this lecture.

> ⚠️ The slides only show the picture; no equations or explanation are given for this architecture, and slide 26 is a video that does not play in the PDF. Treat this as illustration, not as examinable derivation, unless your lecturer says otherwise.

---

## 16. How this connects to Lab 2

Lab 2 ("Mobile Robot Motion and Control") asks you to implement, in Simulink on the TurtleBot3 Burger, exactly the controllers in this lecture:

- **Task 1a, moving to a point** = Section 8. **Task 1b, moving to a pose** = Section 11.
- **Task 2, following a trajectory** (square, your group number, a sinusoid) = Section 10, the pure-pursuit idea with a moving target.
- **Task 3, following a line** $y = ax + b$ = Section 9. Rewrite $y = ax + b$ as $ax - y + b = 0$ to get the $(a, b, c)$ form the distance formula needs.
- **Tasks 4–5** (lidar registration, wall following) use the pose from odometry, which is the "how do we implement it in the real world?" question from Section 11.8.

Your controllers will output $(V, \omega)$; the TurtleBot converts these to wheel speeds exactly as in Section 12.2. Expect to tune $K_v$, $K_h$, $k_\rho$, $k_\alpha$, $k_\beta$ and the look-ahead distance by hand, and remember to wrap heading errors to $[-\pi, \pi]$.

---

## In a nutshell

A wheeled robot's configuration is $q = (x, y, \theta)$, but a car or TurtleBot has only two inputs, so it cannot move sideways: it is non-holonomic. The unicycle model ($\dot x = V\cos\theta$, $\dot y = V\sin\theta$, $\dot\theta = \omega$) is the common core; the bicycle model replaces $\omega$ with $(v/L)\tan\gamma$, and the differential drive produces $V$ and $\omega$ from the sum and difference of its wheel speeds. Four classic controllers build on this: move to a point (speed ∝ distance, steering ∝ heading error), follow a line (steer on signed distance plus heading), follow a path (chase a look-ahead "carrot" with PI speed control), and move to a pose (polar coordinates $\rho, \alpha, \beta$ with $v = k_\rho\rho$, $\omega = k_\alpha\alpha + k_\beta\beta$). Because the models are nonlinear, you check a controller by linearising with the Jacobian and demanding negative-real-part eigenvalues, which for move-to-pose gives $k_\rho > 0$, $k_\beta < 0$, $k_\alpha > k_\rho$. Everything here is implemented directly in Lab 2.

## Key terms

- **Configuration $q$:** the minimal set of numbers that fixes the robot's position and orientation.
- **Degrees of freedom (DoF):** number of independent coordinates in $q$.
- **Actuator:** a motor or device that produces motion; a control input.
- **Holonomic:** can move instantly in any direction of its configuration space (actuators ≥ DoF).
- **Non-holonomic:** has velocity constraints (e.g. no sideways motion) even though all configurations are reachable eventually.
- **Unicycle model:** simplest wheeled model with inputs speed $V$ and turn rate $\omega$.
- **Bicycle / Ackermann model:** car-like model with inputs speed $v$ and steering angle $\gamma$; $\dot\theta = (v/L)\tan\gamma$.
- **Wheelbase $L$:** distance between front and rear axles.
- **Steering angle $\gamma$:** how far the front wheel is turned from straight ahead.
- **Instantaneous centre of rotation (ICR):** the point the vehicle is pivoting around at this instant.
- **Turning radius $R_B = L/\tan\gamma$:** radius of the circle the rear axle follows.
- **Non-holonomic (rolling) constraint:** $\dot x\sin\theta - \dot y\cos\theta = 0$, "no sideways velocity".
- **Slip angle $\beta$ (vehicle dynamics):** angle between where the body points and where it actually travels.
- **Heading error:** desired heading minus actual heading, wrapped to $[-\pi, \pi]$.
- **`atan2`:** four-quadrant inverse tangent; always use it to compute bearings.
- **Pure pursuit / donkey-and-carrot:** follow a target point kept a fixed look-ahead distance ahead on the path.
- **Look-ahead distance $d^*$:** how far ahead the pursuit target sits.
- **PI / PID controller:** proportional (+ integral) (+ derivative) feedback; the integral removes steady-state error.
- **Polar coordinates $\rho, \alpha, \beta$ (pose control):** distance to goal, pointing error to goal, final-heading error.
- **Differential drive:** two independently driven wheels; $V = \frac R2(v_r+v_l)$, $\omega = \frac RL(v_r - v_l)$.
- **Error dynamics in robot frame:** world pose error rotated by $R(\theta)^T$ into longitudinal, lateral and heading errors.
- **Linearization:** approximating a nonlinear system by its tangent (first-order Taylor) at an operating point.
- **Jacobian:** matrix of all first partial derivatives; the "slope" of a vector function.
- **Eigenvalues / $\mathrm{Re}(\mathrm{eig}(A)) < 0$:** decay rates of a linear system; all negative real parts means stable.
- **Dead-zone:** a block that outputs zero for small inputs, used to ignore tiny errors.

## Check yourself

1. A car has 3 DoF and 2 actuators. Explain in one or two sentences why it is non-holonomic, and give the mathematical constraint that expresses this for the rear wheel.
2. A robot with wheelbase $L = 1.5$ m drives at $v = 2$ m/s with steering angle $\gamma = 15°$. What is its turning radius and its turn rate $\dot\theta$?
3. In the move-to-pose controller, what do $\rho$, $\alpha$ and $\beta$ represent, and what are the three conditions on $k_\rho$, $k_\alpha$, $k_\beta$ for stability?
4. A TurtleBot ($R = 0.033$ m, wheel separation $L = 0.16$ m) must drive at $V = 0.1$ m/s while turning at $\omega = -0.25$ rad/s (to the right). What wheel angular velocities $v_r$ and $v_l$ are needed?
5. Why does the "follow a line" steering law need two terms, and what goes wrong if you use the absolute value of the distance $d$?

<details>
<summary>Answers</summary>

1. The car can only move in the direction its wheels point at any instant, so it cannot change $y$ without also changing $x$ or $\theta$ in a coordinated way: there is a velocity constraint, even though every $(x, y, \theta)$ is reachable by a sequence of moves. The rear-wheel constraint is $\dot x\sin\theta - \dot y\cos\theta = 0$ (no velocity perpendicular to the wheel).

2. $\tan 15° = 0.268$. Turning radius $R_B = 1.5/0.268 = 5.6$ m. Turn rate $\dot\theta = v/R_B = 2/5.6 = 0.36$ rad/s (equivalently $(2/1.5)\cdot 0.268 = 0.36$ rad/s), about 20.5° per second.

3. $\rho$ is the straight-line distance to the goal; $\alpha$ is the angle between the robot's heading and the line to the goal (pointing error); $\beta$ is the angle between that line and the goal's required heading (final-heading error). Stability requires $k_\rho > 0$, $k_\beta < 0$ and $k_\alpha - k_\rho > 0$, which is the same as all eigenvalues of the linearised system matrix having negative real part.

4. $2V = 0.2$, $\omega L = -0.25\cdot 0.16 = -0.04$, $2R = 0.066$. $v_r = (0.2 - 0.04)/0.066 = 2.42$ rad/s; $v_l = (0.2 + 0.04)/0.066 = 3.64$ rad/s. The left wheel is faster, so the robot turns right, as required. Check: $V = 0.0165(2.42 + 3.64) = 0.100$ m/s, $\omega = 0.206(2.42 - 3.64) = -0.25$ rad/s.

5. The distance term $-K_d d$ pulls the robot towards the line, and the heading term $K_h(\theta^* - \theta)$ aligns it parallel to the line. With only the distance term the robot overshoots and oscillates across the line; with only the heading term it drives parallel but never closes the gap. If you take $|d|$ you throw away the sign that says which side of the line you are on, so the controller steers the same way from both sides and cannot converge; use the signed distance $d = (ax^o + by^o + c)/\sqrt{a^2+b^2}$.

</details>
