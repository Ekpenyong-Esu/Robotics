# Lecture 10 – Localization (and Mapping, and SLAM)

*Study notes based on `10_R7010E_Lecture_10_Localization.pdf` (35 slides) and the MATLAB script `Lecture_localization_matlab_time.m`. Written as if a tutor were sitting next to you and talking you through it.*

---

## What this lecture is about, and why it matters

This lecture answers the very first of the "fundamental questions of robotics" from Lecture 1: **"Where am I?"** A robot can only plan a path or follow a trajectory if it knows its own position and heading, and it turns out that it can never know these exactly, because wheels slip, sensors are noisy and the world is messy. So the whole lecture is really about one big idea: **instead of pretending the robot knows where it is, we keep track of a best guess plus an honest measure of how unsure we are, and we keep improving both with every measurement.** The mathematical tool for doing that is the Kalman filter, and by the end you will see how the same tool localizes a robot on a known map, builds a map from a known position, and finally does both at once (SLAM), which is the foundation of almost every autonomous navigation system in use today.

---

## Roadmap: the main topics in this lecture

1. Why localization is uncertain, and why we describe the robot's position with probability (PDF, covariance matrix, uncertainty ellipse).
2. Dead reckoning: estimating pose from odometry alone, and why the error grows forever.
3. MATLAB: the `Bicycle` model with noisy odometry.
4. The Kalman filter for linear systems (prediction, innovation, gain, update; why it is optimal).
5. The Extended Kalman Filter (EKF) for nonlinear systems (linearize with Jacobians at every step).
6. EKF dead reckoning in MATLAB: the growing uncertainty ellipse.
7. Localizing with a known map using a range-and-bearing sensor (sensor model, innovation, Jacobians $H_x$ and $H_w$, how observations shrink the uncertainty).
8. Creating a map when the pose is perfectly known (growing state vector, state and covariance extension, insertion Jacobians).
9. Simultaneous Localization and Mapping (SLAM): doing both at once.
10. Pose-graph SLAM (front end, back end, loop closures, optimization).
11. Laser odometry with Iterated Closest Point (ICP) matching.
12. Turning laser scans into an occupancy grid.
13. Real-world examples: underground mine 3D reconstruction and aerial inspection with drones.

---

## 1. Why we need probability: "the robot is somewhere here"

### 1.1 The problem (slide 2)

Think about how a robot knows where it is. It has wheel encoders that count wheel turns, it has sensors that measure distances to things, and it has a model of how it moves. Every one of those is imperfect. The wheels slip a little on a smooth floor. The laser range finder returns a distance that is off by a centimetre or two. The motion model assumes a perfect circle when the real robot wobbled.

The slides put it in two bullet points: there is **uncertainty in the robot's motions and observations**, and we deal with it by **using probability theory to explicitly represent the uncertainty**.

The picture on slide 2 contrasts two attitudes:

- The naive attitude: "The robot is exactly here" (a single point in the $x$–$y$–$z$ frame).
- The honest attitude: "The robot is somewhere here" (a bell-shaped curve, the normal distribution, centred on the best guess).

**Everyday analogy.** If a friend asks where you parked, you might say "about the third row, near the lamp post". You do not give GPS coordinates to the millimetre, because you do not know them. But your answer has a *centre* (third row, by the lamp post) and a *spread* (maybe two or three spaces either way). That is exactly what a probability distribution gives a robot: a centre (the mean) and a spread (the variance).

### 1.2 PDF versus probability (slide 3)

The bell curve is called a **probability density function (PDF)**. In plain words, a PDF is a curve whose *height* tells you how *dense* the probability is around a location. The slides stress a point that people get wrong all the time:

> The PDF is **not** the probability of being at that location. The **volume under the PDF over a region** is the probability of being in that region.

So the height of the bell at a point is not "the probability of being at that exact point" (for a continuous variable that probability is actually zero). What you can ask is "what is the probability the robot is inside this square metre?" and the answer is the volume under the 2D bell over that square.

Slide 3 shows three 2D bell shapes over an $x$–$y$ floor:

- **(a)** A tall, narrow spike: the robot is almost certainly within a small circle. Little uncertainty.
- **(b)** A low, wide hill: the robot could be anywhere in a big region. Large uncertainty. (Notice the height is tiny, about $10^{-4}$, because the same total volume of 1 is spread over a much bigger area.)
- **(c)** Several separate bumps: the robot could be in one of several places (a *multi-modal* belief). This happens, for example, in a building with identical corridors.

**Common mistake.** People read the tall spike as "high probability" and the low hill as "low probability". Both have total volume exactly 1 (the robot is definitely *somewhere*). The difference is how *concentrated* that volume is. Tall and narrow means confident; low and wide means unsure.

### 1.3 The covariance matrix (slide 3, right)

For a one-dimensional quantity the spread is a single number, the variance $\sigma^2$. For a robot pose $(x, y, \theta)$ you have three quantities, and they can be *linked*: for example, if the heading estimate is off, the $x$ and $y$ estimates will be off in a related way. The object that holds all of this is the **covariance matrix**.

Write the random quantities as a vector

$$
\mathbf{X} = \begin{pmatrix} X_1 \\ \vdots \\ X_n \end{pmatrix}.
$$

Then the covariance matrix $\Sigma$ has entries

$$
\Sigma_{ij} = \mathrm{cov}(X_i, X_j) = \mathbb{E}\big[(X_i - \mu_i)(X_j - \mu_j)\big],
$$

where $\mu_i$ is the mean (the expected value) of $X_i$, and $\mathbb{E}[\cdot]$ means "the average over all the randomness".

In words:

- The **diagonal** entries $\Sigma_{ii}$ are plain variances: how spread out $X_i$ is on its own.
- The **off-diagonal** entries $\Sigma_{ij}$ say how $X_i$ and $X_j$ move *together*. Zero means knowing one tells you nothing about the other. Positive means they tend to be high together; negative means one tends to be high when the other is low.

When you draw the set of points where the 2D PDF has a given height you get an **ellipse**, the famous **uncertainty ellipse** (the pink rings under the bells on slide 3). A circle means equal, unrelated uncertainty in $x$ and $y$. A tilted elongated ellipse means more uncertainty in one direction and some correlation. Throughout the lecture, the covariance matrix is called $\mathbf{P}$ when it describes the uncertainty of the robot's estimate.

---

## 2. Dead reckoning: guessing your position from how you moved

### 2.1 The idea (slide 4)

**Dead reckoning** is the oldest navigation method there is: sailors used it for centuries. You know where you started, you know your speed, your heading and how long you travelled, so you can work out where you must be now. In the slide's words:

> Robot Pose$(k)$ = $f$(speed, direction, time of travel, pose$(k-1)$).

Here $k$ is a discrete time step (sample number) and $f$ is just "some function of".

For a wheeled robot the "speed × time" part comes from the **odometer** (wheel encoders that count how far the wheels have rolled), so the method is also called **odometry-based localization**.

### 2.2 The odometry measurement

At every step the odometer gives the robot two numbers:

$$
\boldsymbol{\delta}\langle k\rangle = (\delta_d, \delta_\theta)^T \in \mathbb{R}^{2\times 1}.
$$

- $\delta_d$ is the **distance travelled** during the step.
- $\delta_\theta$ is the **change in heading** during the step.

The angle brackets $\langle k \rangle$ are the textbook's way of writing "at time step $k$". Treat $\boldsymbol{\delta}\langle k\rangle$ as "the odometry reading for step $k$".

### 2.3 Deriving the pose update with homogeneous transforms (slide 4)

Here the lecture reuses Lecture 1. A 2D pose can be written as a $3\times3$ homogeneous transformation matrix:

$$
\boldsymbol{\xi}\langle k\rangle \sim
\begin{pmatrix}
\cos\theta\langle k\rangle & -\sin\theta\langle k\rangle & x\langle k\rangle \\
\sin\theta\langle k\rangle & \cos\theta\langle k\rangle & y\langle k\rangle \\
0 & 0 & 1
\end{pmatrix}.
$$

Moving forward by $\delta_d$ along the robot's own $x$-axis and then turning by $\delta_\theta$ is a *compounding* of relative poses, so you multiply matrices:

$$
\boldsymbol{\xi}\langle k+1\rangle \sim
\underbrace{\begin{pmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta & \cos\theta & y \\
0 & 0 & 1
\end{pmatrix}}_{\text{current pose}}
\underbrace{\begin{pmatrix}
1 & 0 & \delta_d \\
0 & 1 & 0 \\
0 & 0 & 1
\end{pmatrix}}_{\text{drive forward}}
\underbrace{\begin{pmatrix}
\cos\delta_\theta & -\sin\delta_\theta & 0 \\
\sin\delta_\theta & \cos\delta_\theta & 0 \\
0 & 0 & 1
\end{pmatrix}}_{\text{turn}}.
$$

Multiplying out (slide 4 does this for you) gives

$$
\boldsymbol{\xi}\langle k+1\rangle \sim
\begin{pmatrix}
\cos(\theta + \delta_\theta) & -\sin(\theta + \delta_\theta) & x + \delta_d\cos\theta \\
\sin(\theta + \delta_\theta) & \cos(\theta + \delta_\theta) & y + \delta_d\sin\theta \\
0 & 0 & 1
\end{pmatrix}.
$$

Reading off the position (last column) and the angle, and switching to the compact vector form $\mathbf{x} = (x, y, \theta)$, you get the boxed result:

$$
\boxed{\;
\mathbf{x}\langle k+1\rangle =
\begin{pmatrix}
x\langle k\rangle + \delta_d\cos\theta\langle k\rangle \\
y\langle k\rangle + \delta_d\sin\theta\langle k\rangle \\
\theta\langle k\rangle + \delta_\theta
\end{pmatrix}\;}
$$

In words: *the new $x$ is the old $x$ plus the distance travelled times the cosine of the old heading; the new $y$ is the old $y$ plus the distance times the sine of the old heading; the new heading is the old heading plus the turn.*

The small star on slide 4 adds: **for small displacements the order of applying the displacements is not significant**. In other words, "drive then turn" and "turn then drive" give almost the same answer when $\delta_d$ and $\delta_\theta$ are tiny, which is why we can get away with this simple model when the time step is short.

**Worked example.** Suppose the robot is at $x = 1$ m, $y = 2$ m, heading $\theta = 30^\circ$, and the odometer says it moved $\delta_d = 0.5$ m and turned $\delta_\theta = 10^\circ$.

1. $\cos 30^\circ = 0.866$, so $x\langle k+1\rangle = 1 + 0.5 \times 0.866 = 1.433$ m.
2. $\sin 30^\circ = 0.5$, so $y\langle k+1\rangle = 2 + 0.5 \times 0.5 = 2.25$ m.
3. $\theta\langle k+1\rangle = 30^\circ + 10^\circ = 40^\circ$.

New pose: $(1.433, 2.25, 40^\circ)$. Notice that the *old* heading is used for the position update; the new heading only matters for the *next* step.

### 2.4 Adding the noise (slide 5)

The odometer reading is never exact. The lecture models the error as additive **corrupting noise**

$$
\mathbf{v}\langle k\rangle = (v_d, v_\theta)^T \in \mathbb{R}^{2\times1} \sim N(\mathbf{0}, \mathbf{V}),
$$

which reads "$v$ is drawn from a normal (Gaussian) distribution with mean zero and covariance $\mathbf{V}$". A **zero-mean Gaussian process** means the errors are bell-shaped, equally likely to be positive or negative, and average out to zero in the long run. The covariance is

$$
\mathbf{V} = \begin{pmatrix} \sigma_d^2 & 0 \\ 0 & \sigma_\theta^2 \end{pmatrix},
$$

where $\sigma_d$ is the standard deviation (typical size) of the distance error and $\sigma_\theta$ of the heading error. The zeros off the diagonal say the **errors in $d$ and $\theta$ are independent**: a bad distance reading tells you nothing about whether the turn reading is bad.

Putting the noise into the motion model gives the **robot configuration at $k+1$**:

$$
\boxed{\;
\mathbf{x}\langle k+1\rangle = \mathbf{f}\big(\mathbf{x}\langle k\rangle, \boldsymbol{\delta}\langle k\rangle, \mathbf{v}\langle k\rangle\big) =
\begin{pmatrix}
x\langle k\rangle + (\delta_d + v_d)\cos\theta\langle k\rangle \\
y\langle k\rangle + (\delta_d + v_d)\sin\theta\langle k\rangle \\
\theta\langle k\rangle + \delta_\theta + v_\theta
\end{pmatrix}\;}
$$

So the robot really travelled $\delta_d + v_d$, not $\delta_d$, and really turned $\delta_\theta + v_\theta$. We do not know $v_d$ and $v_\theta$ (if we did, we would just subtract them), we only know their statistics.

### 2.5 Why dead reckoning drifts (slide 7)

Slide 7 shows a blue "real path" and a red "estimated path" that start together and then drift apart more and more. This is the fundamental weakness of dead reckoning: **every step adds a little error, and nothing ever removes it**. The errors accumulate without bound. A small heading error is especially nasty: being 1° off in heading puts you 17 cm off after walking 10 m, and it keeps getting worse.

**Everyday analogy.** Walk across a dark room with your eyes closed, counting steps and turns. After three steps you are roughly right. After thirty you could be anywhere. You need to *open your eyes* and look at something fixed, which is exactly what the sensor-based updates in Section 7 will do.

---

## 3. MATLAB time: dead reckoning with the Bicycle model (slide 6, script section 6.1.1)

```matlab
V = diag([0.02, 0.5*pi/180].^2);   % odometry noise covariance: sigma_d = 2 cm, sigma_theta = 0.5 deg
veh = Bicycle('covar', V)          % a bicycle-model vehicle whose odometry is corrupted with noise V
randinit                           % reset the random number generator so results repeat
odo = veh.step(1, 0.3)             % drive one step with speed 1 and steer angle 0.3; returns the noisy odometry (delta_d, delta_theta)
veh.x'                             % the TRUE pose (x, y, theta) that the simulator knows
veh.f([0 0 0], odo)                % apply the motion model f to pose (0,0,0) with that odometry: the dead-reckoned pose
veh.add_driver( RandomPath(10) )   % attach a "driver" that wanders randomly in a 10x10 area
veh.run()                          % simulate and animate
```

What to take from this:

- `diag([...].^2)` builds $\mathbf{V}$ by squaring the standard deviations: $\sigma_d = 0.02$ m and $\sigma_\theta = 0.5^\circ$ (converted to radians). The `.^2` matters because a covariance holds *variances*, not standard deviations.
- `veh.step` returns the *noisy* odometry; `veh.x` is the ground truth that only the simulator knows. Comparing `veh.f([0 0 0], odo)` with `veh.x'` shows you the dead-reckoning error after a single step.
- `veh.f` *is* the boxed function $\mathbf{f}$ from Section 2.4 (without the noise, because we feed it the measured odometry).

---

## 4. The Kalman filter (linear case): the engine behind everything

### 4.1 The setting (slide 10)

Slide 10 is a picture worth memorising. On the left is the physical robot:

- The **true state** evolves as $\mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_k, \mathbf{u}, \mathbf{v})$, where $\mathbf{u}$ is the known **input** (what we commanded or measured with the odometer) and $\mathbf{v}$ is random **process noise** drawn from $N(\mathbf{0}, \mathbf{V})$. The slide notes that *"the true state is not directly measurable"*.
- A **sensor** produces $\mathbf{z}_{k+1} = \mathbf{h}(\mathbf{x}_{k+1}, \mathbf{w})$, where $\mathbf{h}$ is the sensor model and $\mathbf{w}$ is random **sensor noise** drawn from $N(\mathbf{0}, \mathbf{W})$.

On the right is the **estimator** (the Kalman filter). It owns *estimates* of the models, $\mathbf{f}(\cdot)$ and $\mathbf{h}(\cdot)$, and of the noise covariances, $\hat{\mathbf{V}}$ and $\hat{\mathbf{W}}$ (hats mean "our estimate of"). It is fed the input $\mathbf{u}$ and the sensor output $\mathbf{z}$, and it produces two things: the **state estimate** $\hat{\mathbf{x}}\langle k+1\rangle$ and the **uncertainty estimate** $\hat{\mathbf{P}}\langle k+1\rangle$ (the covariance of the estimate).

The problem statement in the box at the bottom of slide 10 is the one to remember:

> Given a model of the system $\mathbf{f}(\cdot)$, $\mathbf{h}(\cdot)$, $\hat{\mathbf{V}}$ and $\hat{\mathbf{W}}$; the known inputs $\mathbf{u}$; and some noisy sensor measurements $\mathbf{z}$, find an estimate $\hat{\mathbf{x}}$ of the system state and our uncertainty $\hat{\mathbf{P}}$ in that estimate.

### 4.2 The linear model (slide 11)

Start with the simplest case, where everything is linear (matrices times vectors):

$$
\mathbf{x}\langle k+1\rangle = \mathbf{F}\mathbf{x}\langle k\rangle + \mathbf{G}\mathbf{u}\langle k\rangle + \mathbf{v}\langle k\rangle, \qquad
\mathbf{z}\langle k\rangle = \mathbf{H}\mathbf{x}\langle k\rangle + \mathbf{w}\langle k\rangle.
$$

- $\mathbf{F}$ says how the state carries itself forward one step.
- $\mathbf{G}$ says how the input pushes the state.
- $\mathbf{H}$ says how the state shows up in the sensor reading.
- $\mathbf{v}$ and $\mathbf{w}$ are the process and sensor noise, zero-mean Gaussian with covariances $\mathbf{V}$ and $\mathbf{W}$.

### 4.3 The two phases

The Kalman filter alternates between two phases, forever.

**Phase 1: Prediction** (also called the time update). Use the model to push the estimate forward in time, *before* looking at the sensor:

$$
\hat{\mathbf{x}}^+\langle k+1\rangle = \mathbf{F}\hat{\mathbf{x}}\langle k\rangle + \mathbf{G}\mathbf{u}\langle k\rangle,
$$

$$
\hat{\mathbf{P}}^+\langle k+1\rangle = \underbrace{\mathbf{F}\hat{\mathbf{P}}\langle k\rangle\mathbf{F}^T}_{\text{old uncertainty, carried forward}} + \hat{\mathbf{V}}.
$$

The superscript $^+$ marks a *predicted* quantity (a "prior"). In words: the predicted state is the model applied to the old estimate (we drop the noise because its mean is zero); the predicted covariance is the old covariance pushed through the model *plus* the process noise. **Prediction always makes you less certain**: you add $\hat{\mathbf{V}}$ and never subtract anything.

**Phase 2: Update** (also called the measurement update). Now look at the sensor and correct the prediction.

First compute the **innovation**:

$$
\boldsymbol{\nu} = \mathbf{z}^{\#}\langle k+1\rangle - \mathbf{H}\hat{\mathbf{x}}^+\langle k+1\rangle \in \mathbb{R}^p.
$$

The innovation is simply *what the sensor actually measured* minus *what we expected the sensor to measure given our prediction*. If it is zero, the sensor agrees with us and there is nothing to learn. If it is large, something is off. ($p$ is the number of sensor values.)

Then compute the **Kalman gain**:

$$
\mathbf{K} = \mathbf{P}^+\langle k+1\rangle\mathbf{H}^T\Big(\underbrace{\mathbf{H}\mathbf{P}^+\langle k+1\rangle\mathbf{H}^T + \hat{\mathbf{W}}}_{\text{estimated covariance of the innovation}}\Big)^{-1} \in \mathbb{R}^{n\times p}.
$$

The bracketed term is the **innovation covariance**, often called $\mathbf{S}$: how much scatter we expect in the innovation, coming partly from our own uncertainty (pushed through $\mathbf{H}$) and partly from the sensor noise $\hat{\mathbf{W}}$. The gain $\mathbf{K}$ is the ratio "our uncertainty ÷ total uncertainty". It decides **how much to trust the sensor versus the prediction**.

Finally apply the correction:

$$
\hat{\mathbf{x}}\langle k+1\rangle = \hat{\mathbf{x}}^+\langle k+1\rangle + \mathbf{K}\boldsymbol{\nu},
$$

$$
\hat{\mathbf{P}}\langle k+1\rangle = \hat{\mathbf{P}}^+\langle k+1\rangle - \mathbf{K}\mathbf{H}\hat{\mathbf{P}}^+\langle k+1\rangle.
$$

In words: *nudge the predicted state by the gain times the innovation; shrink the covariance by an amount that depends on the gain.* **Updates always make you more certain** (you subtract from $\mathbf{P}$).

**Worked example in one dimension.** Suppose you are tracking only $x$, so every matrix is a single number, $\mathbf{F} = \mathbf{H} = 1$.

1. After prediction you believe $\hat{x}^+ = 5.0$ m with variance $P^+ = 4$ (so a standard deviation of 2 m: not very sure).
2. A sensor reports $z = 6.0$ m, and its noise variance is $W = 1$ (standard deviation 1 m: more accurate than your prediction).
3. Innovation: $\nu = 6.0 - 5.0 = 1.0$ m. The sensor says you are a metre further on than you thought.
4. Innovation covariance: $S = H P^+ H^T + W = 4 + 1 = 5$.
5. Gain: $K = P^+ H^T / S = 4 / 5 = 0.8$. You trust the sensor 80 %, because it is more precise than your prediction.
6. Updated state: $\hat{x} = 5.0 + 0.8 \times 1.0 = 5.8$ m. You moved most of the way toward the sensor's answer.
7. Updated variance: $P = P^+ - K H P^+ = 4 - 0.8 \times 4 = 0.8$. Your standard deviation dropped from 2 m to about 0.9 m.

Now flip the numbers: if the sensor were terrible ($W = 100$), $K = 4/104 \approx 0.04$, and you would barely move your estimate. That is the beauty of the gain: it automatically weighs the two sources of information by how reliable each is.

### 4.4 Why it is "optimal" (slide 11, left)

The slide says: *process/sensor noise zero-mean and Gaussian* $\Rightarrow$ *the Kalman filter is an optimal estimator*. "Optimal" here means that, under those assumptions (linear model, zero-mean Gaussian noise with known covariances), no other estimator gives a smaller mean-squared error. If the noise is not Gaussian or the model is not linear, the filter still often works well, but the optimality guarantee is gone.

**Common mistake.** Students sometimes think the Kalman filter "filters out noise" like a low-pass filter smooths a signal. It is really an *estimator*: it combines two uncertain pieces of information (model prediction and sensor) in the statistically best way, keeping track of uncertainty as it goes.

---

## 5. The Extended Kalman Filter (EKF): when the model is nonlinear

### 5.1 The problem (slide 12)

Look back at the dead-reckoning model: it contains $\cos\theta$ and $\sin\theta$. That is **not** linear, so you cannot write it as $\mathbf{F}\mathbf{x} + \mathbf{G}\mathbf{u}$. The slide poses exactly this question: "@Non-linear systems?"

The general nonlinear model is

$$
\mathbf{x}\langle k+1\rangle = \mathbf{f}\big(\mathbf{x}\langle k\rangle, \mathbf{u}\langle k\rangle, \mathbf{v}\langle k\rangle\big), \qquad
\mathbf{z}\langle k\rangle = \mathbf{h}\big(\mathbf{x}\langle k\rangle, \mathbf{w}\langle k\rangle\big).
$$

### 5.2 The trick: linearize around the current estimate

The EKF idea is simple: **near the current estimate $\hat{\mathbf{x}}_k$, pretend the curve is a straight line** (its tangent). Mathematically that is a first-order Taylor expansion, and the slopes of the tangent are **Jacobians**: matrices of partial derivatives. In plain words, a Jacobian answers "if I wiggle each input a little, how much does each output change?"

The linearized model around $\hat{\mathbf{x}}_k$ becomes

$$
\mathbf{x}'\langle k+1\rangle \approx \mathbf{F}_x\mathbf{x}'\langle k\rangle + \mathbf{F}_u\mathbf{u}\langle k\rangle + \mathbf{F}_v\mathbf{v}\langle k\rangle, \qquad
\mathbf{z}'\langle k\rangle \approx \mathbf{H}_x\mathbf{x}'\langle k\rangle + \mathbf{H}_w\mathbf{w}\langle k\rangle,
$$

which has exactly the same *shape* as the linear model in Section 4.2, with the Jacobians playing the roles of $\mathbf{F}$, $\mathbf{G}$ and $\mathbf{H}$:

$$
\mathbf{F}_x = \frac{\partial\mathbf{f}}{\partial\mathbf{x}} \in \mathbb{R}^{n\times n},\quad
\mathbf{F}_u = \frac{\partial\mathbf{f}}{\partial\mathbf{u}} \in \mathbb{R}^{n\times m},\quad
\mathbf{F}_v = \frac{\partial\mathbf{f}}{\partial\mathbf{v}} \in \mathbb{R}^{n\times n},\quad
\mathbf{H}_x = \frac{\partial\mathbf{h}}{\partial\mathbf{x}} \in \mathbb{R}^{p\times n},\quad
\mathbf{H}_w = \frac{\partial\mathbf{h}}{\partial\mathbf{w}} \in \mathbb{R}^{p\times p}.
$$

Here $n$ is the number of states, $m$ the number of inputs, and $p$ the number of sensor outputs.

A quick reminder of what a Jacobian looks like (slide 8, dashed box): if $\mathbf{f}(\mathbf{x})$ has components $f_1, \dots, f_m$ and $\mathbf{x}$ has components $x_1, \dots, x_n$, then

$$
\mathbf{J} = \begin{pmatrix}
\frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial f_m}{\partial x_1} & \cdots & \frac{\partial f_m}{\partial x_n}
\end{pmatrix}.
$$

Row $i$ is the gradient of $f_i$: it tells you how output $i$ responds to each input.

### 5.3 The EKF equations (slides 12 and 13)

**Prediction** (evaluate with $\mathbf{v}\langle k\rangle = \mathbf{0}$, since the noise has zero mean):

$$
\hat{\mathbf{x}}^+\langle k+1\rangle = \mathbf{f}\big(\hat{\mathbf{x}}\langle k\rangle, \mathbf{u}\langle k\rangle\big),
$$

$$
\hat{\mathbf{P}}^+\langle k+1\rangle = \mathbf{F}_x\hat{\mathbf{P}}\langle k\rangle\mathbf{F}_x^T + \mathbf{F}_v\hat{\mathbf{V}}\mathbf{F}_v^T.
$$

Notice two differences from the linear filter. First, the state is pushed through the *real nonlinear* $\mathbf{f}$, not the linear approximation. Second, the process noise is now shaped by $\mathbf{F}_v$ (it enters through $\mathbf{F}_v\hat{\mathbf{V}}\mathbf{F}_v^T$ rather than plain $\hat{\mathbf{V}}$), because the noise is *inside* $\mathbf{f}$ rather than simply added on.

**Update:**

$$
\boldsymbol{\nu} = \mathbf{z}\langle k+1\rangle - \mathbf{h}\big(\hat{\mathbf{x}}^+\langle k+1\rangle\big),
$$

$$
\mathbf{K} = \mathbf{P}^+\langle k+1\rangle\mathbf{H}_x^T\Big[\mathbf{H}_x\mathbf{P}^+\langle k+1\rangle\mathbf{H}_x^T + \mathbf{H}_w\hat{\mathbf{W}}\mathbf{H}_w^T\Big]^{-1},
$$

$$
\hat{\mathbf{x}}\langle k+1\rangle = \hat{\mathbf{x}}^+\langle k+1\rangle + \mathbf{K}\boldsymbol{\nu},
$$

$$
\hat{\mathbf{P}}\langle k+1\rangle = \hat{\mathbf{P}}^+\langle k+1\rangle - \mathbf{K}\mathbf{H}_x\hat{\mathbf{P}}^+\langle k+1\rangle.
$$

Again the innovation uses the *real nonlinear* sensor model $\mathbf{h}$, while the covariance maths uses the Jacobians $\mathbf{H}_x$ and $\mathbf{H}_w$.

Slide 13 lays this out as a procedure:

- **Input:** $\hat{\mathbf{x}}\langle k\rangle \in \mathbb{R}^n$, $\hat{\mathbf{P}}\langle k\rangle \in \mathbb{R}^{n\times n}$, $\mathbf{u}\langle k\rangle \in \mathbb{R}^m$, $\mathbf{z}\langle k+1\rangle \in \mathbb{R}^p$, $\hat{\mathbf{V}} \in \mathbb{R}^{n\times n}$, $\hat{\mathbf{W}} \in \mathbb{R}^{p\times p}$.
- **Output:** $\hat{\mathbf{x}}\langle k+1\rangle$, $\hat{\mathbf{P}}\langle k+1\rangle$.
- **Steps:** linearize about $\mathbf{x} = \hat{\mathbf{x}}\langle k\rangle$; compute the Jacobians $\mathbf{F}_x, \mathbf{F}_v, \mathbf{H}_x, \mathbf{H}_w$; do the prediction step; do the update step.

### 5.4 Two warnings written in big letters on slide 13

1. **"Equations valid only at linearization point $\hat{\mathbf{x}}\langle k\rangle$."** The tangent line is a good stand-in for the curve only *close to* the point where you drew it. If your estimate is far from the truth, the linearization is wrong, the covariance maths is wrong, and the filter can diverge (its estimate runs away and never recovers).
2. **"Jacobians must be calculated at every step."** Because the linearization point moves every step, $\mathbf{F}_x, \mathbf{F}_v, \mathbf{H}_x, \mathbf{H}_w$ change every step and must be recomputed. There is no "compute once and reuse" as in the linear filter.

**Everyday analogy.** Estimating a curved coastline with a ruler works fine if you move the ruler along and re-align it every few metres. Hold the ruler in one place and try to describe the whole coast with it and you will be badly wrong. The EKF re-aligns the ruler (recomputes the Jacobians) at every step.

**Common mistake.** The EKF is *not* optimal in general, unlike the linear Kalman filter. It is an approximation that works well when the nonlinearity is mild over the scale of the uncertainty.

---

## 6. EKF dead reckoning: watching the ellipse grow (slides 8 and 9, script section 6.1.2)

Now apply the EKF to the odometry model with **no sensor at all** (prediction only). The state is $\mathbf{x} = (x, y, \theta)$, so $n = 3$; the "input" is the odometry $(\delta_d, \delta_\theta)$; the noise is $\mathbf{v} = (v_d, v_\theta)$.

### 6.1 The Jacobians (slide 8)

Differentiate the model in Section 2.4 with respect to the state, evaluated at $\mathbf{v} = \mathbf{0}$:

$$
\mathbf{F}_x = \left.\frac{\partial\mathbf{f}}{\partial\mathbf{x}}\right|_{\mathbf{v}=0} =
\begin{pmatrix}
1 & 0 & -\delta_d\sin\theta_v \\
0 & 1 & \delta_d\cos\theta_v \\
0 & 0 & 1
\end{pmatrix}.
$$

Walk through it: the new $x$ depends on the old $x$ with slope 1 (first column), not on $y$ (second column is 0), and on $\theta$ with slope $\partial(\delta_d\cos\theta)/\partial\theta = -\delta_d\sin\theta$. Similarly for $y$. The new $\theta$ depends only on the old $\theta$ (bottom row). The third column is the interesting one: it says **heading uncertainty leaks into position uncertainty**, and more so the further you drive ($\delta_d$ multiplies it).

Differentiate with respect to the noise:

$$
\mathbf{F}_v = \left.\frac{\partial\mathbf{f}}{\partial\mathbf{v}}\right|_{\mathbf{v}=0} =
\begin{pmatrix}
\cos\theta_v & 0 \\
\sin\theta_v & 0 \\
0 & 1
\end{pmatrix}.
$$

Distance noise $v_d$ pushes $x$ by $\cos\theta$ and $y$ by $\sin\theta$ (it acts along the heading). Heading noise $v_\theta$ goes straight into $\theta$.

Then the prediction is

$$
\hat{\mathbf{x}}^+\langle k+1\rangle = \mathbf{f}\big(\hat{\mathbf{x}}\langle k\rangle, \hat{\mathbf{u}}\langle k\rangle\big), \qquad
\hat{\mathbf{P}}^+\langle k+1\rangle = \mathbf{F}_x\hat{\mathbf{P}}\langle k\rangle\mathbf{F}_x^T + \mathbf{F}_v\hat{\mathbf{V}}\mathbf{F}_v^T.
$$

**Worked example (continuing from Section 2.3).** Heading $\theta = 30^\circ$, $\delta_d = 0.5$ m, so

$$
\mathbf{F}_x = \begin{pmatrix} 1 & 0 & -0.25 \\ 0 & 1 & 0.433 \\ 0 & 0 & 1 \end{pmatrix}, \qquad
\mathbf{F}_v = \begin{pmatrix} 0.866 & 0 \\ 0.5 & 0 \\ 0 & 1 \end{pmatrix}.
$$

Take $\hat{\mathbf{P}}\langle k\rangle = \mathrm{diag}(0.01, 0.01, 0.001)$ (standard deviations of 10 cm, 10 cm and about $1.8^\circ$) and $\hat{\mathbf{V}} = \mathrm{diag}(0.02^2, (0.5^\circ)^2) = \mathrm{diag}(4\times10^{-4}, 7.6\times10^{-5})$. Multiplying out:

- $\mathbf{F}_x\hat{\mathbf{P}}\mathbf{F}_x^T$ has diagonal $\approx (0.01006, 0.01019, 0.001)$ and small off-diagonal terms such as $-0.00025$ between $x$ and $\theta$. The heading variance was carried into $x$ and $y$ and created correlations.
- $\mathbf{F}_v\hat{\mathbf{V}}\mathbf{F}_v^T$ has diagonal $\approx (0.0003, 0.0001, 0.000076)$: fresh noise added mainly along the heading direction.
- Sum: $\hat{\mathbf{P}}^+ \approx \mathrm{diag}(0.01036, 0.01029, 0.00108)$ plus off-diagonals.

Every variance **grew**, and off-diagonal entries appeared. Repeat this a thousand times with nothing to shrink $\mathbf{P}$ and you get the picture on slide 9.

### 6.2 MATLAB (slide 9, script 6.1.2)

```matlab
veh.Fx( [0,0,0], [0.5, 0.1] )     % the Jacobian F_x at pose (0,0,0) for odometry (0.5, 0.1): check it matches the formula
P0 = diag([0.005, 0.005, 0.001].^2);   % initial uncertainty: 5 mm in x,y and 0.001 rad in theta
ekf = EKF(veh, V, P0);             % an EKF that knows the vehicle model and noise V; no sensor, no map
randinit
ekf.run(1000);                     % 1000 steps of prediction only
clf; veh.plot_xy()                 % true path (blue)
hold on; ekf.plot_xy('r')          % estimated path (red)
P700 = ekf.history(700).P          % the 3x3 covariance at step 700
sqrt(P700(1,1))                    % standard deviation of x at step 700
ekf.plot_ellipse('g')              % draw the uncertainty ellipses along the path
ekf.plot_P();                      % plot sqrt(det P) against time: overall uncertainty
```

Slide 9 shows the result. The red estimate wanders away from the blue truth, and the green ellipses grow larger and larger along the route. The small graph "Overall Uncertainty" plots $(\det\mathbf{P})^{0.5}$, a single number summarising the size of the ellipse, climbing steadily for 1000 steps. This is the mathematical signature of dead reckoning: **uncertainty grows without bound.**

⚠️ The "Overall Uncertainty" plot on slide 9 has three curves labelled 1, 2 and 0.5 with no explanation. They most likely show the same experiment with the odometry noise scaled by those factors (bigger noise, faster growth), but the slide does not say. Check with your lecturer if you want to be sure.

---

## 7. Localizing with a map: opening your eyes

### 7.1 The idea (slide 14)

Slide 14 shows the same blue and red paths, but now there are yellow stars, **landmarks**, whose positions are known from a map. Each time the robot observes a landmark, the red estimate jumps back toward the blue truth. In between observations it drifts again (dead reckoning), and the next observation pulls it back.

A **landmark** is any distinctive feature whose map position is known: a pillar, a reflector, a corner, a Wi-Fi beacon. In this lecture each landmark $i$ is a point with known coordinates $\mathbf{p}_i = (x_i, y_i)^T$.

### 7.2 The range-and-bearing sensor model (slide 15)

The sensor is a **range-bearing sensor**: it tells you *how far* a landmark is (range $r$) and *in what direction relative to the robot's heading* (bearing $\beta$). A laser scanner that detects reflectors is a real example. The model is

$$
\mathbf{z} = \mathbf{h}(\mathbf{x}, \mathbf{p}_i) \quad\text{with}\quad \mathbf{x} = (x_v, y_v, \theta_v)^T,\ \mathbf{p}_i = (x_i, y_i)^T,
$$

$$
\mathbf{z} = \mathbf{h}(\mathbf{x}, \mathbf{p}_i) =
\begin{pmatrix}
\sqrt{(y_i - y_v)^2 + (x_i - x_v)^2} \\
\tan^{-1}\dfrac{y_i - y_v}{x_i - x_v} - \theta_v
\end{pmatrix}
+ \begin{pmatrix} w_r \\ w_\beta \end{pmatrix}.
$$

Reading it line by line:

- The first row is plain Pythagoras: the straight-line distance from the robot $(x_v, y_v)$ to the landmark $(x_i, y_i)$.
- The second row is the compass direction to the landmark (the $\tan^{-1}$, which in code is `atan2` so that it works in all four quadrants) **minus the robot's own heading** $\theta_v$. That subtraction is what makes it a *bearing relative to the robot* rather than an absolute direction.
- $\mathbf{w} = (w_r, w_\beta)^T \sim N(\mathbf{0}, \mathbf{W})$ is the sensor noise, with

$$
\mathbf{W} = \begin{pmatrix} \sigma_r^2 & 0 \\ 0 & \sigma_\beta^2 \end{pmatrix},
$$

again independent range and bearing errors.

The subscript $v$ stands for *vehicle*. The measurement vector is $\mathbf{z} = (r, \beta)^T$, so $p = 2$.

**Worked example.** Robot at $(x_v, y_v, \theta_v) = (1, 1, 0)$, landmark at $(4, 5)$.

1. Range: $r = \sqrt{(5-1)^2 + (4-1)^2} = \sqrt{16 + 9} = 5$ m.
2. Direction to the landmark: $\tan^{-1}(4/3) = 53.13^\circ$.
3. Bearing: $\beta = 53.13^\circ - 0^\circ = 53.13^\circ$. If the robot had been facing $\theta_v = 90^\circ$ instead, the bearing would be $53.13^\circ - 90^\circ = -36.87^\circ$, meaning "to my right".

### 7.3 The update equations (slides 15 and 16)

Because the sensor measures the *vehicle* state, the EKF update machinery from Section 5.3 applies directly, with the innovation computed from the known landmark position:

$$
\boldsymbol{\nu} = \mathbf{z}^{\#}\langle k+1\rangle - \mathbf{h}\big(\hat{\mathbf{x}}^+\langle k+1\rangle, \mathbf{p}_i\big),
$$

$$
\hat{\mathbf{x}}\langle k+1\rangle = \hat{\mathbf{x}}^+\langle k+1\rangle + \mathbf{K}\boldsymbol{\nu}, \qquad
\hat{\mathbf{P}}\langle k+1\rangle = \hat{\mathbf{P}}^+\langle k+1\rangle - \mathbf{K}\mathbf{H}_x\hat{\mathbf{P}}^+\langle k+1\rangle,
$$

$$
\mathbf{K} = \mathbf{P}^+\langle k+1\rangle\mathbf{H}_x^T\mathbf{S}^{-1}, \qquad
\mathbf{S} = \mathbf{H}_x\mathbf{P}^+\langle k+1\rangle\mathbf{H}_x^T + \mathbf{H}_w\hat{\mathbf{W}}\mathbf{H}_w^T.
$$

Here $\mathbf{z}^{\#}$ is the *actual* sensor reading (the $\#$ just marks "measured"), and $\mathbf{h}(\hat{\mathbf{x}}^+, \mathbf{p}_i)$ is *what we would expect to see* if the predicted pose were correct and the map were right. The innovation is the disagreement, and $\mathbf{K}$ distributes that disagreement onto the three states.

Slide 16 annotates each line in plain English, and it is worth repeating:

- $\hat{\mathbf{x}}^+ = \mathbf{f}(\hat{\mathbf{x}}, \hat{\mathbf{u}})$: *predict state one step ahead*.
- $\hat{\mathbf{P}}^+ = \mathbf{F}_x\hat{\mathbf{P}}\mathbf{F}_x^T + \mathbf{F}_v\mathbf{V}\mathbf{F}_v^T$: *project covariance one step ahead*.
- $\boldsymbol{\nu} = \mathbf{z}^{\#} - \mathbf{h}(\hat{\mathbf{x}}^+, \mathbf{p}_i)$: *new information: innovation*.
- $\mathbf{K} = \dots$: *how to distribute the innovation to the states: Kalman gain*.
- $\hat{\mathbf{x}} = \hat{\mathbf{x}}^+ + \mathbf{K}\boldsymbol{\nu}$: *state updated with innovation*.
- $\hat{\mathbf{P}} = \hat{\mathbf{P}}^+ - \mathbf{K}\mathbf{H}_x\hat{\mathbf{P}}^+$: *updated covariance*.

And the two phases chase each other in a loop: predict, update, predict, update.

### 7.4 The sensor Jacobians (slide 15, bottom)

Differentiate $\mathbf{h}$ with respect to the vehicle state $(x_v, y_v, \theta_v)$:

$$
\mathbf{H}_x = \left.\frac{\partial\mathbf{h}}{\partial\mathbf{x}}\right|_{\mathbf{w}=0} =
\begin{pmatrix}
-\dfrac{x_i - x_v}{r} & -\dfrac{y_i - y_v}{r} & 0 \\[2ex]
\dfrac{y_i - y_v}{r^2} & -\dfrac{x_i - x_v}{r^2} & -1
\end{pmatrix}.
$$

Let's make sense of each entry:

- **Top row (range).** Moving the robot toward the landmark shortens the range, hence the minus signs; the ratios $(x_i - x_v)/r$ and $(y_i - y_v)/r$ are just the cosine and sine of the direction to the landmark. The range does not depend on heading, so the last entry is 0.
- **Bottom row (bearing).** Moving the robot sideways changes the direction to the landmark, and the effect is larger when the landmark is close (notice the $r^2$ in the denominator: far landmarks barely change bearing when you step sideways). Turning the robot by a small angle changes the bearing by exactly minus that angle, hence the $-1$.

And with respect to the noise, since the noise is simply added,

$$
\mathbf{H}_w = \left.\frac{\partial\mathbf{h}}{\partial\mathbf{w}}\right|_{\mathbf{w}=0} = \begin{pmatrix} 1 & 0 \\ 0 & 1 \end{pmatrix}.
$$

**Worked example (continuing).** Robot $(1, 1, 0)$, landmark $(4, 5)$, $r = 5$:

$$
\mathbf{H}_x = \begin{pmatrix} -3/5 & -4/5 & 0 \\ 4/25 & -3/25 & -1 \end{pmatrix}
= \begin{pmatrix} -0.6 & -0.8 & 0 \\ 0.16 & -0.12 & -1 \end{pmatrix}.
$$

So if the robot's $x$ estimate were 10 cm too small, the predicted range would be $0.6 \times 0.1 = 6$ cm too long, and the predicted bearing would be $0.16 \times 0.1 = 0.016$ rad (about $0.9^\circ$) too large. These are exactly the numbers the filter needs to turn a range/bearing mismatch back into a pose correction.

### 7.5 How observations shrink uncertainty (slide 18)

Slide 18 shows the results of 1000 steps with a map of landmarks.

- **Top left:** the true (blue) and estimated (red) paths are now almost on top of each other, unlike slide 9.
- **Top right (zoomed):** the ellipses *grow* while the robot drives blind and *shrink* abruptly whenever a landmark is observed. You can see the estimate being yanked back onto the truth.
- **Bottom left:** $(\det\mathbf{P})^{0.5}$ versus time is a sawtooth: it rises during dead reckoning and drops sharply at each observation. Compare with slide 9, where it only ever rose.
- **Bottom right:** the errors in $x$, $y$ and $\theta$ stay inside the shaded $\pm$ confidence bounds, and the bar chart underneath shows *which* landmark was visible at each time. Gaps with no landmark line up with the peaks of uncertainty.

**Common mistake.** The uncertainty does not shrink to zero. The sensor itself is noisy ($\mathbf{W}$), so even a perfect map leaves some uncertainty. What the observations do is *bound* the uncertainty instead of letting it grow forever.

### 7.6 MATLAB (script section 6.2)

Slide 17 is a blank "MATLAB Time" placeholder; the commands are in the script:

```matlab
randinit
map = LandmarkMap(20, 10);        % 20 random landmarks in a 10x10 area
map.plot()
W = diag([0.1, 1*pi/180].^2);     % sensor noise: sigma_r = 10 cm, sigma_beta = 1 degree
sensor = RangeBearingSensor(veh, map, 'covar', W)   % a range-bearing sensor mounted on the vehicle, looking at the map
[z,i] = sensor.reading()          % one reading: z = (r, beta), i = which landmark it saw
map.landmark(17)                  % true coordinates of landmark 17

randinit
map = LandmarkMap(20);
veh = Bicycle('covar', V);
veh.add_driver( RandomPath(map.dim) );
sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', [-pi/2 pi/2], 'range', 4, 'animate');
                                  % realistic sensor: only sees landmarks within +/-90 deg and within 4 m
ekf = EKF(veh, V, P0, sensor, W, map);   % EKF with vehicle, odometry noise, initial P, sensor, sensor noise, and the KNOWN map
ekf.run(1000);
map.plot(); veh.plot_xy(); ekf.plot_xy('r'); ekf.plot_ellipse('k')
clf; ekf.plot_P(); xlabel('Time step'); ylabel('(det P)^{0.5}'); grid
clf; ekf.plot_error('confidence', 0.95, 'nplots', 4)   % errors in x, y, theta with 95% bounds
plot_poly([1:1000; sensor.landmarklog], 'fill', 'b'); ylabel('land marks'); grid   % which landmark was seen when
```

The crucial difference from Section 6.2 is the last two arguments of `EKF(veh, V, P0, sensor, W, map)`: now the filter has a sensor, knows the sensor noise, and is handed the map. The `'angle'` and `'range'` options make the sensor realistic, which is why there are gaps with no observations in slide 18.

---

## 8. Creating a map: the opposite problem

### 8.1 The setting (slide 19)

Now flip the problem on its head. Suppose the robot knows its pose *perfectly* (imagine a very good external positioning system) but has no map. Can it build one? The slide's assumptions and objectives are:

**Assumptions**

- Perfect vehicle pose.
- A sensor that detects features (the slide writes "$(x, y, \theta)$" here; see the flag below).
- There exist $N$ landmarks with **known identity**, meaning when the sensor sees a landmark it knows *which* landmark it is (the "data association" problem is assumed solved).

**Objectives**

- Estimate the position of the landmarks.
- Build the surrounding map while translating (moving).

⚠️ Slide 19 lists "Sensor to detect features $(x, y, \theta)$" as an assumption. The sensor used throughout is the range-bearing sensor that returns $(r, \beta)$, so the $(x, y, \theta)$ most likely refers to the vehicle pose that is assumed known. Check with your lecturer if the exact wording matters.

### 8.2 What are the states? (slide 20)

The question in the red box on slide 20 is "What are the states to be estimated?" The answer: the thing we are unsure about is now the **landmark positions**, so they *are* the state:

$$
\hat{\mathbf{x}} = (x_1, y_1, x_2, y_2, \dots, x_M, y_M)^T \in \mathbb{R}^{2M\times1}, \qquad \hat{\mathbf{P}} \in \mathbb{R}^{2M\times 2M}.
$$

Here $M$ is the number of landmarks seen *so far*. The slide asks "The size $M$ is varying – why?" and answers: **$M$ starts at 0 and is incremented every time a previously unseen landmark is observed.** The state vector grows as the robot discovers the world.

### 8.3 Prediction is trivial (slide 20)

Landmarks do not move. So the prediction step is

$$
\hat{\mathbf{x}}^+\langle k+1\rangle = \hat{\mathbf{x}}\langle k\rangle, \qquad \hat{\mathbf{P}}^+\langle k+1\rangle = \hat{\mathbf{P}}\langle k\rangle,
$$

described on the slide as **"static, since the landmarks are stationary"**. Nothing changes, no noise is added. (Equivalently $\mathbf{F}_x = \mathbf{I}$ and $\mathbf{F}_v = \mathbf{0}$.)

### 8.4 Where is a newly seen landmark? The inverse sensor model (slide 20)

When the sensor reports a landmark it has *never seen before*, we need its initial position estimate. We have the known robot pose $(x_v, y_v, \theta_v)$ and the reading $(r, \beta)$, so we just go $r$ metres in direction $\theta_v + \beta$:

$$
\mathbf{g}(\mathbf{x}, \mathbf{z}) = \begin{pmatrix} x_v + r\cos(\theta_v + \beta) \\ y_v + r\sin(\theta_v + \beta) \end{pmatrix}.
$$

The slide notes that **$\mathbf{g}(\cdot)$ is the inverse of $\mathbf{h}(\cdot)$**: $\mathbf{h}$ turns a landmark position into a range/bearing; $\mathbf{g}$ turns a range/bearing back into a landmark position.

**Worked example (checking the inverse).** Robot at $(1, 1, 0)$ observes $r = 5$, $\beta = 53.13^\circ$ (the reading we computed in Section 7.2). Then $x = 1 + 5\cos 53.13^\circ = 1 + 5 \times 0.6 = 4$ and $y = 1 + 5\sin 53.13^\circ = 1 + 5 \times 0.8 = 5$. We recover the landmark at $(4, 5)$, as we should.

### 8.5 Extending the state vector and covariance (slide 20)

Adding the new landmark means appending two numbers to the state:

$$
\mathbf{x}\langle k\rangle' = \mathbf{y}\big(\mathbf{x}\langle k\rangle, \mathbf{z}\langle k\rangle, \mathbf{x}_v\langle k\rangle\big) =
\begin{pmatrix} \mathbf{x}\langle k\rangle \\ \mathbf{g}\big(\mathbf{x}_v\langle k\rangle, \mathbf{z}\langle k\rangle\big) \end{pmatrix}.
$$

The prime means "after insertion". The old state stays as it is, and the new landmark's $(x, y)$ from $\mathbf{g}$ is stacked underneath.

The covariance has to grow too, and the new landmark's uncertainty comes from the sensor noise $\hat{\mathbf{W}}$ pushed through $\mathbf{g}$:

$$
\hat{\mathbf{P}}\langle k\rangle' = \mathbf{Y}_z \begin{pmatrix} \hat{\mathbf{P}}\langle k\rangle & \mathbf{0} \\ \mathbf{0} & \hat{\mathbf{W}} \end{pmatrix} \mathbf{Y}_z^T,
\qquad
\mathbf{Y}_z = \frac{\partial\mathbf{y}}{\partial\mathbf{z}} = \begin{pmatrix} \mathbf{I}_{n\times n} & \mathbf{0}_{n\times2} \\ \mathbf{G}_x & \mathbf{0}_{2\times n-3} & \mathbf{G}_z \end{pmatrix},
$$

with the **insertion Jacobians**

$$
\mathbf{G}_x = \frac{\partial\mathbf{g}}{\partial\mathbf{x}} = \begin{pmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \end{pmatrix}, \qquad
\mathbf{G}_z = \frac{\partial\mathbf{g}}{\partial\mathbf{z}} = \begin{pmatrix} \cos(\theta_v + \beta) & -r\sin(\theta_v + \beta) \\ \sin(\theta_v + \beta) & r\cos(\theta_v + \beta) \end{pmatrix}.
$$

What is going on here, in plain words:

- The block matrix $\begin{pmatrix}\hat{\mathbf{P}} & 0 \\ 0 & \hat{\mathbf{W}}\end{pmatrix}$ holds the uncertainty of everything we had *plus* the uncertainty of the new sensor reading, uncorrelated.
- $\mathbf{Y}_z$ is the Jacobian of the insertion function $\mathbf{y}$. Its top block is the identity (old states are untouched). Its bottom block says how the new landmark depends on the vehicle pose ($\mathbf{G}_x$) and on the measurement ($\mathbf{G}_z$).
- $\mathbf{G}_z$ converts range/bearing noise into $x$/$y$ noise: a range error moves the landmark *along* the line of sight (the cosine/sine column), a bearing error moves it *across* the line of sight by $r$ times the angle error (the second column, which grows with $r$: a $1^\circ$ bearing error at 10 m is a 17 cm position error).
- $\mathbf{G}_x = \mathbf{0}$ **because the vehicle pose is assumed perfect** in the pure mapping problem: there is no pose uncertainty to pass on to the landmark.

**Worked example.** Robot $(1,1,0)$, reading $r=5$, $\beta = 53.13^\circ$:
$\mathbf{G}_z = \begin{pmatrix} 0.6 & -4 \\ 0.8 & 3 \end{pmatrix}$. With $\hat{\mathbf{W}} = \mathrm{diag}(0.1^2, (1^\circ)^2) = \mathrm{diag}(0.01, 3.05\times10^{-4})$, the new landmark's covariance block is $\mathbf{G}_z\hat{\mathbf{W}}\mathbf{G}_z^T$, whose diagonal works out to about $(0.0036 + 0.0049,\ 0.0064 + 0.0027) = (0.0085, 0.0091)$, i.e. a standard deviation of roughly 9 cm in each direction. Most of that came from the bearing error multiplied by the 5 m range.

⚠️ The $\mathbf{Y}_z$ matrix on slide 20 is written in the general SLAM layout (it includes a $\mathbf{G}_x$ block and a $\mathbf{0}_{2\times n-3}$ block that assume a 3-element vehicle pose sits at the front of the state). In the pure mapping case the pose is *not* in the state, and $\mathbf{G}_x = \mathbf{0}$ anyway, so the whole bottom-left of $\mathbf{Y}_z$ is zero and the new landmark simply gets covariance $\mathbf{G}_z\hat{\mathbf{W}}\mathbf{G}_z^T$ with no correlation to the others. The slide reuses the SLAM-form matrix; the result is the same.

### 8.6 The observation Jacobian for mapping (slide 21)

For the *update* step, when a landmark that is already in the map is observed again, we need $\mathbf{H}_x$, the Jacobian of $\mathbf{h}$ with respect to the state. But now the state is the landmark positions, not the pose. Differentiating $\mathbf{h}$ with respect to the landmark position $\mathbf{p}_i = (x_i, y_i)$ gives

$$
\mathbf{H}_{p_i} = \frac{\partial\mathbf{h}}{\partial\mathbf{p}_i} =
\begin{pmatrix}
\dfrac{x_i - x_v}{r} & \dfrac{y_i - y_v}{r} \\[2ex]
-\dfrac{y_i - y_v}{r^2} & \dfrac{x_i - x_v}{r^2}
\end{pmatrix},
$$

which the slide describes as *"how the landmark observation changes with respect to the landmark position for a particular robot pose."* Notice it is the first two columns of the localization $\mathbf{H}_x$ from Section 7.4 with the signs flipped: moving the *landmark* toward the robot has the opposite effect of moving the *robot* toward the landmark.

For the whole state vector, only landmark $i$ affects this observation, so

$$
\mathbf{H}_x = \left.\frac{\partial\mathbf{h}}{\partial\mathbf{x}}\right|_{\mathbf{w}=0} = \big(\mathbf{0} \cdots \mathbf{H}_{p_i} \cdots \mathbf{0}\big) \in \mathbb{R}^{2\times 2M}.
$$

The slide: *"Now the observation depends only on the position of the landmark, thus most entries are zeroed."* The $2\times2$ block $\mathbf{H}_{p_i}$ sits in the columns belonging to landmark $i$ and everything else is zero.

### 8.7 MATLAB and the big lesson (slide 23, script section 6.3)

```matlab
randinit
map = LandmarkMap(20);
veh = Bicycle();                   % error-free vehicle: NO 'covar', so the pose is perfect
veh.add_driver( RandomPath(map.dim) );
W = diag([0.1, 1*pi/180].^2);
sensor = RangeBearingSensor(veh, map, 'covar', W, 'animate');
ekf = EKF(veh, [], [], sensor, W, []);   % [] for V and P0: no pose uncertainty; [] for map: the map is UNKNOWN and must be built
ekf.run(1000);
map.plot();                        % true landmarks
ekf.plot_map('g');                 % estimated landmarks with their 95% ellipses
veh.plot_xy('b');
ekf.landmarks(:,6)                 % bookkeeping for landmark 6: where in the state vector it lives, how many times seen
ekf.x_est(19:20)'                  % the estimated (x,y) of the landmark stored at state indices 19-20
ekf.P_est(19:20,19:20)             % its 2x2 covariance block
```

The right plot of slide 23 shows the estimated landmarks (small ellipses) sitting on the true ones. The left plot is the **covariance matrix shown as an image**: it is **block-diagonal**, a chain of $2\times2$ red squares with white (zero) everywhere else. That picture is the boxed statement at the bottom:

> **Observing one landmark provides no new information about any other landmark.**

Why? Because the pose is perfect, each landmark's estimate depends only on *its own* measurements. There is no shared source of error to link them. Hold on to this picture, because in SLAM it will change dramatically.

---

## 9. SLAM: localization and mapping at the same time

### 9.1 The chicken and the egg (slides 24 and 25)

In real life the robot has *neither* a map *nor* a perfect pose. Slide 24 shows the dilemma with a picture of a chicken and an egg:

> A map is needed for localization. A pose estimate is needed for mapping.

Each problem needs the solution of the other. The answer is to solve them **simultaneously**: estimate the robot's pose *and* all the landmark positions in one big filter. This is **Simultaneous Localization and Mapping (SLAM)**. Slide 25 says it is *"considered a fundamental problem for truly autonomous robots"* and *"the basis for most of the autonomous navigation systems"*, illustrated by a dark mine tunnel, where there is no GPS and no prior map, so SLAM is the only option.

The slide prompts you to "think real life, think localization techniques, think limitations". Some things to think about: in a mine or indoors GPS does not work; wheels slip on gravel; the environment may look the same everywhere (long featureless tunnels), which makes landmarks hard to tell apart; and dust or darkness can blind cameras.

**Everyday analogy.** You are dropped in an unfamiliar town with no map. You start walking, noting landmarks: "church on my left, about 50 m". As you move you keep a sketch map and keep track of your own position on it. When you see the church again from a different street you both *fix your own position* ("I must be here, because the church is there") and *refine the church's position on your sketch*. That is SLAM.

### 9.2 The joint state (slide 26)

Stack the vehicle pose and all landmark positions into one state vector:

$$
\hat{\mathbf{x}} = (x_v, y_v, \theta_v, x_1, y_1, x_2, y_2, \dots, x_M, y_M)^T \in \mathbb{R}^{(2M+3)\times1}.
$$

Its covariance is a $(2M+3)\times(2M+3)$ matrix with a very meaningful block structure:

$$
\hat{\mathbf{P}} = \begin{pmatrix} \hat{\mathbf{P}}_{vv} & \hat{\mathbf{P}}_{vm} \\ \hat{\mathbf{P}}_{vm}^T & \hat{\mathbf{P}}_{mm} \end{pmatrix}.
$$

- $\hat{\mathbf{P}}_{vv}$ ($3\times3$): uncertainty of the **vehicle pose**.
- $\hat{\mathbf{P}}_{mm}$ ($2M\times2M$): uncertainty of the **landmark positions**.
- $\hat{\mathbf{P}}_{vm}$ ($3\times2M$): the **correlation between the vehicle and the map**. This block is the heart of SLAM. It exists because every landmark was first placed using an uncertain robot pose, so errors in the pose and errors in the map are tied together.

### 9.3 The SLAM equations (slide 26)

They are the ones you already know, just applied to the bigger state.

**Prediction:**

$$
\hat{\mathbf{x}}^+\langle k+1\rangle = \mathbf{f}\big(\hat{\mathbf{x}}\langle k\rangle, \mathbf{u}\langle k\rangle\big), \qquad
\hat{\mathbf{P}}^+\langle k+1\rangle = \mathbf{F}_x\hat{\mathbf{P}}\langle k\rangle\mathbf{F}_x^T + \mathbf{F}_v\hat{\mathbf{V}}\mathbf{F}_v^T.
$$

Only the first three states (the pose) actually move; the landmarks are static. So $\mathbf{F}_x$ is the $3\times3$ odometry Jacobian from Section 6.1 in the top-left corner and identity everywhere else, and $\mathbf{F}_v$ has the $3\times2$ odometry noise Jacobian on top and zeros below.

**Sensor-based update:**

$$
\hat{\mathbf{x}}\langle k+1\rangle = \hat{\mathbf{x}}^+\langle k+1\rangle + \mathbf{K}\boldsymbol{\nu}, \qquad
\hat{\mathbf{P}}\langle k+1\rangle = \hat{\mathbf{P}}^+\langle k+1\rangle - \mathbf{K}\mathbf{H}_x\hat{\mathbf{P}}^+\langle k+1\rangle,
$$

$$
\mathbf{K} = \mathbf{P}^+\langle k+1\rangle\mathbf{H}_x^T\mathbf{S}^{-1}, \qquad
\mathbf{S} = \mathbf{H}_x\mathbf{P}^+\langle k+1\rangle\mathbf{H}_x^T + \mathbf{H}_w\hat{\mathbf{W}}\mathbf{H}_w^T,
$$

with the observation Jacobian now containing *both* the pose part and the landmark part:

$$
\mathbf{H}_x = \left.\frac{\partial\mathbf{h}}{\partial\mathbf{x}}\right|_{\mathbf{w}=0} = \big(\mathbf{H}_{x_v} \cdots \mathbf{0} \cdots \mathbf{H}_{p_i} \cdots \mathbf{0}\big) \in \mathbb{R}^{2\times(2M+3)}.
$$

$\mathbf{H}_{x_v}$ is the $2\times3$ block from Section 7.4 (how the observation changes with the *pose*), $\mathbf{H}_{p_i}$ is the $2\times2$ block from Section 8.6 (how it changes with *landmark $i$'s position*), placed in landmark $i$'s columns, and zeros everywhere else. Because $\mathbf{K} = \mathbf{P}^+\mathbf{H}_x^T\mathbf{S}^{-1}$ multiplies by the *full* covariance (including $\hat{\mathbf{P}}_{vm}$), one observation of landmark $i$ ends up correcting the pose, landmark $i$, **and every other landmark correlated with the pose**. That is how SLAM spreads information around the map.

**State vector extension** (a new landmark appears):

$$
\mathbf{x}\langle k\rangle' = \mathbf{y}\big(\mathbf{x}\langle k\rangle, \mathbf{z}\langle k\rangle, \mathbf{x}_v\langle k\rangle\big) = \begin{pmatrix} \mathbf{x}\langle k\rangle \\ \mathbf{g}\big(\mathbf{x}_v\langle k\rangle, \mathbf{z}\langle k\rangle\big) \end{pmatrix}.
$$

**Covariance extension:**

$$
\hat{\mathbf{P}}\langle k\rangle' = \mathbf{Y}_z \begin{pmatrix} \hat{\mathbf{P}}\langle k\rangle & \mathbf{0} \\ \mathbf{0} & \hat{\mathbf{W}} \end{pmatrix}\mathbf{Y}_z^T, \qquad
\mathbf{Y}_z = \frac{\partial\mathbf{y}}{\partial\mathbf{z}} = \begin{pmatrix} \mathbf{I}_{n\times n} & \mathbf{0}_{n\times2} \\ \mathbf{G}_x & \mathbf{0}_{2\times n-3} & \mathbf{G}_z \end{pmatrix},
$$

$$
\mathbf{G}_z = \begin{pmatrix} \cos(\theta_v + \beta) & -r\sin(\theta_v + \beta) \\ \sin(\theta_v + \beta) & r\cos(\theta_v + \beta) \end{pmatrix}, \qquad
\mathbf{G}_x = \begin{pmatrix} 1 & 0 & -r\sin(\theta_v + \beta) \\ 0 & 1 & r\cos(\theta_v + \beta) \end{pmatrix}.
$$

Compare $\mathbf{G}_x$ with Section 8.5: there it was all zeros, **here it is not**. $\mathbf{G}_x$ is the derivative of the new landmark's position with respect to the vehicle pose. It is non-zero because the pose is now uncertain, and that uncertainty must be *inherited* by the new landmark: if the robot's $x$ is off by 10 cm, the landmark it just placed is off by 10 cm too (the 1s), and if its heading is off by a small angle the landmark is off by $r$ times that angle sideways (the third column). This is exactly what creates the $\hat{\mathbf{P}}_{vm}$ correlations. The $\mathbf{0}_{2\times n-3}$ block sits under the *other* landmarks: the new landmark is not directly linked to them at insertion time (though it becomes linked indirectly, through the pose, in later updates).

### 9.4 MATLAB and what the result looks like (slide 28, script section 6.4)

```matlab
randinit
P0 = diag([.01, .01, 0.005].^2);  % small initial pose uncertainty
map = LandmarkMap(20);
veh = Bicycle('covar', V);        % noisy odometry
veh.add_driver( RandomPath(map.dim) );
sensor = RangeBearingSensor(veh, map, 'covar', W, 'animate');
ekf = EKF(veh, V, P0, sensor, W, []);   % V and P0 given (pose uncertain), map [] (unknown): this is SLAM
ekf.run(1000);
map.plot(); ekf.plot_map('g'); ekf.plot_xy('r'); veh.plot_xy('b');
```

Compare the three `EKF(...)` calls you have now seen; the pattern is the whole lecture in one line:

| Problem | Call | What is known / unknown |
|---|---|---|
| Dead reckoning | `EKF(veh, V, P0)` | no sensor, no map; pose estimated from odometry only |
| Localization | `EKF(veh, V, P0, sensor, W, map)` | map known; pose unknown |
| Mapping | `EKF(veh, [], [], sensor, W, [])` | pose known (no V, P0); map unknown |
| SLAM | `EKF(veh, V, P0, sensor, W, [])` | pose *and* map unknown |

Slide 28 shows the outcome. The red estimated path hugs the blue truth, and the landmark estimates (green ellipses) sit on the true ones. Two small plots deserve attention:

- **Bottom left:** $(\det\mathbf{P})^{0.5}$ on a log scale falls steeply over time. The combined uncertainty of pose plus map keeps decreasing as more observations come in.
- **Bottom right:** the covariance image is now **densely filled**, red everywhere, not block-diagonal like slide 23. Every state is correlated with every other. The robot's pose error was baked into each landmark when it was first seen, and the landmarks are all tied together through the pose. Observing *any* landmark now improves *all* of them.

⚠️ The $(\det\mathbf{P})^{0.5}$ plot on slide 28 drops to around $10^{-70}$. Part of that is genuine improvement, but part is simply that the determinant of a larger matrix multiplies more small numbers together, so the raw value is not directly comparable to the plots on slides 9 and 18. Treat the *downward trend* as the message, not the absolute number.

**Common mistake.** People see "SLAM" and assume the robot ends up with a perfect map. It does not. The map is good *relative to itself* and relative to where the robot started, but if the first few metres of odometry were wrong the whole map is shifted and rotated. Nothing in this formulation ties the map to absolute world coordinates.

---

## 10. Pose-graph SLAM: a different way to think about it (slides 29 and 30)

The EKF approach keeps one giant state vector and covariance that grows with every landmark, which becomes expensive for big maps. **Pose-graph SLAM** is an alternative, and it is what most modern systems use.

### 10.1 The structure (slide 29)

- **Sensors** (odometry, laser scanner, camera) feed a **front end**.
- The front end builds a **pose graph**: each **node** $\xi_1, \xi_2, \xi_3, \xi_4, \dots$ is a robot pose at some moment, and each **edge** $z_{12}, z_{23}, z_{34}, \dots$ is a **constraint**: a measurement of the *relative pose* between two nodes (for example, "from odometry, pose 2 is 1.2 m ahead and 5° left of pose 1").
- The dashed edge $z_{41}$ is special: it connects pose 4 back to pose 1. That is a **loop closure**: the robot has recognised that it is back at a place it saw before (for example, by matching the current laser scan to an old one). Loop closures are what make pose-graph SLAM powerful, because they tie the accumulated drift of a long journey back to a known point.
- A **back end** takes the graph and finds the set of node poses that best agrees with *all* the edges at once, an **optimization** problem. In plain words, the edges are like springs of measured length; the back end lets the whole structure relax until the total strain is minimal.

**Everyday analogy.** Sketch a route from memory as a chain of "then I walked 50 m, then turned left...". By the end the chain has drifted. Then you realise the final corner is actually the *same corner you started at*. You rubber-band the whole sketch so that the start and end meet, spreading the correction along every link. That rubber-banding is graph optimization.

### 10.2 MATLAB (slide 30, script section 6.5)

```matlab
syms x_i y_i theta_i x_j y_j theta_j x_m y_m theta_m assume real
xi_e = inv( SE2(x_m, y_m, theta_m) ) * inv( SE2(x_i, y_i, theta_i) ) * SE2(x_j, y_j, theta_j);
fk = simplify(xi_e.xyt);           % the error between the measured relative pose (m) and the one implied by nodes i and j
jacobian( fk, [x_i y_i theta_i] ); Ai = simplify(ans)   % how that error changes when node i moves: needed by the optimizer

pg = PoseGraph('pg1.g2o'); pg.plot(); pg.optimize('animate')     % tiny toy graph

pg = PoseGraph('killian-small.toro');   % real data: MIT Killian Court
pg.plot()
pg.optimize('animate')
pg.plot()
```

The first block shows, symbolically, what an edge's **error** is: take the measured relative pose $(x_m, y_m, \theta_m)$ and compare it with the relative pose implied by the current guesses of nodes $i$ and $j$. If they agree, the error is zero. The Jacobian $\mathbf{A}_i$ is how that error changes when node $i$ is nudged, which is what the optimizer needs to know which way to nudge.

Slide 30 shows the Killian Court dataset before and after. **(a)** Before optimization, the raw odometry graph is a tangled mess, with the building's corridors twisted and misaligned and a dense fan of loop-closure edges. **(b)** After `optimize`, the corridors are straight, the rectangles close, and the map looks like an actual building. Nothing new was measured; the optimizer just found the poses that are most consistent with all the constraints.

---

## 11. Laser odometry with ICP (slide 31, script section 6.8)

### 11.1 The idea

Wheel odometry slips. A laser scanner gives you a "snapshot" of the walls around you, hundreds of points at known angles. If you take two scans a moment apart, the walls have not moved, so the apparent shift of the points tells you how *the robot* moved. Estimating motion this way is called **laser odometry**, and the standard algorithm for lining up two point sets is **Iterated Closest Point (ICP)**.

ICP, in plain words:

1. Start with a guess of the transform between the two scans (for example, from wheel odometry, or zero).
2. For every point in scan A, find the **closest point** in scan B.
3. Compute the rigid transform (rotation plus translation) that best lines up all those pairs.
4. Apply it, and **iterate** steps 2 and 3 until the points stop moving.

The "closest point" pairing is wrong at first (points get matched to the wrong wall), but each iteration makes the alignment slightly better, which makes the pairings slightly more correct, and it converges. Slide 31 shows this: **(a)** two scans (red and blue) slightly offset; **(b)** after ICP they lie on top of each other.

### 11.2 MATLAB

```matlab
pg = PoseGraph('killian.g2o', 'laser');   % pose graph that also stores the laser scan at each node
[r, theta] = pg.scan(2580);        % scan number 2580 as range r and angle theta vectors
about r theta
polar(theta, r)                    % polar plot of the scan
[x,y] = pol2cart(theta, r); plot(x, y, '.')   % same scan in Cartesian coordinates

p2580 = pg.scanxy(2580);           % scan as a 2xN matrix of (x,y) points
p2581 = pg.scanxy(2581);           % the next scan
about p2580
T = icp( p2581, p2580, 'verbose', 'T0', transl2(0.5, 0), 'distthresh', 3)
                                   % the SE(2) transform that maps scan 2581 onto scan 2580, i.e. how the robot moved
pg.time(2581) - pg.time(2580)      % time between the scans, so you could get a velocity
```

The options: `'T0'` is the initial guess (half a metre forward here), `'distthresh'` ignores point pairs further apart than 3 m (they are almost certainly wrong matches), `'verbose'` prints each iteration.

The slide also gives a 3D sanity check: make 20 random 3D points `p1 = randn(3,20)`, transform them by a known pose `T = transl(1,2,3)*eul2tr(0.1,0.2,0.3)` to get `p2 = homtrans(T, p1)`, and run `icp(p1, p2)`. ICP should recover `T`. Because every point has a true partner, this is the ideal case; real scans have partial overlap and noise.

---

## 12. From laser scans to an occupancy grid (slide 32)

Once you know the pose at which every laser scan was taken (from the optimized pose graph), you can paint all the scan points into a single **occupancy grid**: a picture of the world divided into cells, with each cell marked *free* (white), *occupied* (black) or *unknown* (grey). You met occupancy grids in Lecture 7 as the input to path planners like D* and PRM; this slide is where they come from.

```matlab
map = pg.scanmap();                % fuse all scans at their optimized poses into one occupancy grid
pg.plot_occgrid(map)
```

Slide 32 **(a)** shows the whole Killian Court building as white corridors on grey; **(b)** zooms into one junction, where the black cells are the walls hit by the laser and the white cells are the free space the laser passed through. This map is exactly what a planner needs to find routes.

---

## 13. Real-world examples (slides 33 to 35)

- **Slide 33, "Creating a Map":** a video of the lab arena (plywood walls, a cardboard box obstacle) alongside the RViz display of the laser point cloud being built as the robot drives. This is the lab version of Section 12.
- **Slide 34, "Mine based 3D reconstruction":** the paper *Evaluation of Localization and 3D Reconstruction in Underground Mining* by Kanellakis and Nikolakopoulos. Underground there is no GPS, so SLAM-type techniques are essential; this is the "think real life" prompt from slide 25 made concrete.
- **Slide 35, "Aerial 3D reconstruction":** *Cooperative UAVs as a tool for Aerial Inspection of large scale Aging Infrastructure* (the AEROWORKS project), with drones mapping a wind turbine. Same ideas, different platform.

These slides are context rather than examinable theory, but they tell you *why* the maths matters.

---

## In a nutshell

A robot can never know its pose exactly, so it keeps a best guess together with a covariance matrix that honestly describes the uncertainty, and pictures it as an ellipse. Dead reckoning predicts the new pose from odometry, but every step adds noise and nothing removes it, so the ellipse grows forever. The Kalman filter fixes this by alternating a prediction step, which grows the uncertainty, with an update step that uses the innovation (measured minus expected sensor value) and the Kalman gain to pull the estimate toward the sensor and shrink the uncertainty; for nonlinear robot models the Extended Kalman Filter does the same after linearizing with Jacobians at every step. With a known map and a range-bearing sensor the EKF localizes the robot; with a known pose it builds a map of landmarks (and the landmarks stay uncorrelated); with neither it does SLAM, where pose and landmarks share one state vector and become fully correlated so that each observation improves everything. Pose-graph SLAM reformulates the problem as poses linked by relative-pose constraints, including loop closures, and an optimizer straightens the whole graph; laser scans can supply those constraints through ICP matching and can then be painted into an occupancy grid for planning.

---

## Key terms

- **Localization:** working out where the robot is (its pose) in some reference frame.
- **Pose:** position plus orientation; in 2D, $(x, y, \theta)$.
- **Probability density function (PDF):** a curve whose *volume over a region* gives the probability of being in that region; its height alone is not a probability.
- **Gaussian / normal distribution:** the bell-shaped PDF fully described by a mean and a (co)variance.
- **Zero-mean noise:** random error that is equally likely positive or negative and averages to zero.
- **Covariance matrix ($\mathbf{P}$, $\Sigma$):** matrix of variances (diagonal) and covariances (off-diagonal) describing the spread and correlation of several uncertain quantities.
- **Uncertainty ellipse:** the contour of a 2D Gaussian; its size and tilt visualise the covariance.
- **Dead reckoning:** estimating pose by integrating odometry (distance and heading changes) from a known start; error grows without bound.
- **Odometry:** measurement of how far and how much the robot has moved, typically from wheel encoders, giving $(\delta_d, \delta_\theta)$.
- **Process noise ($\mathbf{v}$, covariance $\mathbf{V}$):** randomness in how the state evolves (e.g. wheel slip).
- **Sensor (measurement) noise ($\mathbf{w}$, covariance $\mathbf{W}$):** randomness in what the sensor reports.
- **State estimate ($\hat{\mathbf{x}}$):** the filter's best guess of the state; the hat means "estimate".
- **Prediction step:** push the estimate and covariance forward using the motion model; uncertainty grows.
- **Update step:** correct the prediction using a sensor measurement; uncertainty shrinks.
- **Innovation ($\boldsymbol{\nu}$):** measured sensor value minus the value predicted from the state estimate; the "new information".
- **Innovation covariance ($\mathbf{S}$):** expected scatter of the innovation, from state uncertainty and sensor noise.
- **Kalman gain ($\mathbf{K}$):** matrix deciding how much of the innovation to apply to each state; large when the sensor is more trustworthy than the prediction.
- **Kalman filter:** the optimal (minimum mean-square error) estimator for linear systems with zero-mean Gaussian noise.
- **Extended Kalman Filter (EKF):** Kalman filter applied to nonlinear systems by linearizing around the current estimate with Jacobians at every step.
- **Jacobian:** matrix of partial derivatives saying how each output changes with each input; $\mathbf{F}_x, \mathbf{F}_v, \mathbf{H}_x, \mathbf{H}_w$ in this lecture.
- **Linearization point:** the estimate around which the tangent approximation is taken; the EKF equations are only valid near it.
- **Landmark:** a distinguishable fixed feature with (known or estimated) map coordinates $\mathbf{p}_i = (x_i, y_i)$.
- **Range-bearing sensor:** sensor returning distance $r$ and relative angle $\beta$ to a landmark.
- **Inverse sensor model ($\mathbf{g}$):** converts a range-bearing reading plus robot pose into a landmark position; the inverse of $\mathbf{h}$.
- **State vector extension:** appending a newly seen landmark's coordinates to the state vector.
- **Insertion Jacobians ($\mathbf{G}_x, \mathbf{G}_z$, $\mathbf{Y}_z$):** derivatives used to give a new landmark the correct initial covariance and correlations.
- **SLAM (Simultaneous Localization and Mapping):** estimating the robot pose and the map in one joint state; solves the chicken-and-egg problem.
- **Pose graph:** graph whose nodes are robot poses and whose edges are measured relative-pose constraints.
- **Loop closure:** a constraint linking the current pose to a much earlier one when a place is recognised again; corrects accumulated drift.
- **Front end / back end:** the part of pose-graph SLAM that builds the graph from sensors / the optimizer that finds the most consistent poses.
- **Laser odometry:** estimating motion by aligning consecutive laser scans.
- **ICP (Iterated Closest Point):** algorithm that aligns two point clouds by repeatedly pairing closest points and solving for the best rigid transform.
- **Occupancy grid:** a cell map marking each cell free, occupied or unknown; built from laser scans at known poses and used for path planning.

---

## Check yourself

1. The prediction step of a Kalman filter always *adds* to the covariance and the update step always *subtracts* from it. Explain in one or two sentences why that makes sense physically, and what it implies for a robot that drives for a long time without seeing any landmark.

2. A robot at pose $(2, 3, 90^\circ)$ reads odometry $\delta_d = 1$ m, $\delta_\theta = -20^\circ$. What is its dead-reckoned pose at the next step? Then write down $\mathbf{F}_x$ for this step and say in words what its third column represents.

3. You believe a 1D position is $\hat{x}^+ = 10$ with variance $P^+ = 1$. A sensor with noise variance $W = 4$ reports $z = 12$. Compute the Kalman gain, the updated estimate and the updated variance. Why did the estimate move only a little?

4. In the mapping experiment (perfect pose) the covariance image is block-diagonal, but in the SLAM experiment it is densely filled. Explain the cause of the difference and name the specific Jacobian whose value changes between the two cases.

5. What is a loop closure in pose-graph SLAM, and why does it matter more than any single odometry edge? What does the `optimize` method do with it?

<details>
<summary>Answers</summary>

**1.** Prediction uses only the motion model, which is driven by noisy odometry, so moving forward in time can only add uncertainty (you add $\mathbf{F}_v\hat{\mathbf{V}}\mathbf{F}_v^T$ and never remove anything). An update brings in independent outside information from a sensor, which can only make you more certain (you subtract $\mathbf{K}\mathbf{H}_x\hat{\mathbf{P}}^+$). A robot that drives a long time without landmark observations is doing pure dead reckoning, so its covariance, and its uncertainty ellipse, grows without bound, as on slide 9.

**2.** $\cos 90^\circ = 0$, $\sin 90^\circ = 1$, so $x = 2 + 1\times0 = 2$, $y = 3 + 1\times1 = 4$, $\theta = 90^\circ - 20^\circ = 70^\circ$. New pose $(2, 4, 70^\circ)$. The Jacobian is
$\mathbf{F}_x = \begin{pmatrix} 1 & 0 & -\delta_d\sin\theta \\ 0 & 1 & \delta_d\cos\theta \\ 0 & 0 & 1\end{pmatrix} = \begin{pmatrix} 1 & 0 & -1 \\ 0 & 1 & 0 \\ 0 & 0 & 1\end{pmatrix}$ (using the old heading $90^\circ$). The third column says how an error in heading turns into an error in position after this step: here a small heading error of $\epsilon$ rad would shift $x$ by $-\epsilon$ m (the robot is driving along $+y$, so heading error pushes it sideways in $x$) and would not affect $y$ to first order.

**3.** $S = HP^+H^T + W = 1 + 4 = 5$; $K = P^+H^T/S = 1/5 = 0.2$; innovation $\nu = 12 - 10 = 2$; $\hat{x} = 10 + 0.2\times2 = 10.4$; $P = P^+ - KHP^+ = 1 - 0.2\times1 = 0.8$. The estimate moved only 20 % of the way toward the sensor because the sensor (variance 4) is much *less* reliable than the prediction (variance 1), so the gain is small. The variance still shrank, because any independent information helps a bit.

**4.** With a perfectly known pose, each landmark's estimate depends only on its own measurements; there is no shared source of error, so the cross-covariances are zero and $\hat{\mathbf{P}}$ is block-diagonal ("observing one landmark provides no new information about any other"). In SLAM the pose is uncertain, and every new landmark inherits that pose uncertainty when it is inserted, which creates the vehicle-to-map correlations $\hat{\mathbf{P}}_{vm}$; later updates then couple the landmarks to each other through the pose. The Jacobian that changes is $\mathbf{G}_x = \partial\mathbf{g}/\partial\mathbf{x}$, which is all zeros for mapping with a known pose but equals $\begin{pmatrix}1 & 0 & -r\sin(\theta_v+\beta)\\ 0 & 1 & r\cos(\theta_v+\beta)\end{pmatrix}$ in SLAM.

**5.** A loop closure is an edge connecting the current pose node to a much earlier node, created when the robot recognises a previously visited place (for example by matching laser scans). Ordinary odometry edges only link consecutive poses, so their small errors accumulate into drift along the whole chain; a loop closure provides an independent constraint that spans the chain and reveals the total drift. The `optimize` method adjusts all node poses to minimise the total disagreement with every edge, so the drift revealed by the loop closure is spread back along the chain and the map straightens out, as on slide 30.

</details>
