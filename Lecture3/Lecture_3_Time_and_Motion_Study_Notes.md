# Lecture 3 – Time and Motion
### Study notes (R7010E Robotics, LTU)

> **A note on the numbering.** The slide deck itself is titled "Lecture 2 – Time and Motion", but the folder is called Lecture3 and the course plan from Lecture 1 lists "Time and Motion" as lecture number 3 (Lecture 2 was the continuation of Position and Orientation). These notes follow the folder and the course plan and call it Lecture 3. The content is the same either way.

## What this lecture is about, and why it matters

In Lecture 1 you learned how to *describe* where a robot is: a position, an orientation, a pose. This lecture adds the missing ingredient, **time**. It asks two questions. First, if a body is moving, how do its velocities, forces and torques look from different coordinate frames? Second, if you want a robot to go from pose A to pose B, how do you design the *motion in between* so that it is smooth and physically sensible, rather than a sudden jump? Everything in Lab 1 (drawing letters with a robot arm, flying a drone through a 3D map, driving a ground robot around obstacles) is built on exactly these ideas, so this is one of the most practical lectures of the course.

## Main topics in this lecture

1. Motion versus path planning (where this lecture sits in the big picture)
2. Rotation matrices as "axis and angle", and skew-symmetric matrices
3. Angular velocity, and how to express it in a different frame
4. The time derivative of a rotation matrix, $\dot{R} = S(\omega)R$, and incremental rotation
5. Spatial velocity (linear + angular in one 6-vector) and how to transform it between frames (velocity transform, adjoint)
6. Dynamics of a moving body: Newton's second law, Euler's rotational equation, the inertia matrix
7. Wrenches (force + torque in one 6-vector) and how to transform them
8. Smooth 1D trajectories: what "smooth" means, quintic polynomials, `tpoly`
9. Trapezoidal (linear segment with parabolic blends) trajectories, `lspb`
10. Multi-dimensional trajectories, `mtraj`
11. Multi-segment trajectories through via points, `mstraj`
12. Interpolating orientation in 3D (angles versus quaternions, and why you never interpolate rotation matrices directly)
13. Interpolating full poses, `interp` and `ctraj`
14. How all this connects to Lab 1

Let's take them one by one.

---

## 1. Motion and path planning: the big picture

The first slides show a humanoid picking up a cup, two arms carrying a box together, a drone flying under a bridge, and a cartoon robot drawing a wandering route from A to B. They all share one idea: a robot does not teleport. It has to move through a continuous sequence of poses over time.

There are two related but different jobs here, and it helps to keep them apart from the start:

- **Path planning** answers "*which route* should I take from A to B, avoiding obstacles?" That is a geometric question, and it is the subject of Lecture 7.
- **Trajectory generation** (this lecture) answers "*how do I move along a route as a function of time*, so that velocities and accelerations are smooth and the robot's motors can actually follow it?"

**Everyday picture.** Path planning is choosing the roads on a map. Trajectory generation is deciding how you press the accelerator and brake so the ride is smooth and you arrive on time. You need both.

A common mix-up: a **path** is just a sequence of positions with no timing information. A **trajectory** is a path *with a time stamp on every point*. When you see the word trajectory in an exam question, think "position as a function of time".

---

## 2. Rotation matrices, axis-angle form, and skew-symmetric matrices

### 2.1 Start simple: one rotation axis and one angle

Any 3D rotation can be described as "turn by some angle $\theta$ about some axis $\hat{\omega}$". The slide writes the rotation matrix of frame {B} relative to frame {A}, at time $t$, as

$$
{}^A\mathbf{R}_B(t) = e^{\left[{}^A\hat{\omega}(t)\right]_\times \theta(t)} \in SO(3)
$$

In words: the rotation matrix equals the *matrix exponential* of (the skew-symmetric matrix built from the axis) times (the angle). Let's unpack each symbol:

- ${}^A\hat{\omega}(t)$ is the **rotation axis**, a unit vector (length 1), written in the coordinates of frame {A}. The hat means "unit vector".
- $\theta(t)$ is the **rotation angle** about that axis, in radians.
- $[\cdot]_\times$ is the operation that turns a 3-vector into a **skew-symmetric matrix** (explained next).
- $e^{(\dots)}$ is the **matrix exponential**. You do not need to compute it by hand. Just know that `expm` in MATLAB does it, and that it turns "axis and angle" into a proper rotation matrix.
- $SO(3)$ is the name of the set of all valid 3D rotation matrices (orthonormal, determinant +1). Think of it as the club that every legitimate rotation matrix belongs to.

You saw the 2D version of this in Lecture 1's MATLAB file: `R = rot2(0.3)`, then `S = logm(R)` gives a tiny skew-symmetric matrix containing 0.3, and `expm(S)` brings back `R`. The 3D story is the same, just with a 3-component axis.

### 2.2 What is a skew-symmetric matrix?

A **skew-symmetric matrix** is a square matrix whose transpose equals its negative: $S^T = -S$. That forces the diagonal to be zero (because each diagonal entry must equal its own negative) and forces the entries above the diagonal to be the negatives of the ones below.

For a 3-vector $\omega = (\omega_1, \omega_2, \omega_3)$ the skew-symmetric matrix is

$$
S(\omega) = [\omega]_\times =
\begin{pmatrix}
0 & -\omega_3 & \omega_2 \\
\omega_3 & 0 & -\omega_1 \\
-\omega_2 & \omega_1 & 0
\end{pmatrix} \in so(3)
$$

(The lowercase $so(3)$ is just the name for the set of such 3×3 skew-symmetric matrices.)

**Why would anyone want this?** Because of one very useful fact: multiplying $S(\omega)$ by a vector gives the cross product,

$$
S(\omega)\,\mathbf{v} = \omega \times \mathbf{v}.
$$

So the skew-symmetric matrix is simply "the cross product with $\omega$, written as a matrix". That lets you put cross products inside ordinary matrix algebra, which is exactly what we need when we start differentiating rotation matrices.

**Worked example.** Take $\omega = (1, 2, 3)$. Then

$$
S(\omega) = \begin{pmatrix} 0 & -3 & 2 \\ 3 & 0 & -1 \\ -2 & 1 & 0 \end{pmatrix}.
$$

Check the transpose: swapping rows and columns gives $\begin{pmatrix} 0 & 3 & -2 \\ -3 & 0 & 1 \\ 2 & -1 & 0 \end{pmatrix}$, which is exactly $-S(\omega)$. Good.

Now check the cross-product claim with $\mathbf{v} = (1, 0, 0)$. By hand, $\omega \times \mathbf{v} = (2\cdot 0 - 3\cdot 0,\; 3\cdot 1 - 1\cdot 0,\; 1\cdot 0 - 2\cdot 1) = (0, 3, -2)$. With the matrix: first row $0\cdot1 + (-3)\cdot 0 + 2\cdot 0 = 0$, second row $3\cdot 1 = 3$, third row $-2\cdot 1 = -2$. Same answer, $(0, 3, -2)$.

In the toolbox, `skew([1 2 3])` builds this matrix and `vex(S)` does the reverse (pulls the 3-vector back out).

**Common mistake.** People confuse *symmetric* ($S^T = S$) with *skew-symmetric* ($S^T = -S$). The inertia matrix later in this lecture is symmetric. The angular velocity matrix is skew-symmetric. They are opposites in this sense, so do not mix them up.

---

## 3. Angular velocity, and changing the frame it is written in

### 3.1 What angular velocity is

**Angular velocity** $\omega = (\omega_x, \omega_y, \omega_z)$ describes how fast a body is spinning, and about which axis, at one instant of time.

- Its **direction** is the instantaneous axis of rotation.
- Its **magnitude** $\|\omega\|$ is the rate of rotation about that axis, in radians per second.

**Everyday picture.** Imagine a spinning top. The stick through the middle points along the axis of rotation. How fast it spins is the magnitude. Both pieces of information are packed into one vector.

### 3.2 Writing the same angular velocity in another frame

A spinning body has one physical angular velocity, but you can write its three numbers in different coordinate frames. If you know it in the body frame {B} and want it in the world frame {A}, you just rotate the vector:

$$
{}^A\omega = {}^A\mathbf{R}_B\; {}^B\omega
$$

Here ${}^B\omega$ is the rate of rotation written in {B}, ${}^A\omega$ is the same rate of rotation written in {A}, and ${}^A\mathbf{R}_B$ is the rotation matrix of {B} relative to {A} that you learned in Lecture 1. Because a rotation does not change length, $\|{}^A\omega\| = \|{}^B\omega\|$: the spin rate is the same, only the description changes.

**Worked example.** Suppose the body frame {B} is rotated 90° about the world z-axis, so ${}^A\mathbf{R}_B = \mathbf{R}_z(90°) = \begin{pmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{pmatrix}$, and the body spins about its own x-axis at 1 rad/s, so ${}^B\omega = (1, 0, 0)$. Then

$$
{}^A\omega = \begin{pmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{pmatrix}\begin{pmatrix} 1 \\ 0 \\ 0 \end{pmatrix} = \begin{pmatrix} 0 \\ 1 \\ 0 \end{pmatrix}.
$$

That makes sense: the body's x-axis, after a 90° turn about z, points along the world y-axis, so spinning about body-x is the same as spinning about world-y.

---

## 4. How a rotation matrix changes with time

### 4.1 The key formula

If a frame is rotating, its rotation matrix $\mathbf{R}(t)$ changes over time. The slide gives the derivative:

$$
\dot{\mathbf{R}}(t) = S(\omega)\,\mathbf{R}(t)
$$

In words: the rate of change of the rotation matrix equals the skew-symmetric matrix of the angular velocity, times the current rotation matrix. This is where the skew-symmetric matrix earns its keep. It converts "the body is spinning at $\omega$" into "here is how every entry of $\mathbf{R}$ is changing".

### 4.2 Incremental rotation: what $\dot{R}$ actually means

A derivative is a limit of a difference. The slide approximates it over a small time step $\delta_t$:

$$
\dot{\mathbf{R}} \approx \frac{\mathbf{R}(t + \delta_t) - \mathbf{R}(t)}{\delta_t}
$$

Rearranging gives the rotation a moment later:

$$
\mathbf{R}(t + \delta_t) \approx \delta_t\,\dot{\mathbf{R}} + \mathbf{R}(t)
$$

Substituting $\dot{\mathbf{R}} = S(\omega)\mathbf{R}$:

$$
\mathbf{R}(t + \delta_t) \approx \delta_t S(\omega)\mathbf{R}(t) + \mathbf{R}(t) = \big(\delta_t S(\omega) + \mathbf{I}_{3\times 3}\big)\mathbf{R}(t)
$$

The slide's conclusion: this describes how the orthonormal rotation matrix changes as a function of angular velocity. The practical message is that a **small rotation** is approximately "identity plus a small skew-symmetric matrix". That is what the lecture's MATLAB line demonstrates:

```matlab
rotx(0.001)
```

You get a matrix extremely close to $\mathbf{I} + S([0.001, 0, 0])$, that is, ones on the diagonal and $\pm 0.001$ in the lower-right block.

**Worked example.** Let the body spin about z at $\omega = (0, 0, 1)$ rad/s and take $\delta_t = 0.001$ s. Then $\delta_t S(\omega) = \begin{pmatrix} 0 & -0.001 & 0 \\ 0.001 & 0 & 0 \\ 0 & 0 & 0 \end{pmatrix}$ and

$$
\mathbf{I} + \delta_t S(\omega) = \begin{pmatrix} 1 & -0.001 & 0 \\ 0.001 & 1 & 0 \\ 0 & 0 & 1 \end{pmatrix}.
$$

The exact answer is $\mathbf{R}_z(0.001)$, whose cosine entries are $\cos(0.001) = 0.9999995$ and whose sine entries are $0.0009999998$. The approximation is right to about one part in a million. For larger angles the approximation degrades, because $\mathbf{I} + \delta_t S(\omega)$ is no longer exactly orthonormal. That is why real code uses the exact exponential (or a unit quaternion update) over many small steps instead.

**Common mistake.** $\dot{\mathbf{R}}$ is *not* itself a rotation matrix. It is the rate of change of one. Do not try to "apply" $\dot{\mathbf{R}}$ to a point.

---

## 5. Spatial velocity: linear and angular velocity in one vector

### 5.1 The definition

A rigid body moving through space has both a **linear velocity** $\mathbf{v}$ (how fast its reference point is translating, in m/s) and an **angular velocity** $\omega$ (how fast it is spinning, in rad/s). The slides stack them into one 6-vector called the **spatial velocity** (also called a *twist*):

$$
{}^A\nu_B = \big({}^A\mathbf{v}_B,\; {}^A\omega_B\big) \in \mathbb{R}^6
$$

This is the spatial velocity of frame {B} with respect to frame {A}, with both parts written in {A}'s coordinates. The first three numbers are linear velocity, the last three are angular velocity.

The slide lists two assumptions behind using a single spatial velocity for a whole body:

- Every point in the body has the same angular velocity, and we take one representative linear velocity.
- We place {B} at the body's centre of mass.

Under those assumptions, one 6-vector fully describes the instantaneous motion of a rigid body.

**Everyday picture.** Think of a frisbee in flight. It is travelling forward (linear velocity) and spinning (angular velocity) at the same time. Six numbers describe its motion completely.

### 5.2 Transforming spatial velocity between frames: the simple case

The velocity of a moving body can be expressed in the world frame {A} or in the body frame {B}. The slide says they are linearly related by a matrix called the **Jacobian** or **interaction matrix**:

$$
{}^A\nu = \begin{pmatrix} {}^A\mathbf{R}_B & \mathbf{0}_{3\times 3} \\ \mathbf{0}_{3\times 3} & {}^A\mathbf{R}_B \end{pmatrix} {}^B\nu = {}^A\mathbf{J}_B({}^A\xi_B)\,{}^B\nu
$$

where ${}^A\xi_B \sim ({}^A\mathbf{R}_B, {}^A\mathbf{t}_B)$ is the relative pose from Lecture 1.

What this says in words: when you only want to re-describe the *same point's* velocity in a different frame's coordinates, you rotate the linear part and rotate the angular part, separately, with the same rotation matrix. The translation ${}^A\mathbf{t}_B$ plays no role here because we are not moving to a different point on the body, we are only changing which axes we measure along.

In the toolbox this block-diagonal matrix is produced by `velxform`:

```matlab
TB = SE3(1, 2, 0) * SE3.Rz(pi/2)   % pose of body frame {B}: translate (1,2,0), rotate 90° about z
vb = [0.2 0.3 0 0 0 0.5]'          % spatial velocity in {B}: v=(0.2,0.3,0), omega=(0,0,0.5)
va = TB.velxform * vb              % the same velocity written in the world frame {A}
```

**Worked example (the numbers from the lecture file).** ${}^A\mathbf{R}_B = \mathbf{R}_z(90°)$. Rotating the linear part: $\mathbf{R}_z(90°)(0.2, 0.3, 0) = (-0.3, 0.2, 0)$. Rotating the angular part: $\mathbf{R}_z(90°)(0, 0, 0.5) = (0, 0, 0.5)$, unchanged because it is along the z-axis. So `va = [-0.3 0.2 0 0 0 0.5]'`. The body is moving "forward-and-left" in its own frame; seen from the world, which is turned by 90°, that looks like "left-and-forward".

### 5.3 The general case: a second frame {C} on the same body

Now suppose frame {C} is attached to the *same* moving body but at a different place, like a camera mounted on the nose of a drone while {B} is at the centre of mass. The velocity of point C is not just a rotated copy of the velocity of point B. Because the body is spinning, point C sweeps around B, and that adds extra linear velocity.

The slide gives the full transform:

$$
{}^C\nu = \begin{pmatrix} {}^C\mathbf{R}_B & [{}^C\mathbf{t}_B]_\times\,{}^C\mathbf{R}_B \\ \mathbf{0}_{3\times 3} & {}^C\mathbf{R}_B \end{pmatrix} {}^B\nu = \mathrm{Ad}({}^C\xi_B)\,{}^B\nu
$$

The 6×6 matrix is called the **adjoint** of the pose, written $\mathrm{Ad}(\cdot)$. Compared with the simple case, there is one new block in the top right: $[{}^C\mathbf{t}_B]_\times {}^C\mathbf{R}_B$. That block takes the angular velocity, crosses it with the offset between the two frames, and adds the result to the linear velocity. This is just the familiar rule "velocity of a point = velocity of the reference + $\omega \times$ (offset)", packaged as a matrix.

In the toolbox:

```matlab
TBC = SE3(0, 0.4, 0)      % frame {C} is 0.4 m along B's y-axis, same orientation
TCB = inv(TBC)            % pose of {B} relative to {C}
vc  = TCB.Ad * vb         % spatial velocity of the body, expressed at point C
```

**Worked example.** ${}^C\mathbf{R}_B = \mathbf{I}$ (same orientation), ${}^C\mathbf{t}_B = (0, -0.4, 0)$ (B is 0.4 m in the negative y direction when seen from C). From before, $\mathbf{v}_B = (0.2, 0.3, 0)$ and $\omega = (0, 0, 0.5)$.

Step 1: the extra term is ${}^C\mathbf{t}_B \times \omega = (0, -0.4, 0) \times (0, 0, 0.5) = (-0.4\cdot 0.5 - 0,\; 0 - 0,\; 0) = (-0.2, 0, 0)$.

Step 2: linear velocity at C is $\mathbf{v}_B + (\text{extra}) = (0.2 - 0.2,\; 0.3,\; 0) = (0, 0.3, 0)$.

Step 3: angular velocity is the same for every point of a rigid body, so it stays $(0, 0, 0.5)$.

Result: `vc = [0 0.3 0 0 0 0.5]'`. Sanity check with intuition: the body spins counter-clockwise about z at 0.5 rad/s, and C sits 0.4 m to the "left" (positive y) of B. A counter-clockwise spin drags a point on the left *backwards* in x, at speed $0.5 \times 0.4 = 0.2$ m/s. That exactly cancels B's forward speed of 0.2. 

**Common mistake.** Using `velxform` (rotation only) when the two frames are at different points on the body. If there is an offset *and* the body is rotating, you need the full adjoint `Ad`, otherwise you lose the $\omega \times \mathbf{t}$ term.

---

## 6. Dynamics of a moving body

Up to now we talked about *how* a body moves (kinematics). Dynamics asks *why*: what forces and torques produce that motion.

### 6.1 Newton's second law (translation)

$$
m\,{}^0\ddot{\mathbf{x}} = {}^0\mathbf{f}
$$

In words: mass times the acceleration of the centre of mass (written in the world frame {0}) equals the total force acting on the body (also in the world frame). You know this one. **Quick example:** a 2 kg drone whose rotors push up with 20.6 N while gravity pulls down with 19.6 N feels a net 1 N upward, so it accelerates upward at $1/2 = 0.5\ \text{m/s}^2$.

### 6.2 Euler's equation of motion (rotation)

$$
{}^B\mathbf{J}\,{}^B\dot{\omega} + {}^B\omega \times \big({}^B\mathbf{J}\,{}^B\omega\big) = {}^B\tau
$$

This is the rotational cousin of Newton's law, and it is written in the **body frame {B}**. Each symbol:

- ${}^B\tau$ is the **torque** (also called moment) applied to the body, a 3-vector, in N·m.
- ${}^B\mathbf{J}$ is the **rotational inertia matrix**, a 3×3 positive-definite matrix (explained below). It plays the role that mass plays in Newton's law: it measures how hard the body is to spin up.
- ${}^B\dot{\omega}$ is the angular acceleration.
- The term $\omega \times (\mathbf{J}\omega)$ is the **gyroscopic term**. It has no counterpart in Newton's law. It says that even with zero torque, a spinning body can change its spin axis if its inertia is not the same in all directions. That is why a thrown book tumbles.

**Everyday picture.** Newton: a heavy shopping trolley needs more push to speed up. Euler: a long ladder needs more twist to spin than a basketball of the same mass, and *which way* you try to spin it matters.

### 6.3 The inertia matrix

$$
\mathbf{J} = \begin{pmatrix} I_{xx} & I_{xy} & I_{xz} \\ I_{yx} & I_{yy} & I_{yz} \\ I_{zx} & I_{zy} & I_{zz} \end{pmatrix}
$$

- The diagonal entries $I_{xx}, I_{yy}, I_{zz}$ are called **moments of inertia**. Each one says how hard the body is to spin about that axis.
- The off-diagonal entries $I_{ij}$ (with $i \ne j$) are called **products of inertia**. They describe coupling: a non-zero $I_{xy}$ means spinning about x tends to also produce angular momentum about y. For symmetric, well-balanced bodies they are often zero.
- The matrix is symmetric ($I_{xy} = I_{yx}$) and positive-definite (loosely: all "spin stiffnesses" are positive).

### 6.4 Simulating a tumbling body (the lecture's MATLAB loop)

The lecture file simulates a body spinning with **no torque applied**, $\tau = 0$. Rearranging Euler's equation with $\tau = 0$ gives $\dot{\omega} = -\mathbf{J}^{-1}\big(\omega \times \mathbf{J}\omega\big)$, and the code integrates that forward in small steps:

```matlab
J = [2 -1 0; -1 4 0; 0 0 3];   % inertia matrix (note the product of inertia -1 between x and y)
attitude = UnitQuaternion();    % start with no rotation
w = 0.2*[1 2 2]';               % initial angular velocity (rad/s)
dt = 0.05;                      % time step (s)
h = attitude.plot();
for t = 0:dt:10
   wd = -inv(J) * (cross(w, J*w));                  % angular acceleration from Euler's equation
   w  = w + wd*dt;                                  % rectangular (Euler) integration of omega
   attitude = attitude .* UnitQuaternion.omega(wd*dt); % update the orientation (see warning below)
   attitude.plot('handle', h); pause(dt)
end
```

Line by line: `cross(w, J*w)` is the gyroscopic term; multiplying by `-inv(J)` solves for $\dot{\omega}$; `w + wd*dt` is the simplest possible numerical integration ("rectangular integration": new value = old value + rate × step); `UnitQuaternion.omega(...)` builds the small rotation that happened during one step and `.*` composes it onto the current attitude. Using a unit quaternion for the attitude keeps the orientation valid, which is exactly the problem we saw with $\mathbf{I} + \delta_t S(\omega)$ in Section 4.

⚠️ **Probable typo in the lecture file.** The orientation update uses `UnitQuaternion.omega(wd*dt)`, that is, angular *acceleration* times $\delta_t$. To advance the orientation you need the angular *velocity* times $\delta_t$, so the line should almost certainly read `UnitQuaternion.omega(w*dt)`, which is also what the textbook has (Corke, section 3.2.1). The simulation still runs with the typo, but the animated frame will not tumble correctly. Check with your lecturer, and use `w*dt` in your own code.

**Worked example (one step of the loop by hand).** With $\mathbf{J}$ as above and $\omega = (0.2, 0.4, 0.4)$:

1. $\mathbf{J}\omega = (2\cdot 0.2 - 1\cdot 0.4,\; -1\cdot 0.2 + 4\cdot 0.4,\; 3\cdot 0.4) = (0, 1.4, 1.2)$.
2. $\omega \times \mathbf{J}\omega = (0.4\cdot 1.2 - 0.4\cdot 1.4,\; 0.4\cdot 0 - 0.2\cdot 1.2,\; 0.2\cdot 1.4 - 0.4\cdot 0) = (-0.08, -0.24, 0.28)$.
3. $\dot{\omega} = -\mathbf{J}^{-1}(-0.08, -0.24, 0.28) = \mathbf{J}^{-1}(0.08, 0.24, -0.28)$. The upper-left 2×2 block of $\mathbf{J}$ inverts to $\frac{1}{7}\begin{pmatrix} 4 & 1 \\ 1 & 2 \end{pmatrix}$ and the z entry inverts to $1/3$, giving $\dot{\omega} = (0.08, 0.08, -0.093)$.
4. After one step of $\delta_t = 0.05$ s: $\omega_{\text{new}} = (0.204, 0.404, 0.395)$.

So even though no torque is applied, the spin vector is drifting. That drift is the gyroscopic effect in action.

---

## 7. Wrenches: force and torque in one vector

### 7.1 Definition

Just as we packed linear and angular velocity into a 6-vector, we pack force and torque into a **wrench**:

$$
\mathbf{W} = (f_x, f_y, f_z, m_x, m_y, m_z) \in \mathbb{R}^6
$$

The first three entries are the force (N), the last three the moment or torque (N·m). A wrench describes the complete "push and twist" acting on a body at one point.

### 7.2 Transforming a wrench to another frame

If you know the wrench at frame {B} and want it at frame {C} on the same body:

$$
{}^C\mathbf{W} = \begin{pmatrix} {}^B\mathbf{R}_C & [{}^B\mathbf{t}_C]_\times {}^B\mathbf{R}_C \\ \mathbf{0}_{3\times 3} & {}^B\mathbf{R}_C \end{pmatrix}^{T} {}^B\mathbf{W} = \mathrm{Ad}({}^B\xi_C)^{T}\,{}^B\mathbf{W}
$$

Notice two things compared with the velocity transform in Section 5.3. First, it is the **transpose** of the adjoint. Second, the pose inside is ${}^B\xi_C$ (C relative to B), the *opposite direction* from the velocity case, which used ${}^C\xi_B$. This is a standard "duality" between velocities and forces: they transform in opposite ways so that power, which is wrench dotted with velocity, stays the same no matter which frame you compute it in.

In MATLAB:

```matlab
WB = [3 4 0 0 0 0]'      % wrench at {B}: force (3,4,0) N, zero moment
WC = TBC.Ad' * WB        % the same wrench expressed at {C}
```

(⚠️ Small slip in the lecture file's comment: it calls the 3 and 4 "Nm". They are the force components, so the unit is N, not N·m.)

**Worked example.** `TBC = SE3(0, 0.4, 0)`, so ${}^B\mathbf{R}_C = \mathbf{I}$ and ${}^B\mathbf{t}_C = (0, 0.4, 0)$. Because the transpose moves the coupling block to the bottom-left, the force part stays $(3, 4, 0)$ and the moment part becomes $-\,{}^B\mathbf{t}_C \times \mathbf{f}$. Compute $(0, 0.4, 0) \times (3, 4, 0) = (0.4\cdot 0 - 0\cdot 4,\; 0\cdot 3 - 0\cdot 0,\; 0\cdot 4 - 0.4\cdot 3) = (0, 0, -1.2)$, so the moment at C is $(0, 0, 1.2)$ N·m.

Does that make physical sense? A force of 3 N in x applied at B, which sits 0.4 m in the negative y direction from C, gives a lever-arm torque of $0.4 \times 3 = 1.2$ N·m about C's z-axis. The 4 N in y acts along the line joining B and C, so it contributes no torque. Yes, it checks out.

**Common mistake.** Applying `Ad` (no transpose) to a wrench. Velocities use `Ad`, wrenches use `Ad'`. If your torques come out wrong by a sign or by a missing lever-arm term, this is the first thing to check.

---

## 8. Smooth 1D trajectories

Now we switch from describing motion to *designing* it. Start with the simplest case: one quantity $s(t)$ (a single joint angle, or a single coordinate) that must go from a start value to an end value over a time $T$.

### 8.1 What does "smooth" mean?

The slide defines it through derivatives. A trajectory is smooth if

- position $x(t)$,
- velocity $\dot{x}(t)$,
- acceleration $\ddot{x}(t)$,
- jerk $\dddot{x}(t)$ (the rate of change of acceleration),
- and so on

are all **continuous**, meaning no sudden jumps. In practice we usually care most about position, velocity and acceleration.

**Why it matters.** A jump in velocity would need infinite acceleration, and so infinite force. A jump in acceleration means a sudden jerk in force, which rattles the robot, excites vibrations and can damage gears. Real motors cannot deliver either, so a trajectory with jumps simply cannot be followed.

**Everyday picture.** A good lift (elevator) starts gently, speeds up, cruises, slows gently, stops. A bad one jolts you at every change. The jolts are discontinuities in acceleration.

### 8.2 Candidate smooth functions

The slide shows three shapes that are smooth everywhere: a **sinusoid**, a **Gaussian** (bell curve), and a **polynomial**. Robotics most often uses polynomials, because you can tune their coefficients to hit exact start and end conditions.

### 8.3 The quintic polynomial

A **quintic** polynomial is a 5th-order polynomial. It has 6 coefficients:

$$
s(t) = At^5 + Bt^4 + Ct^3 + Dt^2 + Et + F
$$

Differentiating gives velocity and acceleration:

$$
\dot{s}(t) = 5At^4 + 4Bt^3 + 3Ct^2 + 2Dt + E
$$
$$
\ddot{s}(t) = 20At^3 + 12Bt^2 + 6Ct + 2D
$$

Why 5th order? Because 6 coefficients let you impose **6 boundary conditions**: position, velocity and acceleration at the start ($t = 0$) and at the end ($t = T$). The slide's table:

| Time | $s$ | $\dot{s}$ | $\ddot{s}$ |
|---|---|---|---|
| $t = 0$ | $s_0$ | $\dot{s}_0$ | $\ddot{s}_0$ |
| $t = T$ | $s_T$ | $\dot{s}_T$ | $\ddot{s}_T$ |

⚠️ Slide 18 has a few typos: it writes $\ddot{s}(0) = 2d$ (should be $2D$), and under "when $t = T$" it labels the velocity and acceleration lines as $\dot{s}(0)$ and $\ddot{s}(0)$ when they should be $\dot{s}(T)$ and $\ddot{s}(T)$. The matrix below is correct, so follow that.

### 8.4 Setting up the equations, step by step

**Step 1. Plug in $t = 0$.** Every term with a $t$ vanishes:

$s(0) = F$, $\dot{s}(0) = E$, $\ddot{s}(0) = 2D$.

So three of the six coefficients are read straight off the start conditions.

**Step 2. Plug in $t = T$.**

$s(T) = AT^5 + BT^4 + CT^3 + DT^2 + ET + F$
$\dot{s}(T) = 5AT^4 + 4BT^3 + 3CT^2 + 2DT + E$
$\ddot{s}(T) = 20AT^3 + 12BT^2 + 6CT + 2D$

**Step 3. Write all six as one matrix equation.** Unknowns on the right, known boundary values on the left:

$$
\begin{pmatrix} s_0 \\ s_T \\ \dot{s}_0 \\ \dot{s}_T \\ \ddot{s}_0 \\ \ddot{s}_T \end{pmatrix}
=
\begin{pmatrix}
0 & 0 & 0 & 0 & 0 & 1 \\
T^5 & T^4 & T^3 & T^2 & T & 1 \\
0 & 0 & 0 & 0 & 1 & 0 \\
5T^4 & 4T^3 & 3T^2 & 2T & 1 & 0 \\
0 & 0 & 0 & 2 & 0 & 0 \\
20T^3 & 12T^2 & 6T & 2 & 0 & 0
\end{pmatrix}
\begin{pmatrix} A \\ B \\ C \\ D \\ E \\ F \end{pmatrix}
$$

**Step 4. Solve** the 6×6 linear system (in MATLAB, `coeffs = M \ b`) to get $A$ to $F$. That is all `tpoly` does internally.

### 8.5 Worked example

Move from $s_0 = 0$ to $s_T = 1$ in $T = 1$ s, starting and ending at rest with zero acceleration (all four of $\dot{s}_0, \dot{s}_T, \ddot{s}_0, \ddot{s}_T$ are zero).

From Step 1: $F = 0$, $E = 0$, $D = 0$.

From Step 2 with $T = 1$ and the remaining unknowns:
- $A + B + C = 1$
- $5A + 4B + 3C = 0$
- $20A + 12B + 6C = 0 \;\Rightarrow\; 10A + 6B + 3C = 0$

Subtract the second from the third: $5A + 2B = 0 \Rightarrow B = -2.5A$. Put into the second: $5A - 10A + 3C = 0 \Rightarrow C = \tfrac{5}{3}A$. Put both into the first: $A - 2.5A + \tfrac{5}{3}A = 1 \Rightarrow \tfrac{1}{6}A = 1 \Rightarrow A = 6$. Then $B = -15$, $C = 10$.

$$
s(t) = 6t^5 - 15t^4 + 10t^3
$$

Check: $s(1) = 6 - 15 + 10 = 1$. Velocity $\dot{s}(t) = 30t^4 - 60t^3 + 30t^2 = 30t^2(t-1)^2$, which is zero at both ends and never negative, so there is no overshoot. The peak velocity is at $t = 0.5$: $30 \cdot 0.25 \cdot 0.25 = 1.875$. The *average* velocity is distance over time, $1/1 = 1$. So the peak is almost twice the average, which is a known drawback of the quintic: to finish on time it has to rush in the middle.

### 8.6 `tpoly` in MATLAB

```matlab
tpoly(0, 1, 50);                 % plot a quintic from 0 to 1 over 50 time steps (position, velocity, acceleration)
[s, sd, sdd] = tpoly(0, 1, 50);  % return position s, velocity sd, acceleration sdd as vectors
tpoly(0, 1, 50, 0.5, 0);         % same, but with initial velocity 0.5 and final velocity 0
mean(sd) / max(sd)               % ratio of average to peak velocity, about 0.52
```

The last line quantifies the "rushing in the middle" problem: the average speed is only about half the peak. If your motor's top speed is the limit, a quintic uses it for only a brief moment.

With a non-zero initial velocity (`tpoly(0,1,50,0.5,0)`) the curve may **overshoot** the target and come back. That is not a bug, it is simply the only 5th-order curve that satisfies all six conditions. Lab 1's optional Exercise 7 asks you to explore exactly this.

---

## 9. The trapezoidal profile: `lspb`

### 9.1 The idea

To fix the quintic's low average/peak ratio, use a **linear segment with parabolic blends**, abbreviated **LSPB**, also called a **trapezoidal velocity profile**:

1. Accelerate with constant acceleration (position follows a parabola, velocity rises linearly).
2. Cruise at constant velocity $V$ (position follows a straight line, velocity is flat).
3. Decelerate with constant deceleration (another parabola, velocity falls linearly).

The velocity plot looks like a trapezoid, hence the name. Position is smooth and velocity is continuous, but **acceleration jumps** at the two blend boundaries (from $+a$ to $0$ and from $0$ to $-a$). So LSPB is smoother than a bang-bang motion but less smooth than a quintic. In exchange, the robot spends most of its time at top speed.

**Everyday picture.** A train: accelerate out of the station, cruise, brake into the next station. Nobody drives a train with a quintic.

### 9.2 `lspb` in MATLAB

```matlab
lspb(0, 1, 50);                    % plot a trapezoidal trajectory from 0 to 1 in 50 steps
[s, sd, sdd] = lspb(0, 1, 50);     % get position, velocity, acceleration
max(sd)                            % the automatically chosen cruise velocity, about 0.031
s = lspb(0, 1, 50, 0.025);         % force the cruise velocity to 0.025
s = lspb(0, 1, 50, 0.035);         % force the cruise velocity to 0.035
```

### 9.3 Why the cruise velocity cannot be arbitrary

The lecture file points out that as you raise $V$, the cruise segment gets shorter, and eventually its duration hits zero. Beyond that the trajectory is impossible and the function returns an error. There is also a lower limit. Here is the reasoning, step by step, for a move of distance $d$ in time $T$:

1. The two blends together cover distance $V \cdot t_b$ (average speed $V/2$ over total blend time $2t_b$).
2. The cruise covers $V (T - 2t_b)$.
3. Total: $VT - Vt_b = d$, so $t_b = T - d/V$.
4. We need $t_b \ge 0$, which gives $V \ge d/T$. In words: the cruise speed must be at least the average speed, otherwise you cannot cover the distance in time.
5. We need $t_b \le T/2$ (the two blends cannot overlap), which gives $V \le 2d/T$. In words: at most twice the average speed, at which point the profile becomes a triangle with no cruise at all.

For the example $d = 1$ over about 50 steps, the window is roughly $0.02 < V \le 0.04$, which is why 0.025 and 0.035 both work. The toolbox's default picks $V = 1.5 \times d/T \approx 0.03$, right in the middle. Lab 1's Exercise 8 asks you to find these bounds yourself.

**Common mistake.** Thinking "a higher cruise velocity always finishes faster". With `lspb` the total time $T$ is fixed by you. Changing $V$ only reshapes the profile within that time.

---

## 10. Multi-dimensional trajectories: `mtraj`

### 10.1 Configuration vectors

Real robots have more than one coordinate. The slide lists **configuration vectors** $\mathbf{q}$ for several cases:

| Robot / quantity | $\mathbf{q}$ |
|---|---|
| 3-joint manipulator | $(q_1, q_2, q_3)$ |
| 2D position | $(x, y)$ |
| 2D position and heading | $(x, y, \theta)$ |
| 3D orientation | $(\theta_r, \theta_p, \theta_y)$ roll, pitch, yaw |
| 6D pose | $(x, y, z, \theta_r, \theta_p, \theta_y)$ |

In every case we want smooth motion from an initial configuration vector to a final one.

### 10.2 The trick: one scalar profile, applied to every axis

The simplest approach is to compute one scalar profile $s(t)$ that goes from 0 to 1 (quintic or LSPB), and then set each coordinate to

$$
q_i(t) = q_{i,0} + s(t)\,(q_{i,T} - q_{i,0}).
$$

Because all axes share the same $s(t)$, they all start together and arrive together, and the path in configuration space is a straight line. That is what `mtraj` does:

```matlab
q = mtraj(@lspb, [0 2], [1 -1], 50);   % two axes: from (0,2) to (1,-1) in 50 steps, LSPB profile on each
subplot(1,2,1), plot(q)                % each coordinate against time
subplot(1,2,2), plot(q(:,1), q(:,2))   % the path in the (q1,q2) plane: a straight line
```

The first argument is a *function handle* (`@lspb` or `@tpoly`) telling `mtraj` which 1D profile to use.

**Worked example.** At the half-way point, $s = 0.5$: $q_1 = 0 + 0.5 \cdot 1 = 0.5$, $q_2 = 2 + 0.5 \cdot (-3) = 0.5$. So the robot passes through $(0.5, 0.5)$, which is indeed the midpoint of the straight line from $(0,2)$ to $(1,-1)$.

The lecture file also notes you can build a 6-vector from a pose for this purpose, `q = [T1.t' T1.torpy]`, that is, translation followed by roll-pitch-yaw angles. Interpolating angles this way works, but see Section 12 for its limits.

---

## 11. Multi-segment trajectories through via points: `mstraj`

### 11.1 Why via points?

Often a straight line from start to goal is not acceptable. You may need to go *around* an obstacle, or follow a shape (Lab 1 asks you to draw letters). So you specify **via points**: intermediate configurations the trajectory should pass through or near. A trajectory through $M$ configurations has $M - 1$ **motion segments**.

### 11.2 The corner problem and the acceleration time $t_{acc}$

If you simply ran one LSPB per segment, the robot would come to a complete stop at every via point, which is slow and jerky. Instead, `mstraj` blends adjacent segments: as the robot approaches a via point it starts turning towards the next segment, cutting the corner smoothly. The slide's figure shows straight blue segments at constant velocity $\dot{q}_2, \dot{q}_3, \dot{q}_4$ joined by red curved blends of duration $t_{acc}$.

The key parameter is the **acceleration time** $t_{acc}$:

- $t_{acc}$ **small**: the blend is short, so the path passes *close to the via point*, but requires *high acceleration* at the corner.
- $t_{acc}$ **large**: the blend is long, the corner is *rounded further away* from the via point, but acceleration is *gentle*.

**Everyday picture.** A race car clipping an apex takes a tight line but pulls high g-force. A bus takes a wide, gentle arc. The via point is the apex, $t_{acc}$ is how much you round it.

Note that with blending the trajectory generally does **not pass exactly through** the via points (except the first and last). If you need to hit a point exactly you can make the robot stop there, or treat it as a start/end.

### 11.3 How segment timing is chosen: the slowest axis wins

Each axis $i$ has a maximum speed $\dot{q}_{i,\max}$. For a segment where axis $i$ must travel $|\Delta q_i|$, the minimum time it needs is $|\Delta q_i| / \dot{q}_{i,\max}$. `mstraj` computes this for every axis and takes the **largest**, because the segment can only be as fast as its slowest axis. Then every other axis is slowed down so that all axes arrive at the next via point simultaneously.

**Worked example.** Segment from $(0, 0)$ to $(4, 1)$ with maximum speeds $(2, 1)$. Axis x needs $4/2 = 2$ s. Axis y needs $1/1 = 1$ s. The segment therefore takes 2 s, and axis y is driven at only $1/2 = 0.5$, half its limit, so both axes arrive at the same moment.

### 11.4 `mstraj` in MATLAB

Signature, as the lecture file annotates it:

```matlab
traj = mstraj(via, qdmax, tseg, q0, dt, tacc)
%  via   : matrix of via points, one per row (NOT including the start)
%  qdmax : vector of maximum speed per axis
%  tseg  : vector of durations per segment, or [] to let qdmax decide
%  q0    : initial configuration
%  dt    : sample time step
%  tacc  : acceleration (blend) time
```

Example 1 from the lecture file, a path around a map (the Lab 1 Task 2 style):

```matlab
path = [20 30; 20 60; 50 60; 50 90; 90 90; 90 10; 20 10];
QDMAX = [1, 1];            % speed limit per axis
Q0 = path(1,:);            % start at the first point
DT = 0.2;
traj = mstraj(path, QDMAX, [], Q0, DT, 0);   % tacc = 0: sharp corners, stops at each via point
```

Example 2, a square rotated by 30°, showing the effect of $t_{acc}$:

```matlab
via = SO2(30, 'deg') * [-1 1; 1 1; 1 -1; -1 -1]';          % four corners of a tilted square (as columns)
q0 = mstraj(via(:,[2 3 4 1])', [2,1], [], via(:,1)', 0.2, 0);  % tacc = 0
q2 = mstraj(via(:,[2 3 4 1])', [2,1], [], via(:,1)', 0.2, 2);  % tacc = 2: rounded corners
[numrows(q0) numrows(q2)]    % q2 has more rows: blending adds time
```

Line by line: `via(:,[2 3 4 1])'` reorders the corners so we start at corner 1, visit 2, 3, 4 and return to 1. `[2,1]` says x may move at up to 2 units/s and y at up to 1. `[]` lets the speed limits decide segment durations. `via(:,1)'` is the starting corner. `0.2` is the sample time. The final argument is $t_{acc}$. Plotting `q2(:,1)` against `q2(:,2)` shows the same square but with rounded corners, and `numrows` shows it takes more samples (more time) because each blend adds $t_{acc}$.

The lecture file adds an important reminder: `mstraj` "simply interpolates pose represented as a vector." It does not know that three of your six numbers might be angles. See the next section for why that matters.

---

## 12. Interpolating orientation in 3D

### 12.1 Never linearly interpolate a rotation matrix

The slide is emphatic: **linear interpolation of rotation matrices SHOULD NOT be used**, because the result is not a valid orthonormal matrix.

**Worked example to see why.** Interpolate half-way between no rotation, $\mathbf{R}_0 = \mathbf{I}$, and a 180° turn about z, $\mathbf{R}_1 = \mathrm{diag}(-1, -1, 1)$. Averaging entry by entry gives $\tfrac{1}{2}(\mathbf{R}_0 + \mathbf{R}_1) = \mathrm{diag}(0, 0, 1)$. That matrix has determinant 0 and squashes the whole x-y plane to a point. It is not a rotation at all. Any sensible "half-way" answer should be a 90° rotation about z, and averaging matrix entries will never give you that.

### 12.2 Option A: interpolate a 3-angle representation

If you describe orientation with three angles $\Gamma = (\theta_r, \theta_p, \theta_y)$ (roll-pitch-yaw or Euler angles from Lecture 1), you can interpolate those three numbers linearly:

$$
\sigma(\Gamma_0, \Gamma_1, s) = (1 - s)\,\Gamma_0 + s\,\Gamma_1, \qquad s \in [0, 1]
$$

In words: at fraction $s$ of the way, take $(1-s)$ parts of the start angles plus $s$ parts of the end angles. At $s = 0$ you get $\Gamma_0$, at $s = 1$ you get $\Gamma_1$, and in between the angles slide smoothly. Converting each intermediate $\Gamma$ back to a rotation matrix always gives a valid rotation, so the orthonormality problem is gone.

```matlab
R0 = SO3.Rz(-1) * SO3.Ry(-1);        % start orientation
R1 = SO3.Rz(1) * SO3.Ry(1);          % end orientation
rpy0 = R0.torpy();  rpy1 = R1.torpy();   % convert both to roll-pitch-yaw angles
rpy = mtraj(@tpoly, rpy0, rpy1, 50);     % quintic interpolation of the three angles over 50 steps
SO3.rpy(rpy).animate;                    % convert back to rotations and animate
```

The drawbacks: the motion is a straight line in *angle space*, which is not a constant-speed rotation about a fixed axis in the real world, and it misbehaves near the gimbal-lock singularity you met in Lecture 1.

### 12.3 Option B: interpolate unit quaternions (preferred)

Unit quaternions (Lecture 1) have no gimbal lock, and interpolating between two of them along the shortest great-circle arc (called *slerp*, spherical linear interpolation) produces a rotation about a **single fixed axis at constant angular speed**. That is the most natural possible motion between two orientations.

```matlab
q0 = R0.UnitQuaternion;      % start as a unit quaternion
q1 = R1.UnitQuaternion;      % end as a unit quaternion
q = interp(q0, q1, 50);      % 50 interpolated quaternions
q.animate()                  % watch the frame rotate smoothly
```

### 12.4 Direction of rotation: the `'shortest'` option

Two orientations can always be joined in two ways around the circle: the short way and the long way. By default `interp` goes the way the quaternion numbers suggest, which is sometimes the long way.

```matlab
q0 = UnitQuaternion.Rz(-2);          % rotated -2 rad about z
q1 = UnitQuaternion.Rz(2);           % rotated +2 rad about z
q = interp(q0, q1, 50);  q.animate()               % turns through 4 rad (about 229°), the long way
q = interp(q0, q1, 50, 'shortest');  q.animate()   % turns through 2π − 4 ≈ 2.28 rad (about 131°), the short way
```

**Worked check.** From $-2$ rad to $+2$ rad the direct difference is 4 rad. But going the other way round the circle covers $2\pi - 4 \approx 2.28$ rad, which is shorter. The `'shortest'` flag picks that.

**Common mistake.** Assuming the quaternion interpolation "knows" which way you want to turn. A robot arm told to turn 229° when 131° would do may hit a joint limit or a cable. Always think about which direction you need.

---

## 13. Interpolating a full pose: `interp` and `ctraj`

A pose has translation *and* rotation. The toolbox's `SE3` class can interpolate both at once: translation linearly, rotation via quaternions.

```matlab
T0 = SE3([0.4, 0.2, 0]) * SE3.rpy(0, 0, 3);                 % start pose
T1 = SE3([-0.4, -0.2, 0.3]) * SE3.rpy(-pi/4, pi/4, -pi/2);  % end pose
interp(T0, T1, 0.5)          % the single pose half-way (s = 0.5)
Ts = interp(T0, T1, 50);     % 50 poses, with s spaced linearly from 0 to 1
about(Ts)                    % confirms it is a 1x50 array of SE3 objects
Ts(1)                        % the first pose (equals T0)
Ts.animate                   % animate a frame moving from T0 to T1
P = Ts.transl;  plot(P)      % the translation part against step: straight lines
rpy = Ts.torpy; plot(rpy)    % the roll-pitch-yaw angles against step
```

With $s$ spaced linearly, the velocity is constant and then jumps to zero at the ends. To get a smooth start and stop, feed `interp` an LSPB profile for $s$ instead:

```matlab
Ts = T0.interp(T1, lspb(0, 1, 50));   % s follows a trapezoidal profile: accelerate, cruise, decelerate
Ts = ctraj(T0, T1, 50);               % the compact way to write exactly that
```

`ctraj` ("Cartesian trajectory") is the function you will reach for most often: give it two poses and a number of steps, and it returns a smooth pose trajectory. In Lab 1 Task 1 you chain several of these (A to B, B to C, C to D) or use `mstraj` on 6-vectors to get the rounded corners the task asks for.

**Common mistake.** `interp(T0, T1, 50)` and `ctraj(T0, T1, 50)` both return 50 poses between the same endpoints, but only `ctraj` has the smooth velocity profile. If your animation starts and stops with a jolt, you used the wrong one.

---

## 14. How this connects to Lab 1

The last slide shows the Lab 1 scenes: a 2D maze map, a PUMA 560 arm drawing on a wall, 3D obstacle fields, the LTU fountain, and a wind turbine. Here is the mapping from lecture to lab:

- **Task 1 (object through poses A, B, C, D, smooth throughout, rounded corners):** this is `mstraj` on 6-vectors (translation + RPY) with a non-zero $t_{acc}$, or chained `ctraj` segments if stopping at via points is acceptable. Watch out for the orientation part: B has pitch $\pi/2$ and C has pitch $-\pi$, so think about the gimbal-lock discussion from Lecture 1 and the "shortest direction" issue from Section 12.
- **Task 2 (ground robot, speed limit $[1, 1]$, door that blocks for 5 s):** `mstraj` with `qdmax = [1 1]`, via points chosen to avoid obstacles. The "slowest axis wins" rule from Section 11.3 decides your segment times, and the door means inserting a waiting time or a via point so you arrive after it opens.
- **Task 3 (PUMA 560 drawing letters):** a sequence of via points on the wall plane, converted to poses, interpolated with `ctraj` or `mstraj`, then given to the arm. "Different speeds" means different step counts or speed limits for different strokes.
- **Task 4 (drone inspecting structures):** 3D via points with a 1 m safety distance and a 1.5 m × 1.5 m camera footprint, again fed to `mstraj`.

The optional exercises (7, 8, 9, 13) are direct experiments on `tpoly`, `lspb` and `mstraj` and are a very good way to make Sections 8, 9 and 11 stick.

---

## In a nutshell

This lecture adds time to the pose description from Lecture 1. A rotating frame has an angular velocity $\omega$, and the rotation matrix changes according to $\dot{\mathbf{R}} = S(\omega)\mathbf{R}$, where $S(\omega)$ is the skew-symmetric matrix that performs the cross product with $\omega$. Linear and angular velocity together form a 6-vector spatial velocity, and force and torque together form a wrench; both move between frames with the adjoint matrix (velocities use $\mathrm{Ad}$, wrenches use $\mathrm{Ad}^T$). Newton's law handles translation and Euler's equation handles rotation, with the inertia matrix playing the role of mass and the gyroscopic term making torque-free bodies tumble. To move a robot smoothly you design trajectories: a quintic polynomial matches six boundary conditions but has a high peak speed, while the trapezoidal LSPB profile cruises at constant speed with small acceleration jumps. For several axes you apply one profile to all of them (`mtraj`), for several waypoints you blend segments with an acceleration time (`mstraj`), and for orientation you interpolate angles or, better, unit quaternions, never the rotation matrix entries themselves.

## Key terms

- **Path**: a sequence of positions with no timing. **Trajectory**: a path with a time attached to every point.
- **Skew-symmetric matrix**: a square matrix with $S^T = -S$; $S(\omega)\mathbf{v} = \omega \times \mathbf{v}$.
- **$SO(3)$**: the set of all valid 3D rotation matrices (orthonormal, determinant +1).
- **Angular velocity $\omega$**: a vector whose direction is the instantaneous spin axis and whose length is the spin rate (rad/s).
- **Spatial velocity (twist) $\nu$**: the 6-vector (linear velocity, angular velocity) of a rigid body.
- **Jacobian / interaction matrix (here)**: the block-diagonal matrix that rotates a spatial velocity into another frame's coordinates (`velxform`).
- **Adjoint $\mathrm{Ad}(\xi)$**: the 6×6 matrix that transforms a spatial velocity to a different point and frame on the body, including the $\omega \times \mathbf{t}$ term (`.Ad`).
- **Inertia matrix $\mathbf{J}$**: 3×3 symmetric positive-definite matrix; diagonal entries are moments of inertia, off-diagonal are products of inertia.
- **Euler's equation**: $\mathbf{J}\dot{\omega} + \omega \times \mathbf{J}\omega = \tau$, the rotational version of Newton's second law.
- **Gyroscopic term**: $\omega \times \mathbf{J}\omega$; why a spinning body can change its axis with no torque applied.
- **Wrench $\mathbf{W}$**: the 6-vector (force, moment); transforms with $\mathrm{Ad}^T$.
- **Smooth trajectory**: position, velocity, acceleration (and ideally jerk) are all continuous.
- **Jerk**: the time derivative of acceleration.
- **Quintic polynomial**: 5th-order polynomial, 6 coefficients, matches position, velocity and acceleration at both ends (`tpoly`).
- **LSPB / trapezoidal profile**: linear segment with parabolic blends; constant-velocity cruise between constant-acceleration ramps (`lspb`).
- **Configuration vector $\mathbf{q}$**: the list of numbers (joint angles, coordinates, angles) that fixes a robot's state.
- **Via point**: an intermediate configuration a multi-segment trajectory passes through or near.
- **Acceleration time $t_{acc}$**: the blend duration at via points in `mstraj`; small means tight corners with high acceleration, large means rounded corners with gentle acceleration.
- **Slerp**: spherical linear interpolation of unit quaternions; constant-speed rotation about a fixed axis (`interp` on `UnitQuaternion`).
- **`ctraj`**: Cartesian trajectory between two poses with a trapezoidal time profile.

## Check yourself

1. Write down $S(\omega)$ for $\omega = (0, 0, 2)$ and use it to compute $\omega \times (1, 1, 0)$. Then explain in one sentence what $\dot{\mathbf{R}} = S(\omega)\mathbf{R}$ tells you.

2. A body's spatial velocity at its centre of mass {B} is $\mathbf{v} = (1, 0, 0)$ m/s, $\omega = (0, 0, 2)$ rad/s. A sensor frame {C} is mounted 0.5 m along B's x-axis with the same orientation. What is the linear velocity of the sensor? Which toolbox method would you use, `velxform` or `Ad`, and why?

3. You need a 1D move from 0 to 4 in $T = 2$ s with zero start and end velocity and acceleration. Without solving the full system, state the values of $D$, $E$ and $F$, and explain why the quintic will have a peak velocity noticeably above the average of 2 units/s.

4. For an `lspb` move of distance 1 over 50 time steps, roughly what range of cruise velocities is feasible, and what happens physically at each end of the range?

5. Why does the lecture say you must not linearly interpolate rotation matrices, and what two alternatives does it recommend? Which one gives rotation about a fixed axis at constant speed?

<details>
<summary>Answers</summary>

**1.** $S(\omega) = \begin{pmatrix} 0 & -2 & 0 \\ 2 & 0 & 0 \\ 0 & 0 & 0 \end{pmatrix}$. Multiplying by $(1, 1, 0)$: first row $-2$, second row $2$, third row $0$, so $\omega \times (1,1,0) = (-2, 2, 0)$. The equation $\dot{\mathbf{R}} = S(\omega)\mathbf{R}$ says that the rate of change of the rotation matrix is obtained by crossing the angular velocity with each column of the current rotation matrix, so angular velocity completely determines how the orientation evolves.

**2.** Linear velocity at C = $\mathbf{v}_B + \omega \times \mathbf{r}$, with $\mathbf{r} = (0.5, 0, 0)$ the offset from B to C. $\omega \times \mathbf{r} = (0, 0, 2) \times (0.5, 0, 0) = (0, 1, 0)$. So the sensor moves at $(1, 1, 0)$ m/s. The angular velocity is still $(0, 0, 2)$. You must use `Ad` (the full adjoint), because the frames are at different points on a rotating body and `velxform` would miss the $\omega \times \mathbf{r}$ contribution. (Equivalently, with ${}^C\mathbf{t}_B = (-0.5, 0, 0)$ the slide's term $[{}^C\mathbf{t}_B]_\times \omega = (-0.5,0,0)\times(0,0,2) = (0, 1, 0)$, the same result.)

**3.** From the $t = 0$ conditions: $F = s_0 = 0$, $E = \dot{s}_0 = 0$, $2D = \ddot{s}_0 = 0$ so $D = 0$. The quintic must start and end at rest, so it spends time speeding up and slowing down; to still cover 4 units in 2 s it has to exceed the average in the middle. For the zero-boundary quintic the peak is $1.875\times$ the average, so here about 3.75 units/s.

**4.** Roughly $0.02 < V \le 0.04$ (the average speed $d/T$ up to twice the average). At the low end the blends shrink to zero and the whole move is a constant-speed slide with no acceleration phase (and below it the distance cannot be covered in time). At the high end the cruise segment shrinks to zero and the velocity profile becomes a triangle: accelerate all the way to the midpoint, then decelerate. Above it the trajectory is infeasible and `lspb` errors.

**5.** Averaging the entries of two rotation matrices gives a matrix that is generally not orthonormal (for example the half-way point between $\mathbf{I}$ and a 180° turn about z collapses to $\mathrm{diag}(0,0,1)$, which has determinant 0), so it is not a rotation at all. The alternatives are (a) interpolate a three-angle representation such as roll-pitch-yaw, $\sigma = (1-s)\Gamma_0 + s\Gamma_1$, and convert back; or (b) interpolate unit quaternions with `interp` (slerp). Only the quaternion approach rotates about a single fixed axis at constant angular speed, and it also avoids gimbal lock.

</details>
