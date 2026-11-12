# Lecture 1 – Position and Orientation (R7010E Robotics)

> Study notes built from `Final_R7010E_Lecture_1_Pose_v1.pdf` (35 slides), `Matlab_Time_Lecture_1.m` and the live script `Matlab_Time_Lecture_1_2_live_2025.mlx`. The slides follow Chapter 2 of Peter Corke's *Robotics, Vision and Control*. In the course plan this one deck covers both "Lecture 1: Intro and Representing Position and Orientation" and "Lecture 2: Representing Position and Orientation (continuation)".

## What this lecture is about, and why it matters

This lecture answers the most basic question a robot ever has to deal with: *where is something, and which way is it facing?* You will learn how to describe the **pose** (position plus orientation) of a robot, a camera or an object with coordinate frames, how to chain those descriptions together, and how to write them as matrices (and later quaternions) so a computer can calculate with them. Every later lecture in this course, from driving a TurtleBot to flying a Crazyflie to building a map with SLAM, sits on top of these ideas, so it is worth getting them really solid now.

## Roadmap: the main topics

1. Course logistics (what you need to know to pass).
2. What a robot is, the fundamental questions of robotics, and the classical robot architecture.
3. Coordinate frames, pose, and relative pose (the symbol $\xi$).
4. The algebra of poses: the $\oplus$ and $\ominus$ operators, their rules, and the pose graph example (robot relative to a fixed camera).
5. Pose in 2D: the rotation matrix, its properties, and the homogeneous transformation matrix (SE(2)).
6. Composing and inverting homogeneous transformations.
7. Pose in 3D: the three basic rotation matrices $R_x, R_y, R_z$ and the 4×4 transformation (SE(3)).
8. Describing 3D orientation with three angles: Euler angles (ZYZ) and roll‑pitch‑yaw (ZYX), with the NED‑frame yaw example.
9. Gimbal lock (the singularity of three‑angle representations), including the Apollo 13 story.
10. Quaternions and unit quaternions as an alternative way to represent rotation.
11. MATLAB Robotics Toolbox commands used in the lecture ("MATLAB time").

---

## 1. Course logistics (quick, but important)

- **Course code:** R7010E – Robotics, 7.5 credits, Luleå University of Technology.
- **Lecturer and examiner:** George Nikolakopoulos. **Lab assistants:** Christoforos Kanellakis and Björn Lindqvist.
- **Textbook:** *Robotics, Vision and Control* by Peter Corke, plus the Robotics Toolbox for MATLAB (petercorke.com).
- **Robots used:** the TurtleBot3 (a small ground robot with a 360° LiDAR, a Raspberry Pi and two Dynamixel wheel motors) and the Crazyflie 2 (a palm‑sized quadrotor drone).
- **Grading:** U, 3, 4, 5. **There is no final written exam.** You pass by completing the lab assignments and passing all the demonstrations. So when these notes say "exam", read it as "lab report, demonstration and oral questioning".
- **Course structure (16 sessions):** pose (1–2), time and motion (3), Lab 1, mobile robotics (5), Lab 2, aerial robotics (7), programming robots with MATLAB/ROS/LabVIEW (8), navigation (9–10), Lab 3, localization and mapping (12–13), sensors and perception (14), Lab 4 (2D SLAM), robotic vision (16).

---

## 2. What is a robot, and how is one organised?

### 2.1 The definition

The slides give one sentence you should memorise:

> **A robot is a goal‑oriented machine that can sense, plan and act autonomously.**

Take it apart. *Goal‑oriented* means it is trying to achieve something, not just moving randomly. *Sense* means it has sensors (cameras, LiDAR, wheel encoders) to learn about the world. *Plan* means it decides what to do based on what it sensed. *Act* means it has actuators (motors, propellers) to change the world. *Autonomously* means it does this without a human steering it moment to moment.

**Everyday analogy:** a thermostat senses temperature and acts (turns the heater on), but it does not really plan toward a goal in a changing world, so people do not usually call it a robot. A robot vacuum senses walls, plans a path around your living room and drives itself, so it qualifies.

The slide on aerial robots adds two terms. **UAV (Unmanned Aerial Vehicle)** is the general name for an aircraft with no pilot on board, i.e. an aerial robot. **RPAV (Remotely Piloted Aerial Vehicle)** is what most people mean by a "drone": a UAV that a human is still flying by remote control. The difference is who does the planning: the on‑board computer (UAV, autonomous) or a human with a joystick (RPAV).

### 2.2 The fundamental questions of robotics

Every autonomous robot has to answer five questions, and the whole course is organised around them:

1. **Where am I?** (localization – Lecture 10)
2. **What is the surrounding environment?** (perception and mapping – Lectures 8, 10, 11)
3. **Where should I go, what should I do?** (mission planning)
4. **How do I get from A to B?** (navigation and path planning – Lecture 7)
5. **How do I perform the task autonomously?** (control – Lectures 3, 4, 6)

### 2.3 A classical robotic architecture

The slide shows a chain of blocks with a feedback loop:

```
Mission Planner → Path Planning → Control Layer → [Robot] → Perception
                        ↑               ↑                        |
                        └───────────────┴────────────────────────┘
```

Reading left to right:

- The **Mission Planner** decides the high‑level goal ("go to the kitchen").
- **Path Planning** turns that goal into a route the robot can actually follow, avoiding obstacles.
- The **Control Layer** turns the route into motor commands, moment by moment.
- The **Robot** (the physical machine) moves.
- **Perception** uses the sensors to measure what actually happened, and feeds that back to both the path planner (so it can re‑plan if something is in the way) and the controller (so it can correct errors).

**Everyday analogy:** think of a road trip. You decide to visit your grandmother (mission). Your sat‑nav picks the roads (path planning). Your hands and feet steer and brake (control). Your eyes watch the road (perception) and tell your hands to correct, or tell the sat‑nav to re‑route when a road is closed.

Why does this matter for *this* lecture? Because every arrow in that diagram carries information about *where things are*: where the robot is, where the goal is, where the obstacle is. All of that has to be written in a common mathematical language, and that language is the pose.

---

## 3. Coordinate frames, pose and relative pose

### 3.1 Coordinate frames

A **coordinate frame** is a set of perpendicular axes (x and y in 2D; x, y and z in 3D) meeting at a point called the **origin**. We write a frame with curly braces, such as $\{A\}$ or $\{B\}$. The world frame is often written $\{O\}$ or $\{W\}$.

A **point** $P$ is located by its **coordinate vector**: how far along each axis you must go from the origin to reach it. In 2D, $P = x\,\hat{x} + y\,\hat{y}$, which just says "go $x$ units along the x‑axis direction and $y$ units along the y‑axis direction". The hats on $\hat{x}, \hat{y}$ mean *unit vectors*, arrows of length one pointing along the axes.

The key insight is that *a point has different coordinates in different frames*. A point that is at $(3, 2)$ in the world frame might be at $(1.7, -1.0)$ in a robot's own frame. The numbers change, the point does not.

### 3.2 Pose

**Pose** means *position and orientation together*. For a robot in a plane, that is where it is ($x, y$) and which way it faces (an angle $\theta$). We attach a frame $\{B\}$ to the robot body; the pose of the robot is simply the pose of its frame $\{B\}$.

### 3.3 Relative pose, $\xi$

The Greek letter $\xi$ (pronounced "ksi") stands for a **relative pose**: the position and orientation of one frame *as seen from* another frame.

Notation: ${}^A\xi_B$ is "the pose of $\{B\}$ relative to $\{A\}$". The leading superscript is the frame you are standing in (the reference); the subscript is the frame you are describing. If no superscript is written, the reference is the world frame.

Now the most important formula of the whole section. If you know a point's coordinates in $\{B\}$ (written ${}^B p$) and you know the pose of $\{B\}$ relative to $\{A\}$, you can get the coordinates in $\{A\}$:

$$ {}^A p = {}^A\xi_B \cdot {}^B p $$

In words: "the point seen from A equals the relative pose of B with respect to A, *applied to* the point seen from B". The dot means "apply the pose to the vector". Notice how the letters line up: the $B$ at the bottom right of $\xi$ matches the $B$ on $p$, and they "cancel" leaving $A$. That letter‑matching trick is a great way to check your work.

### 3.4 Why bother with frames at all?

Slide 18 shows the reason with a picture. There is a world frame $\{O\}$, a fixed camera $\{F\}$ on the ceiling, an object $\{B\}$ on the floor, a robot $\{R\}$ and a camera mounted on the robot $\{C\}$. Each sensor measures things *in its own frame*: the ceiling camera sees the object relative to itself (${}^F\xi_B$), the robot's camera sees the object relative to itself (${}^C\xi_B$), and the robot's camera is bolted to the robot at a known offset (${}^R\xi_C$). To make all this useful (for example to find where the robot is), you need a way to *combine* relative poses. That is what the next section gives you.

**Everyday analogy:** a friend phones you and says "the café is 200 m ahead of me on my left". That is useful only if you know where your friend is and which way they are facing. Their description is in *their* frame; you need to convert it into *your* frame (or the city map's frame). Pose algebra is exactly that conversion.

---

## 4. The algebra of poses: $\oplus$ and $\ominus$

### 4.1 Compounding (chaining) poses

If you know ${}^A\xi_B$ (B relative to A) and ${}^B\xi_C$ (C relative to B), then the pose of C relative to A is:

$$ {}^A\xi_C = {}^A\xi_B \oplus {}^B\xi_C $$

The circled plus $\oplus$ is the **compounding operator**: it means "first apply the pose on the left, then the pose on the right". Again, watch the letters: the inner $B$'s cancel and you are left with $A$ and $C$.

And a point can be carried all the way through a chain:

$$ {}^A p = {}^A\xi_B \oplus {}^B\xi_C \cdot {}^C p $$

The slide stresses that this works identically in 3D.

### 4.2 The rules (slide 20)

Write $0$ for the "null pose" (no displacement, no rotation). Then:

| Rule | Meaning in words |
|---|---|
| $\xi \oplus 0 = \xi$ and $\xi \ominus 0 = \xi$ | Adding or removing "nothing" changes nothing. |
| $\xi \ominus \xi = 0$ and $\ominus\xi \oplus \xi = 0$ | A pose followed by its own inverse gets you back where you started. |
| $\ominus\,{}^X\xi_Y = {}^Y\xi_X$ | **Inverse of a pose:** flipping the reference and target frames is the same as taking the inverse. "B seen from A" inverted is "A seen from B". |
| ${}^X\xi_Y \oplus {}^Y\xi_Z = {}^X\xi_Z$ | **Composition:** chain through a middle frame. |
| $\xi_1 \oplus \xi_2 \neq \xi_2 \oplus \xi_1$ | **Not commutative!** Order matters. |
| ${}^x p = {}^x\xi_y \cdot {}^y p$ | Applying a pose to a point changes the point's frame of reference. |

The circled minus $\ominus$ used alone in front of a pose ($\ominus\xi$) means **the inverse pose**. Used between two poses ($\xi_1 \ominus \xi_2$) it means $\xi_1 \oplus (\ominus\xi_2)$.

⚠️ The slide writes "No commutative, except $\xi_1 \oplus \xi_2 = 0$". In plain words: order of composition matters in general; the only obvious exception is when one pose is the inverse of the other (then both orders give the null pose). There are a few other special cases where order does not matter (for example two pure translations, or two rotations about the same axis), but you should treat non‑commutativity as the rule.

**Common mistake:** students often think "it's just adding displacements, so order can't matter". It does, because the second displacement is applied *in the rotated frame* produced by the first. Walking 1 m forward then turning 90° leaves you in a different place than turning 90° then walking 1 m forward. You will see this numerically in the MATLAB section (frames {3} and {4}).

### 4.3 The pose graph example: where is the robot relative to the fixed camera?

Slide 19 redraws the camera/robot scene as a **directed graph**: every frame is a **node** (a dot) and every known relative pose is an **edge** (an arrow). There are two different routes from $\{O\}$ to the object $\{B\}$:

- Route 1: world → fixed camera → object: $\xi_F \oplus {}^F\xi_B$
- Route 2: world → robot → robot camera → object: $\xi_R \oplus {}^R\xi_C \oplus {}^C\xi_B$

Both describe the same physical thing (where the object is), so they must be equal:

$$ \xi_F \oplus {}^F\xi_B = \xi_R \oplus {}^R\xi_C \oplus {}^C\xi_B $$

That is one equation you could solve for an unknown edge, for example the robot's world pose $\xi_R$ if the robot camera sees the object and the fixed camera sees it too.

The slide then asks: **what is the pose of the robot relative to the fixed camera, ${}^F\xi_R$?** Walk through it step by step.

1. Write the two routes from the world to the robot. Directly: $\xi_R$ (written ${}^0\xi_R$ on the slide, with 0 meaning the world). Via the camera: $\xi_F \oplus {}^F\xi_R$. They must be equal:
   $$ \xi_F \oplus {}^F\xi_R = \xi_R $$
2. You want ${}^F\xi_R$ alone, so you need to get rid of $\xi_F$ on the left. "Subtract" it by compounding **on the left** with its inverse $\ominus\xi_F$ **on both sides** (you must do it on the same side on both sides, because order matters):
   $$ \ominus\xi_F \oplus \xi_F \oplus {}^F\xi_R = \ominus\xi_F \oplus \xi_R $$
3. By the rule $\ominus\xi \oplus \xi = 0$, the left side collapses:
   $$ {}^F\xi_R = \ominus\xi_F \oplus \xi_R $$

Read the answer in words: "to go from the fixed camera to the robot, first undo the camera's world pose (go from the camera back to the world), then apply the robot's world pose (go from the world to the robot)". Check the letters: $\ominus\xi_F = {}^F\xi_O$, and ${}^F\xi_O \oplus {}^O\xi_R = {}^F\xi_R$. 

**Everyday analogy:** to give directions from the café to the station when you only know directions from your house to each, you say "go from the café back to my house (reverse the café directions), then follow the directions from my house to the station".

---

## 5. Pose in two dimensions

Now we make $\xi$ concrete with numbers. In 2D a relative pose ${}^A\xi_B$ is described by three numbers $(x, y, \theta)$: the position of B's origin in A, and the angle $\theta$ that B's x‑axis makes with A's x‑axis (positive counter‑clockwise).

⚠️ Slide 21 crosses out the expression $(x_1, y_1, \theta_1) \oplus (x_2, y_2, \theta_2)$. The point is that you **cannot** compound two poses just by adding the three numbers component‑wise. That is exactly why we need matrices.

### 5.1 Rotation only: the 2D rotation matrix

Slide 22 introduces a helper frame $\{V\}$: it has the *same origin* as $\{B\}$ but its axes are *parallel* to $\{A\}$. So going from $\{V\}$ to $\{B\}$ is a pure rotation by $\theta$, and going from $\{A\}$ to $\{V\}$ is a pure translation. We handle rotation first.

Look at the picture: the unit vector $\hat{x}_B$ (B's x‑axis), seen in frame V, has components $(\cos\theta, \sin\theta)$. The unit vector $\hat{y}_B$ has components $(-\sin\theta, \cos\theta)$. Stack those two as the **columns** of a matrix and you get the **rotation matrix**:

$$ {}^V\!R_B = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} $$

**Memory aid:** the columns of a rotation matrix are the axes of the *new* frame (B) written in the coordinates of the *old* frame (V).

A point with coordinates $({}^Bx, {}^By)$ in B has coordinates in V given by:

$$ \begin{bmatrix} {}^Vx \\ {}^Vy \end{bmatrix} = {}^V\!R_B \begin{bmatrix} {}^Bx \\ {}^By \end{bmatrix} = \begin{bmatrix} \cos\theta\,{}^Bx - \sin\theta\,{}^By \\ \sin\theta\,{}^Bx + \cos\theta\,{}^By \end{bmatrix} $$

**Worked example.** Let $\theta = 30°$, so $\cos 30° = 0.866$ and $\sin 30° = 0.5$. Take the point ${}^Bp = (2, 1)$.

- ${}^Vx = 0.866 \cdot 2 - 0.5 \cdot 1 = 1.732 - 0.5 = 1.232$
- ${}^Vy = 0.5 \cdot 2 + 0.866 \cdot 1 = 1.0 + 0.866 = 1.866$

So ${}^Vp = (1.232, 1.866)$. Sanity check: the length of $(2,1)$ is $\sqrt{5} = 2.236$ and the length of $(1.232, 1.866)$ is $\sqrt{1.518 + 3.482} = \sqrt{5.0} = 2.236$. Rotation never changes lengths, so this is consistent.

### 5.2 Properties of the rotation matrix (slide 23)

These three facts are asked about constantly:

1. **R is orthogonal.** Its columns are unit vectors that are perpendicular to each other. (They are the axes of a frame, so of course they are.)
2. **det R = +1.** The determinant is exactly one. For the 2D matrix: $\cos^2\theta + \sin^2\theta = 1$. A determinant of −1 would mean a mirror image, which no physical rotation can produce.
3. **The inverse is the transpose:** $R^{-1} = R^T$. Also $R(-\theta) = R(\theta)^T$: rotating backwards is the transpose.

Property 3 is a gift: inverting a general matrix is tedious, but transposing is just flipping rows and columns. So to go the other way, from V to B:

$$ \begin{bmatrix} {}^Bx \\ {}^By \end{bmatrix} = ({}^V\!R_B)^{-1}\begin{bmatrix} {}^Vx \\ {}^Vy \end{bmatrix} = ({}^V\!R_B)^{T}\begin{bmatrix} {}^Vx \\ {}^Vy \end{bmatrix} = {}^B\!R_V \begin{bmatrix} {}^Vx \\ {}^Vy \end{bmatrix} $$

**Worked check.** With $\theta = 30°$, $R^T = \begin{bmatrix} 0.866 & 0.5 \\ -0.5 & 0.866\end{bmatrix}$. Apply it to $(1.232, 1.866)$: $x = 0.866\cdot1.232 + 0.5\cdot1.866 = 1.067 + 0.933 = 2.0$; $y = -0.5\cdot1.232 + 0.866\cdot1.866 = -0.616 + 1.616 = 1.0$. We get $(2, 1)$ back. 

**Common mistake:** mixing up which way the matrix goes. ${}^V\!R_B$ takes coordinates *from* B *to* V (the subscript is the frame you start in; the superscript is the frame you end up in). If your answer comes out rotated the wrong way, you probably needed the transpose.

### 5.3 Adding translation: the homogeneous transformation (slide 24)

Now bring back the translation. Since $\{V\}$ and $\{A\}$ have parallel axes, going from V to A just adds the position $t = (x, y)$ of B's origin:

$$ \begin{bmatrix} {}^Ax \\ {}^Ay \end{bmatrix} = \begin{bmatrix} {}^Vx \\ {}^Vy \end{bmatrix} + \begin{bmatrix} x \\ y \end{bmatrix} = {}^A\!R_B \begin{bmatrix} {}^Bx \\ {}^By \end{bmatrix} + t $$

That is "rotate, then add" — a two‑step recipe, which is awkward when you want to chain many frames. The trick is to add a third coordinate that is always 1. Then rotation and translation fit in **one** 3×3 matrix multiplication:

$$ \begin{bmatrix} {}^Ax \\ {}^Ay \\ 1 \end{bmatrix} = \begin{bmatrix} {}^A\!R_B & t \\ 0_{1\times2} & 1 \end{bmatrix} \begin{bmatrix} {}^Bx \\ {}^By \\ 1 \end{bmatrix} \qquad\Longrightarrow\qquad {}^A\tilde{p} = {}^A T_B\,{}^B\tilde{p} $$

Here:

- ${}^A\!R_B$ is the 2×2 rotation block (top‑left).
- $t = (x, y)^T$ is the translation column (top‑right).
- $0_{1\times2} = [0\ 0]$ is a row of zeros (bottom‑left).
- The bottom‑right $1$ is a **scaling** factor; for rigid motions it is always 1.

$T$ is called a **homogeneous transformation matrix**, and the vectors with the extra 1 are in **homogeneous coordinates** (the tilde on $\tilde p$ marks this). Written out:

$$ {}^A T_B = \begin{bmatrix} \cos\theta & -\sin\theta & x \\ \sin\theta & \cos\theta & y \\ 0 & 0 & 1 \end{bmatrix} $$

⚠️ **Watch out for a typo on slide 24.** The coloured matrix at the bottom of that slide shows the rotation block as $\begin{bmatrix}\cos\theta & \sin\theta\\ -\sin\theta & \cos\theta\end{bmatrix}$, i.e. with the signs of $\sin\theta$ swapped compared with the rotation matrix defined on slide 23. That is the rotation by $-\theta$ (the transpose). For a counter‑clockwise rotation by $+\theta$ use the form above, with $-\sin\theta$ in the top‑right of the rotation block. Check with your lecturer if unsure, but the MATLAB output (`transl2(1,2)*trot2(30,'deg')` gives `-0.5` in position (1,2)) confirms the version in these notes.

**Why do we call it "homogeneous"?** Because adding the 1 makes translation look like multiplication (homogeneous means "of the same kind"): both rotation and translation are now handled by the same operation, matrix multiplication. That is what lets us chain poses by multiplying matrices.

**Worked example.** B is at $(1, 2)$ in A and rotated $30°$. Then

$$ {}^A T_B = \begin{bmatrix} 0.866 & -0.5 & 1 \\ 0.5 & 0.866 & 2 \\ 0 & 0 & 1 \end{bmatrix} $$

The point ${}^Bp = (2, 1)$ becomes ${}^Ap$: $x = 0.866\cdot2 - 0.5\cdot1 + 1 = 2.232$; $y = 0.5\cdot2 + 0.866\cdot1 + 2 = 3.866$; and the third entry is $0+0+1 = 1$. So ${}^Ap = (2.232, 3.866)$. Notice that this is exactly our earlier rotated point $(1.232, 1.866)$ plus the translation $(1, 2)$. Good.

### 5.4 Composition and inverse of transformations (slide 25)

Now the abstract symbols get concrete meanings. The set of all 2D homogeneous transformations is called **SE(2)** ("special Euclidean group in 2 dimensions"; you only need the name). The dictionary is:

| Abstract | Concrete |
|---|---|
| relative pose $\xi$ | matrix $T \in SE(2)$ |
| $\xi_1 \oplus \xi_2$ | $T_1 \cdot T_2$ (matrix product) |
| null pose $0$ | identity matrix $I$ |
| $\ominus\xi$ | $T^{-1}$ (matrix inverse) |
| $\xi \cdot p$ | $T\tilde{p}$ (matrix times homogeneous vector) |

**Composition formula.** Multiply two transformations block by block:

$$ T_1 T_2 = \begin{bmatrix} R_1 & t_1 \\ 0 & 1\end{bmatrix}\begin{bmatrix} R_2 & t_2 \\ 0 & 1\end{bmatrix} = \begin{bmatrix} R_1 R_2 & t_1 + R_1 t_2 \\ 0 & 1\end{bmatrix} $$

Read it: the combined rotation is the product of rotations, and the combined translation is "the first translation plus the second translation *rotated by the first rotation*". That $R_1 t_2$ term is the reason order matters.

**Worked example (this is the MATLAB demo on slides and in the .m file).** $T_1$ = translate $(1,2)$ then rotate $30°$ (matrix above). $T_2$ = pure translation $(2,1)$, so $R_2 = I$, $t_2 = (2,1)$.

- $T_3 = T_1 T_2$: rotation $R_1 I = R_1$ (still $30°$). Translation $t_1 + R_1 t_2 = (1,2) + (0.866\cdot2 - 0.5\cdot1,\ 0.5\cdot2 + 0.866\cdot1) = (1,2) + (1.232, 1.866) = (2.232, 3.866)$.
- $T_4 = T_2 T_1$: rotation $I R_1 = R_1$ (still $30°$). Translation $t_2 + I t_1 = (2,1) + (1,2) = (3, 3)$.

Same rotation, **different positions**: $(2.232, 3.866)$ versus $(3, 3)$. Frame {3} is not frame {4}. This is non‑commutativity made visible. Intuitively, in $T_1 T_2$ the step of $(2,1)$ is taken *in frame {1}*, which is already rotated by 30°, so the step does not land at $(3,3)$.

**Identity and inverse rules** carry over: $T \cdot I = T$ and $T\,T^{-1} = I$.

**Inverse formula.** You do not need to invert a 3×3 matrix the hard way:

$$ T^{-1} = \begin{bmatrix} R & t \\ 0 & 1\end{bmatrix}^{-1} = \begin{bmatrix} R^T & -R^T t \\ 0 & 1 \end{bmatrix} $$

In words: transpose the rotation, and the new translation is minus the transposed rotation applied to the old translation.

**Worked example.** Invert $T_1$ ($\theta = 30°$, $t = (1, 2)$). $R^T = \begin{bmatrix}0.866 & 0.5\\-0.5 & 0.866\end{bmatrix}$. $R^T t = (0.866\cdot1 + 0.5\cdot2,\ -0.5\cdot1 + 0.866\cdot2) = (1.866, 1.232)$, so $-R^T t = (-1.866, -1.232)$:

$$ T_1^{-1} = \begin{bmatrix} 0.866 & 0.5 & -1.866 \\ -0.5 & 0.866 & -1.232 \\ 0 & 0 & 1\end{bmatrix} $$

Use it: where is the world point $P = (3, 2)$ in frame {1}? Compute $T_1^{-1}\tilde P$: $x = 0.866\cdot3 + 0.5\cdot2 - 1.866 = 2.598 + 1.0 - 1.866 = 1.732$; $y = -0.5\cdot3 + 0.866\cdot2 - 1.232 = -1.5 + 1.732 - 1.232 = -1.0$. So ${}^1P = (1.732, -1.0)$, which matches MATLAB's `inv(T1)*[P;1]` output (`1.7321, -1.0000, 1`). The negative y makes sense: P is on the right‑hand side of frame {1}'s x‑axis.

**Common mistake:** writing $T^{-1}$ as $\begin{bmatrix} R^T & -t \\ 0 & 1\end{bmatrix}$ (forgetting to rotate $t$). The translation must be expressed in the *new* frame, hence the $R^T$.

---

## 6. Pose in three dimensions

### 6.1 Right‑handed frames

In 3D a point is $P = x\hat{x} + y\hat{y} + z\hat{z}$. Frames are **right‑handed**: point your right index finger along x and your middle finger along y; your thumb points along z. (Slide 26 shows the left‑ and right‑hand pictures.) Robotics always uses right‑handed frames; a left‑handed one would silently flip the sign of some rotations.

### 6.2 The three elementary rotation matrices

A 3D rotation matrix ${}^A\!R_B$ is 3×3 and maps ${}^Bp$ to ${}^Ap$ exactly as in 2D. Any rotation can be built from rotations about the three axes:

$$ R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta & \cos\theta \end{bmatrix},\quad
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix},\quad
R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} $$

How to remember them: the axis you rotate *about* is untouched, so its row and column are those of the identity (a 1 and zeros). The remaining 2×2 block is the familiar 2D rotation. **The only oddity is $R_y$:** the minus sign sits on the *lower‑left* $\sin\theta$, not the upper‑right. That is because the cyclic order of axes is x→y→z→x; for a rotation about y, the "next" axis is z and the "previous" one is x, which flips the usual layout.

**Worked example.** Rotate the point $(1, 0, 0)$ by $90°$ about z. $R_z(90°)$ has $\cos = 0$, $\sin = 1$: first column is $(0, 1, 0)$, and multiplying by $(1,0,0)^T$ picks out exactly the first column. So the x‑axis rotates onto the y‑axis, as expected (counter‑clockwise seen from above).

All the 2D properties hold: orthogonal, $\det R = 1$, $R^{-1} = R^T$.

### 6.3 Homogeneous transformations in 3D: SE(3)

Add the translation $t = (x, y, z)^T$ and a fourth homogeneous coordinate to get a 4×4 matrix:

$$ \begin{bmatrix} {}^Ax \\ {}^Ay \\ {}^Az \\ 1 \end{bmatrix} = \begin{bmatrix} {}^A\!R_B & t \\ 0_{1\times3} & 1 \end{bmatrix}\begin{bmatrix} {}^Bx \\ {}^By \\ {}^Bz \\ 1 \end{bmatrix}, \qquad {}^A\tilde p = {}^AT_B\,{}^B\tilde p $$

The set of these is **SE(3)**. Slide 28 is the 3D twin of slide 25 and the formulas are *identical in form*:

- Composition: $T_1 T_2 = \begin{bmatrix} R_1R_2 & t_1 + R_1 t_2 \\ 0 & 1\end{bmatrix}$
- Identity: $T \oplus I = T$; inverse: $T_1 T_1^{-1} = I$
- Inverse: $T^{-1} = \begin{bmatrix} R^T & -R^Tt \\ 0 & 1\end{bmatrix}$

The only difference is the size: $R$ is 3×3, $t$ is 3×1, and the zero row is $1\times3$.

---

## 7. Three‑angle descriptions of orientation: Euler angles and roll‑pitch‑yaw

A 3×3 rotation matrix has nine numbers but only three independent "degrees of freedom" (a rigid body can only rotate in three independent ways). So people like to describe orientation with just three angles. There are many conventions; the lecture shows the two you will meet most.

### 7.1 Euler angles, ZYZ sequence

$$ R = R_z(\varphi)\, R_y(\theta)\, R_z(\psi), \qquad \Gamma = (\varphi, \theta, \psi) $$

In words: rotate about z by $\varphi$, then about the *new* y by $\theta$, then about the *new* z by $\psi$. The first and last rotations are about the same (body) axis, which is what makes this "Euler" style. The slide shows a gimbal (nested rings) and a robot arm wrist, because this sequence is the natural description of a spherical wrist on a manipulator.

⚠️ Slide 29 says "ZYZ sequence is commonly used in aeronautics and ground robotics", but slide 30 (and the MATLAB live script) say that *vehicles* use the ZYX roll‑pitch‑yaw sequence, and Corke's book associates ZYZ with robot‑arm wrists. Treat ZYZ as the manipulator/wrist convention and ZYX as the vehicle convention, and check with your lecturer if this comes up.

### 7.2 Roll‑pitch‑yaw, the vehicle convention

For a vehicle, the body frame usually has **x forward, y to the side, z up or down**. Then:

- **Roll** $\theta_r$ is rotation about the forward (x) axis: one wing dips, like a boat rocking side to side.
- **Pitch** $\theta_p$ is rotation about the sideways (y) axis: nose up or down.
- **Yaw** $\theta_y$ is rotation about the vertical (z) axis: turning left or right, i.e. the heading.

The **ZYX sequence** is:

$$ R = R_z(\theta_y)\, R_y(\theta_p)\, R_x(\theta_r) $$

Read right to left when thinking about the body: roll first about the body x, then pitch, then yaw. Read left to right when thinking about the fixed world axes. Both views give the same matrix. The toolbox uses ZYX as its default for `rpy2r`.

⚠️ **Typo on slide 30.** The boxed formula there is written $R = R_z(\theta_y)R_y(\theta_p)R_z(\theta_r)$; the last factor should be $R_x(\theta_r)$ for a ZYX sequence (roll is about x). The picture on the same slide shows roll about X, which confirms this.

⚠️ Slide 29 also shows an **XYZ sequence** $R = R_x(\theta_y)R_y(\theta_p)R_z(\theta_r)$ labelled "Yaw, Pitch, Roll". This is the convention for a *robot gripper*, where (as the live script says) the z‑axis points forward and x is up or down, so yaw is about x and roll is about z. It is easy to confuse with the vehicle case. The rule: **roll is always about the forward axis, yaw about the up/down axis**; which letter that is depends on how the body frame is attached.

### 7.3 Example: yaw in the NED frame (slide 31)

Aerial robots often use the **NED frame**: x points **N**orth, y points **E**ast, z points **D**own. (Yes, down: so that a right‑handed frame with x north and y east has z pointing into the ground.) Slide 31 shows a drone whose heading differs from north by the yaw angle $\psi$. If a vector is known in the vehicle's body frame ($p^v$) you get it in the NED frame ($p^D$) by:

$$ p^D = R^D_v\, p^v, \qquad R^D_v = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix} $$

That is just $R_z(\psi)$: a yaw is a rotation about the vertical axis. **Worked example:** the drone points $\psi = 90°$ (east). A sensor reading of "1 m straight ahead" is $p^v = (1, 0, 0)$. In NED: $p^D = (\cos90°, \sin90°, 0) = (0, 1, 0)$, i.e. 1 m east. Correct.

The same slide shows the opposite situation (left picture): a world frame painted on the floor (x North, y East, z Down) seen by a camera on the drone, and the pose (R, T) between the body frame and the world frame. This is exactly the fixed‑camera/robot‑camera problem from Section 4, now in 3D.

---

## 8. Gimbal lock

### 8.1 What it is

A **gimbal** is a set of nested rings that lets a platform rotate freely about three axes. Slide 32 shows the Apollo Lunar Module's inertial measurement unit: outer, middle and inner gimbals carrying a stable platform $\{S\}$ inside the body $\{B\}$. Each ring measures one of the three angles.

**Gimbal lock** happens when two of the three rotation axes line up. At that moment the system has lost one degree of freedom: there is a direction you simply cannot rotate in (or, mathematically, two of your three angles do the same job and you cannot tell them apart). The right‑hand pictures on slide 32 show the normal case (three distinct rings) and the locked case (two rings lying in the same plane).

This is not a flaw of one particular gimbal. **Every three‑angle representation has at least one such singularity.** It is built into the mathematics.

### 8.2 The ZYZ numerical example on slide 33

Take Euler angles $\Gamma = (0.1, 0, 0.3)$ in the ZYZ sequence. The middle angle is $\theta = 0$, so $R_y(0) = I$ (rotating by zero does nothing). Then:

$$ R = R_z(0.1)\, I\, R_z(0.3) = R_z(0.1 + 0.3) = R_z(0.4) $$

With $\cos 0.4 = 0.9211$ and $\sin 0.4 = 0.3894$:

$$ R = \begin{bmatrix} 0.9211 & -0.3894 & 0 \\ 0.3894 & 0.9211 & 0 \\ 0 & 0 & 1\end{bmatrix} $$

Now ask the inverse question: which Euler angles produce this $R$? Any pair with $\varphi + \psi = 0.4$ works: $(0.1, 0, 0.3)$, $(0.2, 0, 0.2)$, $(0, 0, 0.4)$, and infinitely many more. The toolbox function `tr2eul` returns $\Gamma = (0, 0, 0.4)$ by convention (it sets $\varphi = 0$). The slide labels $\theta = 0$ as the **singularity**: when the middle angle is zero, the first and third rotations are about the *same* axis and the three‑angle description is no longer unique.

### 8.3 The Apollo 13 case (YZX sequence)

The Apollo platform used a sequence written on the slide as

$$ {}^S R_B = R_y(\theta_p)\, R_z(\theta_r)\, R_x(\theta_y) $$

(The angle names on this slide follow the gimbal axis labels in the Apollo drawing, where the X‑axis carries $\theta_y$ and the Z‑axis carries $\theta_r$; do not expect them to match the aircraft roll/pitch/yaw convention from Section 7.2.)

The singularity is when the **middle** angle $\theta_r = \pm\pi/2$ (±90°). Here is why, step by step:

1. There is a general identity: rotating about y and then about z by 90° equals rotating about z by 90° and then about x by the same angle:
   $$ R_y(\theta)\, R_z(\tfrac{\pi}{2}) = R_z(\tfrac{\pi}{2})\, R_x(\theta) $$
   Intuition: $R_z(90°)$ swaps the x‑axis onto the y‑axis, so "rotate about y" on one side of it is the same as "rotate about x" on the other side.
2. Substitute $\theta_r = \pi/2$ into the Apollo formula and use the identity:
   $$ {}^S R_B = R_y(\theta_p)\, R_z(\tfrac{\pi}{2})\, R_x(\theta_y) = R_z(\tfrac{\pi}{2})\, R_x(\theta_p)\, R_x(\theta_y) $$
3. Two rotations about the same axis just add:
   $$ {}^S R_B = R_z(\tfrac{\pi}{2})\, R_x(\theta_p + \theta_y) $$

Only the **sum** $\theta_p + \theta_y$ matters now; the two angles are indistinguishable, and there is **no way to rotate about y** anymore ("singularity, no rotation around y"). On Apollo 13 the crew had to avoid attitudes near this configuration, because in gimbal lock the guidance platform loses track of orientation.

**Everyday analogy:** imagine a camera tripod head where you can tilt up/down and pan left/right. Point the camera straight up at the ceiling (tilt = 90°). Now "pan" and "roll the camera about its lens" do exactly the same thing: spin the picture. You have lost a direction of control. That is gimbal lock.

**Common mistake:** believing that gimbal lock is a mechanical problem that software avoids. Software that uses three angles has exactly the same problem: near the singularity, small changes of orientation require huge, jumpy changes of the angles. That is the motivation for the next topic.

---

## 9. Quaternions

### 9.1 What a quaternion is

A **quaternion** is a number with one real (scalar) part and three imaginary parts:

$$ q = s + \mathbf{v} = s + v_1\mathbf{i} + v_2\mathbf{j} + v_3\mathbf{k}, \qquad \mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = \mathbf{ijk} = -1 $$

The compact notation is $q = s\langle v_1, v_2, v_3\rangle$: a scalar $s$ and a 3‑vector $\mathbf{v}$. Think of it as a generalisation of complex numbers (which have one imaginary unit) to three imaginary units. The rules $\mathbf{i}^2 = \mathbf{j}^2 = \mathbf{k}^2 = -1$ are just like $i^2 = -1$ for complex numbers; the extra rule $\mathbf{ijk} = -1$ fixes how the three units multiply with each other.

### 9.2 Properties (slide 34)

| Property | Formula | In words |
|---|---|---|
| Norm (squared) | $\lvert q\rvert^2 = s^2 + v_1^2 + v_2^2 + v_3^2$ | "Length" of the quaternion, like Pythagoras with four terms. |
| Conjugate | $\bar q = s - v_1\mathbf{i} - v_2\mathbf{j} - v_3\mathbf{k}$ | Flip the sign of the vector part, keep the scalar. |
| Inverse | $q^{-1} = \bar q / \lvert q\rvert^2$ | Conjugate divided by the squared norm. |
| Unit quaternion | $\lvert q\rvert = 1$ | Norm equal to one. |
| Inverse of a unit quaternion | $q^{-1} = \bar q$ (and $\lvert q^{-1}\rvert = 1$) | For unit quaternions the inverse *is* the conjugate — no division needed. |

**Worked example.** $q = 1 + 2\mathbf{i}$ (so $s = 1$, $\mathbf v = (2,0,0)$). Norm squared: $1 + 4 = 5$. Conjugate: $1 - 2\mathbf{i}$. Inverse: $(1 - 2\mathbf{i})/5 = 0.2 - 0.4\mathbf{i}$. It is not a unit quaternion (norm $\sqrt5 \ne 1$), so it does not represent a rotation.

**Multiplication (Hamilton product).** For $q_1 = s_1\langle\mathbf v_1\rangle$ and $q_2 = s_2\langle\mathbf v_2\rangle$:

$$ q_1 q_2 = (s_1 s_2 - \mathbf v_1\cdot\mathbf v_2)\ \big\langle\, s_1\mathbf v_2 + s_2\mathbf v_1 + \mathbf v_1\times\mathbf v_2 \,\big\rangle $$

The scalar part is "scalars multiplied minus the dot product of the vectors"; the vector part is "each scalar times the other vector, plus the cross product". Because of the cross product, **quaternion multiplication is not commutative**, exactly like rotations. (The slide also shows this product as a 4×4 matrix acting on the second quaternion's four components; it is the same formula written out.)

⚠️ The slide writes the scalar part as "$s_1s_2 - v_1v_2$"; the product $v_1 v_2$ there means the **dot product** $\mathbf v_1\cdot\mathbf v_2$.

### 9.3 Unit quaternions represent rotations (slide 35)

A rotation by angle $\theta$ about a unit axis $\hat{\mathbf v} = (v_1, v_2, v_3)$ is the unit quaternion

$$ \tilde q = \cos\tfrac{\theta}{2} + (v_1\mathbf i + v_2\mathbf j + v_3\mathbf k)\sin\tfrac{\theta}{2} $$

Note the **half angle**. A $90°$ rotation about z is $\cos45° + \mathbf k\sin45° = 0.7071 + 0.7071\mathbf k$. The norm is $0.7071^2 + 0.7071^2 = 1$, as it must be.

**Rotating a vector.** Write the vector as a *pure* quaternion (zero scalar part): $p = p_x\mathbf i + p_y\mathbf j + p_z\mathbf k$. Then the rotated vector is

$$ p' = \tilde q\, p\, \tilde q^{-1} $$

"Sandwich" the vector between the quaternion and its inverse.

**Worked example.** Rotate $p = (1, 0, 0) = \mathbf i$ by $90°$ about z using $\tilde q = 0.7071\langle 0, 0, 0.7071\rangle$ and $\tilde q^{-1} = \bar{\tilde q} = 0.7071\langle 0, 0, -0.7071\rangle$.

1. First product $\tilde q\, p$ with $s_1 = 0.7071$, $\mathbf v_1 = (0,0,0.7071)$, $s_2 = 0$, $\mathbf v_2 = (1,0,0)$:
   - scalar: $0.7071\cdot 0 - (0,0,0.7071)\cdot(1,0,0) = 0 - 0 = 0$
   - vector: $0.7071\,(1,0,0) + 0 + (0,0,0.7071)\times(1,0,0) = (0.7071, 0, 0) + (0, 0.7071, 0) = (0.7071, 0.7071, 0)$
   - so $\tilde q p = 0\langle 0.7071, 0.7071, 0\rangle$.
2. Second product $(\tilde q p)\,\tilde q^{-1}$ with $s_1 = 0$, $\mathbf v_1 = (0.7071, 0.7071, 0)$, $s_2 = 0.7071$, $\mathbf v_2 = (0, 0, -0.7071)$:
   - scalar: $0 - \mathbf v_1\cdot\mathbf v_2 = 0 - 0 = 0$
   - vector: $0 + 0.7071\,(0.7071, 0.7071, 0) + (0.7071, 0.7071, 0)\times(0,0,-0.7071) = (0.5, 0.5, 0) + (-0.5, 0.5, 0) = (0, 1, 0)$.

Result: $p' = (0, 1, 0)$. The x‑axis went to the y‑axis, the same answer as $R_z(90°)$ gave in Section 6.2. 

**Composing two rotations.** If you rotate first by $\tilde q_1$ and then by $\tilde q_2$, the total rotation is

$$ k = \tilde q_2 \cdot \tilde q_1 $$

Notice the order: the *first* rotation is written on the *right*, just like matrices ($R_2 R_1$). **Inverse rotation:** $\ominus\tilde q = \tilde q^{-1} = s\langle -\mathbf u\rangle$, i.e. the conjugate, same axis, opposite angle.

### 9.4 Why bother with quaternions?

- Only four numbers (versus nine for a matrix) and they are easy to keep normalised.
- **No gimbal lock.** Unit quaternions have no singular configuration, which is why flight controllers and game engines use them.
- Smooth interpolation between orientations is easy (this comes up in Lecture 3).

The price is that they are less intuitive to read than three angles. In practice you often convert: angles for humans, quaternions for computation.

**Common mistake:** forgetting the half angle, or forgetting to normalise. A quaternion with norm $\ne 1$ will scale your vectors as well as rotate them.

---

## 10. MATLAB time: the Robotics Toolbox commands

The lecture demonstrated the ideas live with Peter Corke's Robotics Toolbox. Here is what each command does, in the order used in `Matlab_Time_Lecture_1.m`. (Page numbers refer to the Corke textbook, 2nd edition.)

### 10.1 2D rotation matrices (book p. 24–25)

```matlab
R = rot2(0.2)        % 2x2 rotation matrix for 0.2 rad (angle in radians!)
det(R)               % = 1, property of a rotation matrix
det(R*R)             % still 1: the product of two rotations is a rotation
syms theta
R = rot2(theta)      % symbolic version, shows cos/sin layout
simplify(R*R)        % R(theta)*R(theta) = R(2*theta)
simplify(det(R))     % = 1 symbolically (cos^2 + sin^2)
```

`rot2(θ)` builds the 2×2 matrix from Section 5.1. The `syms` lines show the toolbox works with symbolic algebra too, so you can *prove* the determinant is one rather than just checking numbers.

### 10.2 The skew‑symmetric (logarithm) view of rotation

```matlab
R = rot2(0.3)
S = logm(R)          % matrix logarithm: a 2x2 skew-symmetric matrix with +/-0.3
vex(S)               % extract the single unique element: 0.3 (the angle!)
expm(S)              % matrix exponential: gets R back
R = expm(skew(0.3))  % same as rot2(0.3)
```

A **skew‑symmetric matrix** is one whose transpose is its negative: $S^T = -S$. For 2×2 that forces zeros on the diagonal and $\pm$ the same number off‑diagonal, so it holds just *one* number. The surprising fact is that the matrix logarithm of a rotation matrix is skew‑symmetric and the number inside is the rotation angle. `skew(θ)` builds that matrix and `vex` ("vector extract") pulls the number back out; `expm` and `logm` are the matrix exponential and logarithm, which undo each other. You will meet this again in Lecture 3, where the same skew matrix describes angular velocity.

### 10.3 2D homogeneous transformations and composition (p. 27–29)

```matlab
T1 = transl2(1, 2) * trot2(30, 'deg')   % translate (1,2) then rotate 30 deg
plotvol([0 5 0 5]);                      % set up a plotting area
trplot2(T1, 'frame', '1', 'color', 'b') % draw frame {1}
T2 = transl2(2, 1)                       % pure translation (2,1)
trplot2(T2, 'frame', '2', 'color', 'r');
T3 = T1*T2                               % compose: {3} = (2,1) step taken in frame {1}
trplot2(T3, 'frame', '3', 'color', 'g');
T4 = T2*T1;                              % other order: a different frame!
trplot2(T4, 'frame', '4', 'color', 'c');
```

- `transl2(x, y)` makes a 3×3 homogeneous matrix with translation only (rotation = identity).
- `trot2(θ)` makes one with rotation only (translation zero). `'deg'` lets you give degrees.
- `*` is the $\oplus$ operator. `T3` has translation $(2.232, 3.866)$ and `T4` has $(3, 3)$, exactly the worked example in Section 5.4. The book's comment is worth quoting: "our final displacement is not (3, 3) because the displacement is with respect to the rotated coordinate frame".
- `trplot2` draws a frame; `plotvol` sets the axes.

```matlab
P = [3 ; 2 ];                              % a point in the world frame
plot_point(P, 'label', 'P', 'solid', 'ko');
P1 = inv(T1) * [P; 1]                      % P in frame {1}: [1.7321; -1.0000; 1]
h2e( inv(T1) * e2h(P) )                    % same thing with helper functions
```

`e2h` ("Euclidean to homogeneous") appends the 1; `h2e` removes it. `inv(T1)` is $\ominus\xi_1$. The result $(1.732, -1.0)$ matches Section 5.4.

### 10.4 Rotating about an arbitrary point

```matlab
T0 = eye(3,3);  trplot2(T0, 'frame', '0');       % world frame
X = transl2(2, 3);  trplot2(X, 'frame', 'X');    % a frame at (2,3)
R = trot2(2);                                     % rotation of 2 rad (~115 deg)
trplot2(R*X, 'framelabel', 'RX', 'color', 'r');  % rotates {X} about the WORLD origin
trplot2(X*R, 'framelabel', 'XR', 'color', 'r');  % rotates {X} about ITS OWN origin
```

This is non‑commutativity again, now with a geometric reading: **pre‑multiplying** by a rotation ($R X$) rotates the frame about the world origin and swings its position around; **post‑multiplying** ($X R$) spins the frame in place about its own origin.

```matlab
C = [1 2]';                           % a chosen pivot point
RC = transl2(C) * R * transl2(-C)     % rotate about point C
trplot2(RC*X, 'framelabel', 'XC', 'color', 'r');
```

To rotate about any point C: move C to the origin (`transl2(-C)`), rotate (`R`), move back (`transl2(C)`). Read the product right‑to‑left to see the order of operations. This "move, do, move back" pattern appears everywhere in robotics.

### 10.5 3D rotations (p. 34–37)

```matlab
R = rotx(pi/2)                   % 3x3 rotation of 90 deg about x
trplot(R)                        % draw the 3D frame
tranimate(R)                     % animate the rotation from identity to R
R = rotx(pi/2) * roty(pi/2)      % x first, then y (reading right to left in body terms)
roty(pi/2)*rotx(pi/2)            % different matrix: non-commutative
```

⚠️ The comment in the .m file says `rotx(pi/2)` is "around x axis for 45 degrees". It is **90 degrees** ($\pi/2$ rad = 90°). Trust the code, not the comment.

### 10.6 Euler angles and the singularity

```matlab
R = rotz(0.1) * roty(0.2) * rotz(0.3)   % ZYZ Euler sequence by hand
R = eul2r(0.1, 0.2, 0.3)                % same thing, compact
gamma = tr2eul(R)                       % inverse: angles from the matrix -> [0.1 0.2 0.3]
R = eul2r(0.1, -0.2, 0.3)               % negative middle angle
tr2eul(R)                               % returns a POSITIVE theta and different phi, psi
eul2r(ans)                              % ...but the SAME rotation matrix
R = eul2r(0.1, 0, 0.3)                  % theta = 0: the singular case
tr2eul(R)                               % returns [0 0 0.4]
```

Two lessons. First, the mapping from a matrix to Euler angles is **not unique**: $(0.1, -0.2, 0.3)$ and the toolbox's answer with positive $\theta$ are two different angle sets for the same orientation; the toolbox always picks $\theta \ge 0$. Second, at $\theta = 0$ only the sum $\varphi + \psi$ can be recovered (here $0.4$), and the toolbox sets $\varphi = 0$ by convention. This is the gimbal‑lock example from Section 8.2 live.

### 10.7 Roll‑pitch‑yaw (p. 38)

```matlab
R = rpy2r(0.1, 0.2, 0.3)     % roll 0.1, pitch 0.2, yaw 0.3; ZYX is the toolbox default
gamma = tr2rpy(R)            % inverse: recover the angles
tripleangle_App              % interactive tool to play with angle sequences
```

The live script adds the context: a *gripper* (z forward, x up/down) uses the XYZ sequence; a *vehicle* (x forward, z up/down) uses ZYX, which is the default. Roll‑pitch‑yaw has its singularity at pitch $= \pm90°$, which is "fortunately outside the range of feasible attitudes for most vehicles" (a car or a TurtleBot is never pitched straight up).

### 10.8 Quaternions (p. 45)

```matlab
q = UnitQuaternion( rpy2tr(0.1, 0.2, 0.3) )  % build a unit quaternion from a rotation
q = q * q          % quaternion multiplication = composing the rotation with itself
inv(q)             % inverse (= conjugate for a unit quaternion)
q*inv(q)           % identity quaternion 1 < 0, 0, 0 >
q/q                % same: a null rotation
q.R                % convert back to a 3x3 rotation matrix
q.plot()           % draw the orientation
q*[1 0 0]'         % rotate a vector (the q p q^-1 sandwich, done for you)
```

`UnitQuaternion` is the toolbox class; the constructor accepts a rotation matrix or homogeneous transform (`rpy2tr` makes a 4×4 transform from roll‑pitch‑yaw). `*` between two quaternions is the Hamilton product; `*` between a quaternion and a 3‑vector performs the rotation.

### 10.9 3D homogeneous transformations (p. 47)

```matlab
T = transl(1, 0, 0) * trotx(pi/2) * transl(0, 1, 0)
trplot(T)
t2r(T)       % pull out the 3x3 rotation block
transl(T)    % pull out the translation column
```

`transl(x,y,z)` and `trotx/troty/trotz` are the 3D versions of `transl2`/`trot2`. Read the product right to left for the body: move 1 along y, rotate 90° about x, move 1 along x. Because the middle rotation turns the original y‑axis onto z, the final translation is $(1, 0, 1)$: one unit along the original x and one along the original z. `t2r` and `transl` (with a matrix argument) split $T$ back into $R$ and $t$.

---

## In a nutshell

A robot is a goal‑oriented machine that senses, plans and acts on its own, and every part of its software needs to know *where things are*, so we attach coordinate frames to everything and describe each frame's **pose** (position plus orientation) relative to another frame with the symbol $\xi$. Relative poses can be chained with $\oplus$ and undone with $\ominus$, following rules that look like addition except that **order matters**. In 2D a pose becomes a 3×3 **homogeneous transformation matrix** $T$ containing a rotation matrix $R$ (orthogonal, determinant 1, inverse equals transpose) and a translation $t$; in 3D it is a 4×4 matrix built the same way, and $\oplus$ becomes matrix multiplication with $T_1T_2 = [R_1R_2,\ t_1 + R_1t_2]$ and $T^{-1} = [R^T,\ -R^Tt]$. Orientation in 3D can be written with three angles (ZYZ Euler angles for wrists, ZYX roll‑pitch‑yaw for vehicles) but every three‑angle scheme suffers **gimbal lock** at some configuration, where two angles become indistinguishable. **Unit quaternions** (four numbers, half‑angle formula, rotate a vector with $q\,p\,q^{-1}$) avoid that problem and are what most real flight controllers use.

## Key terms

- **Robot** – a goal‑oriented machine that can sense, plan and act autonomously.
- **UAV / RPAV** – Unmanned Aerial Vehicle (autonomous aerial robot) / Remotely Piloted Aerial Vehicle (a human‑flown drone).
- **Coordinate frame {A}** – a set of perpendicular axes with an origin, used as a reference for measuring positions.
- **Pose** – position and orientation together.
- **Relative pose ${}^A\xi_B$** – the pose of frame B as seen from frame A.
- **$\oplus$ (compounding)** – chaining two relative poses: ${}^A\xi_B \oplus {}^B\xi_C = {}^A\xi_C$.
- **$\ominus$ (inverse)** – the reverse pose: $\ominus{}^A\xi_B = {}^B\xi_A$.
- **Non‑commutative** – the order of operations matters: $\xi_1\oplus\xi_2 \ne \xi_2\oplus\xi_1$.
- **Pose graph** – a diagram with frames as nodes and relative poses as edges; different routes between two nodes must agree.
- **Rotation matrix R** – an orthogonal matrix with determinant +1 whose columns are the new frame's axes; $R^{-1} = R^T$.
- **Orthogonal matrix** – columns are mutually perpendicular unit vectors; transpose equals inverse.
- **Homogeneous coordinates** – a vector with an extra 1 appended so that translation can be done by matrix multiplication.
- **Homogeneous transformation T** – the (n+1)×(n+1) matrix $[R\ t; 0\ 1]$ that represents a pose.
- **SE(2) / SE(3)** – the sets of all rigid‑body transformations in 2D / 3D.
- **Skew‑symmetric matrix** – a matrix with $S^T = -S$; the matrix logarithm of a rotation.
- **Euler angles (ZYZ)** – three angles $(\varphi, \theta, \psi)$ with $R = R_z(\varphi)R_y(\theta)R_z(\psi)$; singular at $\theta = 0$.
- **Roll, pitch, yaw** – rotations about the forward, sideways and vertical body axes; ZYX sequence $R = R_z(\theta_y)R_y(\theta_p)R_x(\theta_r)$, singular at pitch $= \pm90°$.
- **NED frame** – a world frame with x North, y East, z Down, common in aerial robotics.
- **Gimbal lock** – the singular configuration of a three‑angle representation where two rotation axes align and one degree of freedom is lost.
- **Quaternion** – a number $s + v_1\mathbf i + v_2\mathbf j + v_3\mathbf k$ with one scalar and three imaginary parts.
- **Unit quaternion** – a quaternion of norm 1; represents a rotation as $\cos\frac{\theta}{2} + \hat{\mathbf v}\sin\frac{\theta}{2}$.
- **Conjugate $\bar q$** – same scalar, negated vector part; equals the inverse for a unit quaternion.
- **Hamilton product** – quaternion multiplication; composes rotations and is non‑commutative.

## Check yourself

1. A robot's camera frame {C} is mounted on the robot {R} with known offset ${}^R\xi_C$, and the camera sees an object at ${}^C\xi_B$. Write the object's pose relative to the robot, and then relative to the world if the robot's world pose is $\xi_R$.
2. Frame {B} is at $(0, 2)$ in frame {A} and rotated by $90°$. A point is at ${}^Bp = (1, 0)$. Write ${}^AT_B$ and compute ${}^Ap$. Then write ${}^AT_B^{-1}$.
3. Give the three properties of a rotation matrix and explain in one sentence why "inverse equals transpose" is useful.
4. Explain, using the ZYZ example with $\theta = 0$, why gimbal lock makes it impossible to recover unique Euler angles from a rotation matrix.
5. Write the unit quaternion for a rotation of $180°$ about the x‑axis, and use $p' = q p q^{-1}$ to show what happens to the vector $(0, 1, 0)$.

<details>
<summary>Answers</summary>

1. Relative to the robot: ${}^R\xi_B = {}^R\xi_C \oplus {}^C\xi_B$ (the inner C's cancel). Relative to the world: $\xi_B = \xi_R \oplus {}^R\xi_C \oplus {}^C\xi_B$.

2. With $\theta = 90°$, $\cos = 0$, $\sin = 1$, $t = (0, 2)$:
   $${}^AT_B = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 2 \\ 0 & 0 & 1\end{bmatrix}$$
   ${}^Ap$: $x = 0\cdot1 - 1\cdot0 + 0 = 0$; $y = 1\cdot1 + 0\cdot0 + 2 = 3$. So ${}^Ap = (0, 3)$. (B's x‑axis points along A's y‑axis, so one step along B's x from $(0,2)$ lands at $(0,3)$.)
   Inverse: $R^T = \begin{bmatrix} 0 & 1 \\ -1 & 0\end{bmatrix}$, $R^Tt = (0\cdot0 + 1\cdot2,\ -1\cdot0 + 0\cdot2) = (2, 0)$, so $-R^Tt = (-2, 0)$:
   $${}^AT_B^{-1} = \begin{bmatrix} 0 & 1 & -2 \\ -1 & 0 & 0 \\ 0 & 0 & 1\end{bmatrix}$$
   Check: applying it to $(0, 3)$ gives $x = 3 - 2 = 1$, $y = 0$, i.e. $(1, 0)$. ✔

3. (a) It is orthogonal (columns are perpendicular unit vectors); (b) its determinant is +1; (c) its inverse equals its transpose, $R^{-1} = R^T$, and $R(-\theta) = R(\theta)^T$. Usefulness: to transform coordinates in the opposite direction (from A to B instead of B to A) you just flip rows and columns, with no matrix inversion needed, which is faster and numerically safer.

4. In ZYZ, $R = R_z(\varphi)R_y(\theta)R_z(\psi)$. When $\theta = 0$, $R_y(0) = I$, so $R = R_z(\varphi)R_z(\psi) = R_z(\varphi + \psi)$. The matrix depends only on the sum $\varphi + \psi$, so any pair of angles with the same sum (e.g. $(0.1, 0, 0.3)$ and $(0, 0, 0.4)$) gives the identical matrix. Going backwards from the matrix you can find the sum but not the individual angles; the toolbox picks $\varphi = 0$ by convention. The first and third rotation axes have lined up, which is gimbal lock.

5. Half angle $90°$: $q = \cos90° + \mathbf i\sin90° = 0 + 1\mathbf i = 0\langle 1, 0, 0\rangle$, and $q^{-1} = \bar q = 0\langle -1, 0, 0\rangle$. Vector $p = \mathbf j = 0\langle 0,1,0\rangle$.
   First, $q p$: scalar $= 0\cdot0 - (1,0,0)\cdot(0,1,0) = 0$; vector $= 0 + 0 + (1,0,0)\times(0,1,0) = (0,0,1)$. So $qp = 0\langle 0,0,1\rangle$ ($= \mathbf k$).
   Then $(qp)\,q^{-1}$: scalar $= 0 - (0,0,1)\cdot(-1,0,0) = 0$; vector $= 0 + 0 + (0,0,1)\times(-1,0,0) = (0\cdot0 - 1\cdot0,\ 1\cdot(-1) - 0\cdot0,\ 0) = (0, -1, 0)$.
   Result $p' = (0, -1, 0)$: the y‑axis is flipped to $-y$, which is exactly what a 180° rotation about x does. ✔

</details>
