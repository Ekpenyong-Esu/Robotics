# Lecture 6 – Aerial Robotics (Study Notes)

*Course R7010E – Robotics, Luleå University of Technology. Based on the slide deck `R7010E_Lecture_6_Aerial_Robotics.pdf` (34 slides), plus a short link to Lab 3 (Crazyflie).*

---

This lecture is about flying robots, and in particular the quadrotor (a drone with four rotors). You will learn how a quadrotor makes itself go up, tilt, and spin by changing the speed of its four motors, how physicists write that down as equations, and how a controller uses those equations to make the drone hover and fly to a point. This matters because the same ideas (model the physics, then stack simple controllers on top) are used on every commercial drone, and you will need them directly in Lab 3 when you fly the Crazyflie 2.

---

## Roadmap: the main topics in this lecture

1. Kinds of aerial robots (fixed-wing, rotorcraft, hybrids, futuristic designs).
2. The quadrotor: roll, pitch, yaw, and how speeding up or slowing down rotors creates each one.
3. The closed-loop robot architecture and its constraints (hard real time, limited computing, blocks in parallel).
4. States: what we need to know about the drone and which sensors tell us.
5. From rotor speed to force: thrust $T_i = b\,\varpi_i^2$ and drag torque $Q_i = k\,\varpi_i^2$.
6. Translational dynamics (Newton's second law for the drone).
7. Rotational dynamics (Euler's equation, pairwise differences cause rotation, $\omega \neq \dot\theta$).
8. The matrix form $(T, \tau) = A\,\varpi^2$ and its inverse, the control mixer.
9. The full state-space model from references [2], [3].
10. The simplified formulation when yaw is neglected and angles are small.
11. Why control is "the enabling factor", and Model Predictive Control (MPC) in plain language [5].
12. A simplified cascade control design: altitude, position, attitude, yaw, mixer.
13. Pros and cons of cascaded control, and the alternatives.
14. Hardware: where each controller runs and why the rates differ so much.
15. Coordinate conventions: NED versus ENU.
16. The GRASP testbed equations [4].
17. How this connects to Lab 3.
18. Further reading.

---

## 1. Kinds of aerial robots

Slide 2 shows a gallery. There are three big families.

**Rotorcraft** keep themselves in the air with spinning rotors. The quadrotor (four rotors) is the star of this lecture. Slide 2 also shows a hexacopter (six rotors) carrying sensors, and even a passenger-carrying multirotor.

**Fixed-wing** aircraft look like small aeroplanes. They are very efficient for long distances because the wing does the lifting, but they cannot hover and they need space to take off and land. Slide 2 shows a seaplane-style fixed wing and a "flying wing" that has two rotors on the wing (a hybrid that can take off vertically and then fly like a plane).

**Bio-inspired and micro robots** copy insects or birds. The tiny flapping-wing robot sitting on a fingertip in slide 2 is an example.

Slide 3 goes further into the future: a modular drone made of several small units that reshapes itself in flight to squeeze through a narrow horizontal gap ("aerial transformation"), and a hexagonal multi-module flying platform. The message is that the rigid quadrotor you study here is just the simplest member of a growing family.

Why does the course focus on the quadrotor? It is mechanically simple (four fixed motors, no moving control surfaces), cheap, can hover, and its mathematics is a clean example of rigid-body dynamics plus control. In the lab you fly a **Crazyflie 2**, a palm-sized quadrotor (shown in the workspace photo `Crazy_Fly_screenshot.png`).

---

## 2. Meet the quadrotor: roll, pitch and yaw

### 2.1 The three rotations and their names

Slide 4 uses fish to introduce the three ways a body can rotate, then shows them on a quadrotor frame.

- **Roll** is tipping sideways, like a fish rolling onto its side. It is a rotation about the body's forward axis (the $x$ axis, which points toward the "Front" of the drone).
- **Pitch** is tipping nose-up or nose-down. It is a rotation about the sideways axis ($y$).
- **Yaw** is turning left or right while staying level, like a fish turning to swim in a new direction. It is a rotation about the vertical axis ($z$).

In the slides the three angles are written $\phi$ (phi, roll), $\theta$ (theta, pitch) and $\psi$ (psi, yaw). You will see this triple everywhere in the lecture, so it is worth memorising the pairing now: $\phi$-roll, $\theta$-pitch, $\psi$-yaw.

### 2.2 Why two rotors spin one way and two spin the other

Slide 5 shows the four rotors numbered 1 (front), 2 (right), 3 (back) and 4 (left). Rotors 1 and 3 spin in one direction, and rotors 2 and 4 spin in the opposite direction. This is not a design choice for looks.

A spinning propeller pushes against the air, and the air pushes back with a twisting force (a **torque**, which is a force that tries to rotate something) on the drone body. One rotor alone would make the whole drone spin the opposite way, exactly like a helicopter without its tail rotor. If two rotors spin clockwise and two spin anticlockwise, and all four run at the same speed, their twisting effects cancel and the drone does not spin.

Everyday picture: imagine standing on a swivel chair and spinning a heavy bicycle wheel. The chair turns the other way. Hold two wheels spinning in opposite directions and the chair stays still.

### 2.3 The recipe: which rotors to speed up for each motion

Slide 5 gives eight pictures labelled (a) to (h). A bold arrow means "this rotor is spinning faster than normal". Here is the recipe in words. "Normal" means the hover speed where total lift just balances the drone's weight.

1. **Take-off / go up (c):** speed up all four rotors equally. More total lift than weight, so the drone rises. No tilting, no spinning, because everything stays balanced.
2. **Land / go down (g):** slow down all four equally.
3. **Roll (d) and (h):** speed up one of the side rotors (2 or 4) and slow down the other. One side gets more lift than the other, so the drone tips sideways. Total lift stays the same if you speed one up by as much as you slow the other down.
4. **Pitch (e) and (f):** speed up the front rotor (1) and slow the back rotor (3), or the reverse. The drone tips nose-down or nose-up.
5. **Yaw (a) and (b):** speed up the pair 1 and 3 and slow down the pair 2 and 4 (or the reverse). Total lift is unchanged and nothing tips, because the two speeded-up rotors sit opposite each other. But now the reaction torques no longer cancel, so the drone starts to spin about its vertical axis.

So a quadrotor has four motors and exactly four things it can control directly: total thrust, roll torque, pitch torque and yaw torque. It cannot, for example, slide sideways without tilting first. That is why in Section 12 position control works by asking for a tilt angle.

**Common mistake:** people often think yaw is made by "tilting". It is not. Yaw comes purely from the difference between the drag torques of the clockwise and anticlockwise pairs. Nothing tilts during a pure yaw.

Slide 6 is a video still of a real quadrotor demonstrating these motions in the lab.

---

## 3. The closed-loop architecture

Slide 7 shows the standard robot pipeline you met in Lecture 1, now for a drone:

**Mission Planner → Path Planning → Control Layer → Drone → Perception**, with Perception feeding back into both Path Planning and the Control Layer.

Three constraints are written on the slide, and all three matter for drones more than for ground robots.

- **Hard bounds, real-time system.** A ground robot that misses a control update just stops for a moment. A drone that misses attitude updates falls out of the sky. Deadlines must be met every time.
- **Limited computational resources.** Everything that flies must be light, and light means small batteries and small processors.
- **All the blocks are executed in parallel.** Perception, planning and control run at the same time at different rates (see Section 14), not one after another.

---

## 4. States and sensors

A **state** is a number you need to know to describe the drone's situation completely at this instant. Slide 8 lists what the drone needs to know and where it gets it.

- **Position of the quadcopter in the environment**, from GPS outdoors, or a motion-capture system indoors (in the lab, the Vicon cameras).
- **Inertial Measurement Unit (IMU)**: a chip containing accelerometers (measure acceleration), gyroscopes (measure rotation rate) and magnetometers (measure Earth's magnetic field, like a compass). The IMU is what tells the drone its attitude and angular rates.
- **Barometer** for altitude (air pressure drops as you go up).
- **GPS** for location.
- **Ultrasonic sensors** for distance to the ground or to obstacles.
- **Cameras** for perception.

Slide 9 defines the two coordinate frames and the two state vectors:

$$\mathbf{x} = (x, y, z)^T, \qquad \boldsymbol{\theta} = (\phi, \theta, \psi)^T$$

In words, the drone's **position** is three numbers in the **inertial frame** (a frame fixed to the world, also called the world frame), and its **attitude** (orientation) is three angles. The **body frame** is a frame glued to the drone itself, with $x$ forward, $y$ sideways and $z$ up through the body. Rotor thrust always points along the body $z$ axis, so when the body tilts, the thrust tilts with it. Keeping track of which quantity lives in which frame is the main source of confusion in this lecture, so each formula below says which frame it is in.

The full state actually has twelve numbers: three positions, three velocities, three angles and three angular rates. Slide 14 writes them out as $x_1 \ldots x_4$, and you will see them again in Section 9.

---

## 5. From rotor speed to force

Slide 10 gives the two formulas that connect motor speed to physics. Let $\varpi_i$ (a variant of the Greek letter pi, read "rotor speed of rotor $i$") be the angular speed of rotor $i$ in radians per second.

$$T_i = b\,\varpi_i^2, \qquad Q_i = k\,\varpi_i^2$$

- $T_i$ is the **thrust** (upward lift force) produced by rotor $i$, in newtons.
- $Q_i$ is the **drag torque** (the reaction twist from Section 2.2) of rotor $i$, in newton-metres.
- $b$ is the thrust coefficient and $k$ is the drag-torque coefficient. Both are constants that depend on the propeller shape, size and air density.

The important shape of these formulas is the **square**: double the rotor speed and you get four times the thrust. This also means the relationship is not linear, which is why control is done in terms of $\varpi_i^2$ rather than $\varpi_i$ (Section 8).

Total upward thrust is simply the sum:

$$T = \sum_i T_i = b\,(\varpi_1^2 + \varpi_2^2 + \varpi_3^2 + \varpi_4^2)$$

**Worked example (numbers from the quadrotor in Table 2 of slide 19).** Mass $m = 1.1\ \text{kg}$, thrust coefficient $b = 9.29 \times 10^{-5}\ \text{N s}^2$. To hover, total thrust must equal weight:

$$T = m g = 1.1 \times 9.81 \approx 10.79\ \text{N}$$

Each of the four rotors supplies a quarter, about $2.70\ \text{N}$. Then

$$\varpi_i^2 = \frac{2.70}{9.29 \times 10^{-5}} \approx 29{,}040 \quad\Rightarrow\quad \varpi_i \approx 170\ \text{rad/s}$$

which is about 1,630 revolutions per minute. Keep this hover speed in mind; the later examples perturb it.

⚠️ **Symbol clash:** on slides 10–12, $b$ is the thrust coefficient, $k$ is the drag coefficient and $d$ is the arm length. The table on slide 19 (copied from paper [5]) reuses the letters differently: there $b$ is still thrust, but $d$ is the drag coefficient and $l_a$ is the arm length. Always read which paper a formula comes from before plugging numbers in.

---

## 6. Translational dynamics (how the drone moves through space)

**Translation** means moving from place to place without considering rotation. The governing law is Newton's second law, "mass times acceleration equals the sum of forces". Slide 10 writes it in the world frame:

$$m\,\dot{\mathbf v} = \begin{pmatrix} 0 \\ 0 \\ m g \end{pmatrix} - {}^0\!\mathbf R_B \begin{pmatrix} 0 \\ 0 \\ T \end{pmatrix} - B\,\mathbf v$$

Let's take each piece.

- $m$ is the mass and $\dot{\mathbf v}$ is the acceleration vector (rate of change of velocity) in the world frame.
- The first term is **gravity**. It acts straight down, so it only has a $z$ component.
- ${}^0\!\mathbf R_B$ is the **rotation matrix** from the body frame $\{B\}$ to the world frame $\{0\}$ (Lecture 1). Thrust $T$ points along the body $z$ axis, so the vector $(0, 0, T)^T$ is written in the body frame, and multiplying by ${}^0\!\mathbf R_B$ expresses it in the world frame. This is the key idea of quadrotor flight: **tilt the body and the thrust gets a sideways component, which accelerates the drone sideways.**
- $B\,\mathbf v$ is **aerodynamic drag**, modelled as proportional to velocity. The faster you fly, the more the air pushes back.

Slide 10 follows Peter Corke's book and uses a frame where $z$ points **down** (the NED convention of Section 15). That is why gravity appears as $+mg$ and thrust appears with a minus sign: in a $z$-down frame "up" is negative.

Slide 14 writes the same law in a $z$-**up** frame:

$$\dot{\mathbf x}_2 = \begin{pmatrix} 0 \\ 0 \\ -g \end{pmatrix} + \frac{1}{m}\,\mathbf R\,\mathbf T_B + \frac{1}{m}\,\mathbf F_D$$

Here the signs are flipped (gravity negative, thrust positive) because now "up" is positive, and the drag is written as a general force $\mathbf F_D$.

⚠️ **Two sign conventions in one deck.** Slides 10 and 14 are both correct, but they use opposite $z$ directions, so the signs of gravity and thrust are swapped. On an exam, state which convention you are using before you write the equation. Mixing them is the single most common error.

**Worked example: tilting to move.** Suppose the drone is level and hovering ($T = mg$). Now pitch it nose-down by a small angle $\theta$. The thrust vector tilts forward by $\theta$. Its horizontal component is $T\sin\theta$ and its vertical component is $T\cos\theta$. For a small angle, $\sin\theta \approx \theta$ and $\cos\theta \approx 1$, so

$$m\ddot x \approx T\,\theta = m g\,\theta \quad\Rightarrow\quad \ddot x \approx g\,\theta$$

With $\theta = 0.1\ \text{rad}$ (about $6°$) the forward acceleration is about $0.98\ \text{m/s}^2$, and the drone loses only $1 - \cos(0.1) \approx 0.5\%$ of its vertical lift. This tiny result, "acceleration equals $g$ times the tilt angle", is the heart of the position controller in Section 12.

**Analogy:** think of a tray of drinks held flat on your palm. If you tilt the tray slightly forward, your upward push now has a small forward part and the tray slides forward. The quadrotor is a tray that can tilt itself.

---

## 7. Rotational dynamics (how the drone tips and spins)

### 7.1 Pairwise differences cause rotation

Slide 11 turns the recipe of Section 2.3 into formulas. The rotor layout: 1 at the front on $+x$, 3 at the back, 2 and 4 on the $y$ axis, each at distance $d$ (the **arm length**) from the centre.

**Rolling torque** (about $x$) comes from the difference between the two side rotors, each acting at lever arm $d$:

$$\tau_x = d\,T_4 - d\,T_2 = d\,b\,(\varpi_4^2 - \varpi_2^2)$$

**Pitching torque** (about $y$) comes from the front/back pair:

$$\tau_y = d\,b\,(\varpi_1^2 - \varpi_3^2)$$

**Yaw torque** (about $z$) comes from the drag torques $Q_i = k\varpi_i^2$, counted positive for one spin direction and negative for the other:

$$\tau_z = Q_1 - Q_2 + Q_3 - Q_4 = k\,(\varpi_1^2 + \varpi_3^2 - \varpi_2^2 - \varpi_4^2)$$

Notice that there is **no arm length $d$ in the yaw torque**, because drag torque acts around each rotor's own axis, not through a lever. That is why yaw authority is usually the weakest of the three.

**Worked example (same drone as before, $d = 0.21\ \text{m}$).** Start from hover with all $\varpi_i^2 \approx 29{,}040$. Speed rotor 4 up by 5% and slow rotor 2 down by 5%. Squaring, $\varpi_4^2 \approx 29{,}040 \times 1.1025 \approx 32{,}020$ and $\varpi_2^2 \approx 29{,}040 \times 0.9025 \approx 26{,}210$. Then

$$\tau_x = 0.21 \times 9.29\times10^{-5} \times (32{,}020 - 26{,}210) \approx 0.113\ \text{N m}$$

Total thrust barely changes (the two squares add to $58{,}230$ instead of $58{,}080$, a difference of $0.014\ \text{N}$), so the drone rolls without noticeably climbing. Now do the same 5% trick on the pairs for yaw, using $k = 1.1\times10^{-6}$ from Table 2:

$$\tau_z = 1.1\times10^{-6} \times (2\times 32{,}020 - 2\times 26{,}210) \approx 0.013\ \text{N m}$$

Nearly ten times smaller than the roll torque for the same speed change, which confirms the "weak yaw" remark.

### 7.2 Euler's equation

The boxed equation on slides 11 and 13 is the rotational version of Newton's law, written in the **body frame**:

$$\mathbf J\,\dot{\boldsymbol\omega} = -\boldsymbol\omega \times \mathbf J\boldsymbol\omega + \boldsymbol\tau$$

- $\boldsymbol\omega = (\omega_x, \omega_y, \omega_z)^T$ is the **angular velocity** of the body, in the body frame.
- $\dot{\boldsymbol\omega}$ is the angular acceleration.
- $\mathbf J$ (also written $\mathbf I$ on slide 13) is the **inertia matrix**, the rotational equivalent of mass. It says how hard the body is to spin about each axis. A quadrotor is roughly symmetric, so $\mathbf J$ is usually taken as diagonal, $\mathrm{diag}(I_{xx}, I_{yy}, I_{zz})$, with the off-diagonal "products of inertia" $I_{xy}$ etc. set to zero (slide 13 shows both the full and the diagonal version).
- $\boldsymbol\tau = (\tau_x, \tau_y, \tau_z)^T$ are the torques from Section 7.1.
- $-\boldsymbol\omega \times \mathbf J\boldsymbol\omega$ is the **gyroscopic term**. It says that a body already spinning about one axis resists being tipped about another. It is zero when the drone is not rotating, which is why it is often ignored near hover.

Rearranged, the equation becomes a formula for angular acceleration:

$$\dot{\boldsymbol\omega} = \mathbf I^{-1}\big(\boldsymbol\tau - \boldsymbol\omega \times (\mathbf I\boldsymbol\omega)\big)$$

**Worked example.** With the roll torque from Section 7.1, $\tau_x = 0.113\ \text{N m}$, and $I_{xx} = 0.0196\ \text{kg m}^2$, starting from rest (so the gyroscopic term is zero):

$$\dot\omega_x = \frac{0.113}{0.0196} \approx 5.8\ \text{rad/s}^2$$

After $0.1\ \text{s}$ the drone is rolling at about $0.58\ \text{rad/s}$ ($33°/\text{s}$). That is how a 5% change in two motor speeds becomes a brisk roll.

⚠️ **Sign of the gyroscopic terms on slide 13.** The slide expands the vector equation into components as
$\dot\omega_x = \tau_\phi / I_{xx} - \frac{I_{yy}-I_{zz}}{I_{xx}}\omega_y\omega_z$ (and similarly for the other two). If you expand the boxed equation yourself, the $x$ component of $\boldsymbol\omega\times(\mathbf I\boldsymbol\omega)$ is $(I_{zz}-I_{yy})\omega_y\omega_z$, so $\dot\omega_x = \tau_\phi/I_{xx} + \frac{I_{yy}-I_{zz}}{I_{xx}}\omega_y\omega_z$, with a **plus** sign. The same expansion with the minus sign appears in reference [3] (Gibiansky), which is where the slide takes it from. Trust the boxed vector equation, and check with your lecturer which form is expected on the exam.

### 7.3 Angular velocity is not the rate of change of the Euler angles

Slide 11 makes a point that trips up almost everyone: $\boldsymbol\omega \neq \dot{\boldsymbol\theta}$. The gyroscope measures $\boldsymbol\omega$, the spin rate about the body axes. But $\dot{\boldsymbol\theta} = (\dot\phi, \dot\theta, \dot\psi)^T$ is how fast the three Euler angles are changing, and Euler angles are defined as a sequence of rotations about axes that move with each other. The two are related by a matrix that depends on the current angles:

$$\boldsymbol\omega = \begin{bmatrix} 1 & 0 & -s_\theta \\ 0 & c_\phi & c_\theta s_\phi \\ 0 & -s_\phi & c_\theta c_\phi \end{bmatrix} \dot{\boldsymbol\theta}$$

where $s_\theta$ means $\sin\theta$ and $c_\phi$ means $\cos\phi$. When the drone is level ($\phi = \theta = 0$) the matrix becomes the identity and the two coincide, which is why near hover people often get away with treating them as equal. At larger tilt angles they differ noticeably, and at $\theta = 90°$ the matrix cannot be inverted (this is the gimbal lock you met in Lecture 1).

**Analogy:** on a spinning globe, "how fast the globe spins about its own axle" is one number ($\omega$). "How fast the latitude and longitude of a painted dot change" are different numbers that depend on where the dot is. Same motion, different bookkeeping.

---

## 8. Everything in one matrix: $A$ and the control mixer

Slide 12 collects total thrust and the three torques into a single matrix equation:

$$\begin{pmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{pmatrix} = \begin{pmatrix} -b & -b & -b & -b \\ 0 & -db & 0 & db \\ db & 0 & -db & 0 \\ k & -k & k & -k \end{pmatrix} \begin{pmatrix} \varpi_1^2 \\ \varpi_2^2 \\ \varpi_3^2 \\ \varpi_4^2 \end{pmatrix} = \mathbf A \begin{pmatrix} \varpi_1^2 \\ \varpi_2^2 \\ \varpi_3^2 \\ \varpi_4^2 \end{pmatrix}$$

Read it row by row and you will recognise Section 7.1.

1. Row 1: every rotor contributes $b\varpi_i^2$ to the thrust. The minus signs are there because slide 12 is in the $z$-down convention (thrust points to negative $z$). In a $z$-up convention this row is $+b$ everywhere.
2. Row 2: roll torque is $db(\varpi_4^2 - \varpi_2^2)$, so rotor 4 gets $+db$ and rotor 2 gets $-db$.
3. Row 3: pitch torque is $db(\varpi_1^2 - \varpi_3^2)$.
4. Row 4: yaw torque is $k(\varpi_1^2 - \varpi_2^2 + \varpi_3^2 - \varpi_4^2)$.

Now the question on the slide: "From a control point of view?" A controller decides what thrust and torques it wants. It does not think in motor speeds. So we need to go the other way, and because $\mathbf A$ is a square 4×4 matrix with non-zero determinant, we can invert it:

$$\begin{pmatrix} \varpi_1^2 \\ \varpi_2^2 \\ \varpi_3^2 \\ \varpi_4^2 \end{pmatrix} = \mathbf A^{-1} \begin{pmatrix} T \\ \tau_x \\ \tau_y \\ \tau_z \end{pmatrix}$$

This inverse is called the **control mixer** (slide 25). It takes the four "wishes" (one thrust, three torques) and mixes them into four motor speeds. Every flight controller on the market has a mixer, and for a plus-shaped quadrotor it is exactly this matrix.

**Worked example (hover).** Ask for $T = 10.79\ \text{N}$ and all torques zero. Row 1 of the inverse spreads thrust evenly, rows 2–4 add nothing, so every $\varpi_i^2 = T/(4b) \approx 29{,}040$, matching Section 5. Now additionally ask for $\tau_x = 0.113\ \text{N m}$: the mixer adds $\tau_x/(2db) \approx 2{,}900$ to $\varpi_4^2$ and subtracts the same from $\varpi_2^2$, which is the 5% up/down change we used before. You can check: $(32{,}020 - 26{,}210)\,db = 0.113$.

**Common mistake:** the mixer outputs the **squares** of the rotor speeds. You still have to take the square root to get a motor command, and if the requested torques are large compared with the thrust, one of the squares can come out negative, which is physically impossible (a rotor cannot produce downward thrust). Real mixers clip the commands when this happens.

---

## 9. The full state-space model (slide 14)

Slide 14 gathers everything into one standard "state-space" form, meaning a list of state vectors and a formula for the rate of change of each. It groups the twelve states into four 3-vectors:

$$\mathbf x_1 = \begin{bmatrix} x \\ y \\ z \end{bmatrix},\quad \mathbf x_2 = \begin{bmatrix} \dot x \\ \dot y \\ \dot z \end{bmatrix},\quad \mathbf x_3 = \begin{bmatrix} \varphi \\ \theta \\ \psi \end{bmatrix},\quad \mathbf x_4 = \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}$$

and then writes four equations:

1. $\dot{\mathbf x}_1 = \mathbf x_2$. Position changes at the rate given by velocity. (Pure definition.)
2. $\dot{\mathbf x}_2 = (0, 0, -g)^T + \frac{1}{m}\mathbf R\,\mathbf T_B + \frac{1}{m}\mathbf F_D$. Velocity changes because of gravity, rotated thrust and drag. (Section 6, $z$-up.)
3. $\dot{\mathbf x}_3 = \begin{bmatrix} 1 & 0 & -s_\theta \\ 0 & c_\phi & c_\theta s_\phi \\ 0 & -s_\phi & c_\theta c_\phi \end{bmatrix}^{-1} \mathbf x_4$. Euler angles change at a rate given by the body angular velocity through the inverse of the matrix in Section 7.3.
4. $\dot{\mathbf x}_4 = \begin{bmatrix} \tau_\phi I_{xx}^{-1} \\ \tau_\theta I_{yy}^{-1} \\ \tau_\psi I_{zz}^{-1} \end{bmatrix} - \begin{bmatrix} \frac{I_{yy}-I_{zz}}{I_{xx}}\omega_y\omega_z \\ \frac{I_{zz}-I_{xx}}{I_{yy}}\omega_x\omega_z \\ \frac{I_{xx}-I_{yy}}{I_{zz}}\omega_x\omega_y \end{bmatrix}$. Angular velocity changes because of applied torque and the gyroscopic term (Section 7.2, same ⚠️ sign caveat).

The inputs to this whole system are $T$ and $\boldsymbol\tau$, which the mixer turns into four motor speeds. This is "the model" that any model-based controller, including MPC, uses to predict what the drone will do next.

---

## 10. The simplified formulation (slide 15)

Twelve coupled nonlinear equations are too much for a first controller, so slide 15 gives a trimmed version. First, the translational equations written out with explicit trigonometry (this is what you get when you multiply out $\mathbf R\,(0,0,T)^T$ for a ZYX rotation):

$$m\ddot x = T(\sin\phi\sin\psi + \cos\phi\cos\psi\sin\theta)$$
$$m\ddot y = T(\cos\phi\sin\theta\sin\psi - \cos\psi\sin\phi)$$
$$m\ddot z = T\cos\theta\cos\phi - mg$$

and the rotational part reduced to "angular acceleration equals a transformed torque":

$$\ddot\psi = \tilde\tau_\psi,\quad \ddot\theta = \tilde\tau_\theta,\quad \ddot\phi = \tilde\tau_\phi, \qquad \tilde{\boldsymbol\tau} = \mathbb J^{-1}\big(\boldsymbol\tau - C(\boldsymbol\eta, \dot{\boldsymbol\eta})\dot{\boldsymbol\eta}\big)$$

The tilde (~) means "the torque after the inertia and the gyroscopic/Coriolis effects $C(\boldsymbol\eta,\dot{\boldsymbol\eta})$ have been absorbed", so that what is left looks like three independent double integrators (torque in, angle out, twice integrated). $\boldsymbol\eta$ is just another name for the Euler-angle vector.

The slide then says: **"Yaw hard to estimate, if neglected then the equations take the form"** and crosses out some terms. If you set $\psi = 0$ (so $\sin\psi = 0$, $\cos\psi = 1$), you get

$$m\ddot x = T\cos\phi\sin\theta, \qquad m\ddot y = -T\sin\phi, \qquad m\ddot z = T\cos\theta\cos\phi - mg$$

and for small angles near hover ($T \approx mg$):

$$\ddot x \approx g\,\theta, \qquad \ddot y \approx -g\,\phi, \qquad \ddot z \approx \frac{T}{m} - g$$

This is the model the cascade controller in Section 12 is built on: **pitch moves you in $x$, roll moves you in $y$ (with a minus sign), thrust moves you in $z$, and each is almost independent of the others.**

⚠️ **The red markings on slide 15 do not match the text.** The text says yaw ($\psi$) is neglected, but the terms that are crossed out and the "1"s written in are the ones you would remove by setting the *roll* angle $\phi = 0$ (the slide crosses out $\sin\phi\sin\psi$, $\cos\psi\sin\phi$ and the whole $\ddot\phi = \tilde\tau_\phi$ line, and writes 1 over $\cos\phi$). The notes above follow the written text ($\psi = 0$), because that is the assumption the later cascade actually uses. Check with your lecturer which simplification is intended.

**Why is yaw "hard to estimate"?** Roll and pitch can be found from the accelerometer, because gravity tells you which way is down. Yaw cannot: spinning about the vertical axis does not change where "down" is. You need a magnetometer (disturbed by motors and buildings) or external positioning. That is why yaw is handled by its own separate loop in Section 12.5.

---

## 11. Control is the enabling factor, and MPC in plain words

Slide 16 is a video still from the lab: without a controller a quadrotor is unflyable, because it is unstable (a small tilt produces sideways acceleration, which does nothing to undo the tilt). Control is what turns four motors and a frame into a useful robot.

Slides 17–19 show one advanced option, **Model Predictive Control (MPC)**, from reference [5]. You are not expected to reproduce the matrices, but you should understand the idea.

1. **Have a model.** Slide 17 shows the 12-state nonlinear model of Section 9 (with inputs $U_1 \ldots U_4$, which are the thrust and three torques, and $\Omega_r$, the net rotor speed that creates a small gyroscopic effect) being linearised at the current operating point into $\dot{\mathbf x} = \mathbf A_j\mathbf x + \mathbf B_j\,\delta\mathbf u + \delta\tilde{\mathbf w}_N$. The $j$ means the matrices are re-computed for the current attitude, and $\tilde{\mathbf w}$ represents disturbances such as wind.
2. **Predict.** At every control step, use the model to predict where the drone will be over a short future window (the **prediction horizon** $N_p$) for a candidate sequence of inputs.
3. **Optimise.** Choose the input sequence that minimises a cost $J(k)$ (slide 19). The three sums in $J$ penalise, in order: predicted tracking error (weighted by $\mathbf Q$), how much the input changes from step to step (weighted by $\mathbf R$, to avoid jerky motors), and how far the input is from a reference input (weighted by $\mathbf N$). Constraints such as "a rotor cannot spin faster than this" are included directly.
4. **Apply only the first input**, measure what actually happened, and repeat from step 2. This repetition is why the method is also called receding-horizon control.

Slide 18 shows the architecture: references in, an **SMPC** (switching MPC, which swaps between the linear models $\mathbf A_j, \mathbf B_j$ as the attitude changes) computes $U_2, U_3, U_4$ and $\Omega_r$, the quadrotor responds, an **EKF** (Extended Kalman Filter, an estimator you meet in Lecture 10) turns noisy measurements $m_x, m_y, m_z$ into clean estimates $\hat\phi, \hat\theta, \hat\psi$ and their rates, which are fed back. Slide 19 shows an experimental flight path.

**Analogy:** driving a car at night. You look ahead as far as the headlights reach (the horizon), plan the steering for that stretch, turn the wheel a little, and then look again. You never commit to the whole plan, only to the first move.

MPC gives the best performance but needs a lot of computing power, which is why slide 27 lists it under "alternatives" and the lecture focuses on the cheaper cascade.

---

## 12. A simplified cascade control design

### 12.1 The idea and the notation

Slide 20 states the plan in three steps:

1. Control the height of the vehicle.
2. Go to the desired position, under a controlled height (use roll and pitch for $x$–$y$ translation).
3. Control the heading (yaw) of the vehicle.

A **cascade** means controllers nested inside each other: the outer controller does not drive the motors directly, it hands a **reference** (a target value) to the next controller inwards. For position this makes physical sense because of Section 10: to move in $x$ you cannot push sideways, you must first tilt, so the position controller's output is a *tilt angle request*, which the attitude controller then achieves with torques.

Notation on slides 21–26: a star $^*$ means **desired** (reference) and a hash $^\#$ means **measured** (what the sensors say). A dot means rate of change. So $z^*$ is the height you want and $\dot z^\#$ is the measured vertical speed. $K_p$ and $K_d$ are tuning gains: $K_p$ multiplies the error (proportional action), $K_d$ multiplies the rate error (derivative action, which adds damping). Together they form a PD controller.

### 12.2 Altitude controller (slide 21)

$$T = K_p\,(z^* - z^\#) + K_d\,(\dot z^* - \dot z^\#) + T_0$$

Inputs: desired height and climb rate, measured height and climb rate. Output: total thrust $T$.

- The $K_p$ term pushes harder the further below target you are.
- The $K_d$ term pushes less if you are already climbing fast, which stops overshoot.
- $T_0$ is the **hover thrust** (feed-forward), approximately $mg$. Without it the drone would need a permanent height error to generate enough lift just to stay airborne. In the Lab 3 template this is the `to_thrust = 0.6` constant.

**Worked example.** Target $z^* = 1.0\ \text{m}$, measured $z^\# = 0.8\ \text{m}$, not moving, $K_p = 2\ \text{N/m}$, $K_d = 1\ \text{N s/m}$, $T_0 = 10.79\ \text{N}$:
$T = 2(0.2) + 1(0) + 10.79 = 11.19\ \text{N}$, so about 4% more than hover, and the drone climbs gently. As it starts climbing at, say, $0.2\ \text{m/s}$, the $K_d$ term subtracts $0.2\ \text{N}$ and the climb settles smoothly.

### 12.3 Position controller (slide 22)

Two P controllers in a row, for each horizontal axis.

$$u_x^* = K_p\,(x^\# - x^*), \qquad u_y^* = K_p\,(y^\# - y^*)$$
$$\theta_p^* = K_{p_1}\,(u_x^* - \dot x^\#), \qquad \theta_r^* = K_{p_2}\,(u_y^* - \dot y^\#)$$

Step by step:

1. Position error becomes a **velocity reference** $u_x^*$: the further away you are, the faster you should be moving toward the goal.
2. Velocity error (reference minus measured $\dot x^\#$) becomes an **angle reference**: pitch $\theta_p^*$ for the $x$ axis, roll $\theta_r^*$ for the $y$ axis, consistent with $\ddot x \approx g\theta$ and $\ddot y \approx -g\phi$ from Section 10.

Inputs: $x^*, y^*, x^\#, y^\#, \dot x^\#, \dot y^\#$. Outputs: $\theta_p^*, \theta_r^*$. In practice these angle references are also clipped to a few degrees, so the small-angle model stays valid.

⚠️ **Sign convention inside the slides.** The altitude and yaw controllers use "desired minus measured" ($z^* - z^\#$), while the position and attitude controllers use "measured minus desired" ($x^\# - x^*$). Whether this is intentional depends on how positive pitch is defined on the platform (nose-down pitch moves the drone toward $+x$ on some drones and $-x$ on others), and in practice the sign is absorbed into the gain. For the exam, learn the **structure** (position error → velocity reference → angle reference) and be ready to argue the sign from $\ddot x \approx g\theta$.

**Worked example (structure only).** Target $x^* = 2\ \text{m}$, measured $x^\# = 0$, at rest. With $K_p = 0.5\ \text{s}^{-1}$ the velocity reference magnitude is $1\ \text{m/s}$. With $K_{p_1} = 0.2\ \text{s/m}$ the angle request magnitude is $0.2\ \text{rad}$ ($11°$), which the attitude loop would clip to perhaps $0.17\ \text{rad}$ ($10°$). From Section 6, that gives $\ddot x \approx g \times 0.17 \approx 1.7\ \text{m/s}^2$, so the drone reaches its $1\ \text{m/s}$ velocity reference in about $0.6\ \text{s}$. As it closes in, the position error shrinks, the velocity reference shrinks, and the angle request returns to zero.

### 12.4 Attitude controller (slide 23)

Again two P loops in a row, for roll and pitch.

$$\dot\theta_r^* = K_p\,(\theta_r^\# - \theta_r^*), \qquad \dot\theta_p^* = K_p\,(\theta_p^\# - \theta_p^*)$$
$$\tau_x = K_{p_1}\,(\dot\theta_p^* - \dot\theta_p^\#), \qquad \tau_y = K_{p_1}\,(\dot\theta_r^* - \dot\theta_r^\#)$$

1. Angle error becomes an **angular-rate reference** (how fast to rotate toward the requested tilt).
2. Rate error (reference minus the gyroscope reading) becomes a **torque**.

Inputs: $\theta_p^*, \theta_r^*$ (from the position controller), $\theta_p^\#, \theta_r^\#, \dot\theta_p^\#, \dot\theta_r^\#$ (from the IMU). Outputs: $\tau_x, \tau_y$. The inner rate loop is what makes the drone feel "stiff" and resist gusts, and it is the loop that must run fastest (Section 14).

⚠️ **Possible cross-wiring on slide 23.** The slide computes the roll torque $\tau_x$ from the *pitch-rate* error and the pitch torque $\tau_y$ from the *roll-rate* error. Physically, roll torque (about $x$) should be driven by the roll-rate error and pitch torque (about $y$) by the pitch-rate error. This looks like a labelling slip. Use the physically consistent pairing and confirm with your lecturer.

### 12.5 Decoupled yaw controller (slide 24)

$$\tau_z = K_p\,(\psi^* - \psi^\#) + K_d\,(\dot\psi^* - \dot\psi^\#)$$

A single PD loop from heading error to yaw torque. It is **decoupled**, meaning it runs on its own, because (thanks to Section 10) yaw does not affect where the drone goes, only which way it faces. Inputs: desired and measured yaw angle and yaw rate. Output: $\tau_z$.

### 12.6 Torque and thrust to motor commands (slide 25)

The four outputs ($T$ from altitude, $\tau_x, \tau_y$ from attitude, $\tau_z$ from yaw) go into the **control mixer** of Section 8, which returns $\varpi_1^2 \ldots \varpi_4^2$ and hence the four motor speeds $\omega_1 \ldots \omega_4$.

### 12.7 The whole cascade (slide 26)

Reading the diagram left to right:

1. **Position Controller** takes $x^\#, x^*, y^\#, y^*$ and outputs $\theta_p^*, \theta_r^*$.
2. **Attitude Controller** takes those angle references and outputs $\tau_x, \tau_y$.
3. **Yaw Controller** takes $\psi^\#, \psi^*$ and outputs $\tau_z$.
4. **Altitude Controller** takes $z^\#, z^*$ and outputs $T$.
5. **Control Mixer** turns $(T, \tau_x, \tau_y, \tau_z)$ into $\omega_{1,2,3,4}$.
6. **Quad Rotor** flies, and its **Full States** are measured and fed back to every controller.

Slide 27 summarises the chain of thinking as "position error → velocity error → attitude error". Each arrow is one P (or PD) gain.

### 12.8 What counts as the "low-level controller" (slide 29)

Slide 29 overlays two boxes on the cascade. In **2010**, the low-level controller shipped with a drone was just the mixer, and you had to write attitude, yaw and altitude loops yourself. By **2018**, the onboard autopilot includes attitude, yaw and altitude control plus the mixer. You only send it **angle, yaw-rate and thrust references**, and you write just the outer position loop. That is exactly the situation in Lab 3 (Section 17).

---

## 13. Pros and cons of cascaded control (slide 27)

**Pros**

- Easy to use and easy to understand: each loop is a one-line P or PD law.
- Easy to tune: start with the innermost (rate) loop and work outward. Each loop sees the one inside it as "already fast and well behaved".

**Cons**

- Mediocre performance: errors trickle down the chain, so the whole system is only as fast as its slowest loop. An outer loop must be several times slower than the one inside it to stay stable.
- Poor handling of large tilt angles: the design relies on the small-angle approximations of Section 10, which fail when the drone is aggressive.

**Alternatives**

- Full-model controllers such as **Integral Backstepping** or **MPC** (Section 11). They use the full nonlinear model, so they handle big angles and give near-optimal performance, at the price of heavy computation.
- A hybrid that is common in practice: keep a fast low-level attitude/thrust controller onboard (which removes the dependence on rotational kinematics and mass), and put a state-feedback controller for position and velocity on top that outputs angle references. This decouples the frame-dependent low-level part from the high-level part, which can then be linear or nonlinear as you like.

---

## 14. Hardware: where do the algorithms run? (slide 28)

Two computers, connected by a communication link.

**Low-level controllers** (sensor sampling, rate and attitude loops) run on a small **microcontroller** with hard real-time guarantees, at **1,000 to 32,000 Hz**. The reason for such high rates is the frame: a drone's arms vibrate at their natural (eigen) frequencies, often several hundred hertz. The **Nyquist criterion** says you must sample at more than twice the highest frequency present, otherwise the vibration shows up as a fake slow signal (aliasing) and the controller fights a ghost. Sampling in the kilohertz range keeps the controller safely above those vibration frequencies.

**High-level controllers** (position and trajectory control, and later planning and perception) need much more computing but only need to run at **10 to 100 Hz**, and they do not need hard real-time. They run on a **companion computer** (Intel Atom, i3, i5, i7 class boards).

**Analogy:** reflexes versus thinking. Your spinal reflex pulls your hand off a hot stove in milliseconds without consulting the brain. Deciding where to walk happens much more slowly in the brain. The drone's microcontroller is the spinal cord; the companion computer is the brain.

---

## 15. Coordinate conventions: NED versus ENU (slide 31)

Two right-handed world frames are in common use, and the lab makes you switch between them.

- **NED (North-East-Down)**: $x$ north, $y$ east, $z$ **down**. Used in aeronautics and in Peter Corke's book (so slides 10 and 12 are in NED, which is why gravity is $+mg$).
- **ENU (East-North-Up)**: $x$ east, $y$ north, $z$ **up**. Used by ROS and therefore by the Lab 3 software. Slide 31 draws the ROS version and the "standard" version from two different viewpoints, but they are the same frame: $z$ up in both.

The slide writes the change of frame as the relative pose ${}^{ENU}\xi_{NED}$ (Lecture 1 notation). ⚠️ The slide does not give the explicit transform. For a vector it is: swap $x$ and $y$, and flip the sign of $z$:

$$(x, y, z)_{ENU} = (y, x, -z)_{NED}$$

so, for example, a height of $1\ \text{m}$ above the ground is $z = -1$ in NED and $z = +1$ in ENU. If your controller was derived in NED and your sensor data arrives in ENU, every sign in the altitude loop flips. Check this before you fly.

---

## 16. The GRASP testbed equations (slides 33–34)

Slides 33 and 34 show the same cascade from reference [4], the GRASP lab at the University of Pennsylvania (Vijay Kumar's group). It is useful because it is the same physics written by a different group, so you can see what is convention and what is law.

Slide 33 block diagram: a desired trajectory $\mathbf r_T(t)$ and the measured position $\mathbf r(t)$ go into **Position Control**, which outputs desired angles $\phi^{des}, \theta^{des}, \psi^{des}$ and a thrust correction $\Delta\omega_F$. **Attitude Control** outputs rotor-speed corrections $\Delta\omega_\phi, \Delta\omega_\theta, \Delta\omega_\psi$ (this is the mixer folded into the attitude controller). **Motor Dynamics** turn speeds into forces $F_i$ and moments $M_i$. **Rigid Body Dynamics** produce acceleration $\ddot{\mathbf r}$, which is integrated twice to get position, while the body rates $p, q, r$ (their names for $\omega_x, \omega_y, \omega_z$) are fed back to the attitude controller.

Slide 34 equations, matched to earlier sections:

- **Centre of mass accelerations**: $m\ddot{\mathbf r} = (0, 0, -mg)^T + \mathbf R\,(0, 0, \Sigma F_i)^T$. Same as slide 14 in a $z$-up frame, with drag ignored.
- **Angular velocities**: $(p, q, r)^T = \begin{bmatrix} c\theta & 0 & -c\phi\,s\theta \\ 0 & 1 & s\phi \\ s\theta & 0 & c\phi\,c\theta \end{bmatrix}(\dot\phi, \dot\theta, \dot\psi)^T$. Same role as the matrix in Section 7.3, but ⚠️ the entries differ because GRASP uses a Z-X-Y Euler angle sequence while slide 11 uses Z-Y-X. Both are correct for their own convention; do not mix them.
- **Angular acceleration**: $\mathbf I\,(\dot p, \dot q, \dot r)^T = \big(L(F_2 - F_4),\ L(F_3 - F_1),\ M_1 - M_2 + M_3 - M_4\big)^T - (p, q, r)^T \times \mathbf I\,(p, q, r)^T$. This is Euler's equation of Section 7.2 with the torques written out, where $L$ is the arm length. ⚠️ The rotor numbering differs from slide 11 (here rotor 1 is on the $x_B$ axis and rotors 2, 4 are on the $y_B$ axis, so roll torque is $L(F_2 - F_4)$ rather than $d(T_4 - T_2)$). Again a convention difference, not an error.

The lesson: when you read any paper, first find the rotor numbering, the Euler sequence and the $z$ direction, then the formulas will match.

---

## 17. How this connects to Lab 3 (Crazyflie)

The lab template `Lab 3 - Aerial Robotics/crazyflie_controller_template.py` is the cascade of Section 12.8 in its 2018 form.

- A ROS subscriber reads **odometry from the Vicon motion-capture system**: position $(x, y, z)$, orientation as a quaternion (converted to roll, pitch, yaw with a small helper, because "Euler angles are easier to use"), linear velocities and the yaw rate. These are your $^\#$ (measured) values, in the ENU frame.
- The loop runs at **20 Hz**, which is the "high-level, 10–100 Hz" rate of Section 14.
- The references are $x^* = 0$, $y^* = 4$, $z^* = 1$, $\psi^* = 0$ (a centre point in the aerial lab), and a hover thrust constant `to_thrust = 0.6` plays the role of $T_0$.
- You fill in the position, altitude and yaw loops. The template then publishes a `Twist` message where `linear.x` is the **pitch command**, `linear.y` the **roll command**, `linear.z` the **thrust** and `angular.z` the **yaw rate**. The Crazyflie's onboard firmware runs the attitude controller and mixer at kilohertz rates.

So in the lab you write Sections 12.2, 12.3 and 12.5, and the drone does 12.4 and 12.6 for you.

---

## 18. Further reading (slide 32)

1. A survey on open-source flight control platforms for UAVs (hardware, Section 14).
2. Quadrotor rotorcraft control: modelling of UAVs (Section 9).
3. Gibiansky, *Quadcopter Dynamics, Simulation, and Control* (Sections 7, 9).
4. GRASP cascade control (Sections 12, 16).
5. Alexis, Nikolakopoulos, Tzes, switching MPC for UAVs (Section 11).

---

## In a nutshell

A quadrotor has four fixed rotors, two spinning each way, and it can only do four things: push up, roll, pitch and yaw, by changing which rotors spin faster. Thrust and drag torque grow with the square of rotor speed, and a 4×4 matrix $\mathbf A$ maps the four squared speeds to total thrust and three torques; its inverse, the control mixer, goes the other way. Newton's law in the world frame (gravity, rotated thrust, drag) describes translation, and Euler's equation in the body frame (inertia, gyroscopic term, torques) describes rotation, with the subtlety that the gyroscope's angular velocity is not the same as the rate of change of the Euler angles. Near hover with yaw held at zero, the model collapses to "forward acceleration is $g$ times pitch, sideways acceleration is minus $g$ times roll", which is why a simple cascade works: position error gives a velocity reference, velocity error gives an angle reference, angle error gives a rate reference, rate error gives a torque, and the mixer gives motor speeds. Cascades are easy to tune but slow and limited to small angles; MPC and backstepping do better at a much higher computing cost. Fast inner loops run at kilohertz rates on a microcontroller to stay above frame vibrations, while position control runs at tens of hertz on a companion computer, and you must keep straight whether your frame is NED ($z$ down) or ENU ($z$ up).

---

## Key terms

- **Quadrotor**: a drone with four fixed-pitch rotors, two clockwise and two anticlockwise.
- **Roll / pitch / yaw ($\phi, \theta, \psi$)**: rotation about the body's forward, sideways and vertical axes.
- **Body frame / inertial (world) frame**: axes glued to the drone versus axes fixed to the world.
- **State**: a quantity needed to describe the system now; here 12 of them (position, velocity, angles, angular rates).
- **IMU**: inertial measurement unit; accelerometers, gyroscopes and magnetometers on one chip.
- **Thrust coefficient $b$ / drag coefficient $k$**: constants in $T_i = b\varpi_i^2$ and $Q_i = k\varpi_i^2$.
- **Arm length $d$ (or $L$)**: distance from the drone's centre to each rotor; the lever arm for roll and pitch torque.
- **Torque**: a twisting force that produces angular acceleration.
- **Inertia matrix $\mathbf J$ (or $\mathbf I$)**: rotational "mass"; how hard the body is to spin about each axis.
- **Euler's equation**: $\mathbf J\dot{\boldsymbol\omega} = -\boldsymbol\omega\times\mathbf J\boldsymbol\omega + \boldsymbol\tau$, Newton's law for rotation.
- **Gyroscopic term**: $-\boldsymbol\omega\times\mathbf J\boldsymbol\omega$, the coupling that makes a spinning body resist tipping.
- **Control mixer**: $\mathbf A^{-1}$, which turns desired thrust and torques into four squared rotor speeds.
- **State-space model**: a list of states plus formulas for each one's rate of change.
- **Small-angle approximation**: $\sin\theta \approx \theta$, $\cos\theta \approx 1$, valid for angles below roughly $15°$.
- **Cascade control**: nested loops where each outer loop outputs the reference for the next inner loop.
- **PD controller**: output = $K_p$ × error + $K_d$ × rate-of-error; proportional plus derivative action.
- **Feed-forward (hover thrust $T_0$)**: a constant added so the controller does not need an error to hold altitude.
- **Reference ($^*$) / measurement ($^\#$)**: desired value versus sensor value.
- **MPC (Model Predictive Control)**: predict the future with a model, optimise the inputs over a horizon, apply the first, repeat.
- **EKF (Extended Kalman Filter)**: an estimator that fuses noisy sensors into state estimates (Lecture 10).
- **Hard real time**: deadlines that must never be missed.
- **Nyquist criterion**: sample at more than twice the highest frequency present, or aliasing occurs.
- **NED / ENU**: North-East-Down (z down, aeronautics, Corke) versus East-North-Up (z up, ROS).
- **Crazyflie 2**: the small open-source quadrotor used in Lab 3.

---

## Check yourself

1. A quadrotor is hovering. Describe exactly which rotors you speed up and slow down to (a) climb, (b) roll, (c) yaw, and explain why the yaw manoeuvre does not make the drone tilt or change height.
2. Starting from Newton's second law, explain in words why tilting the drone by a small pitch angle $\theta$ produces a forward acceleration of about $g\theta$. What two approximations did you use?
3. Write the 4×4 matrix $\mathbf A$ that relates squared rotor speeds to $(T, \tau_x, \tau_y, \tau_z)$ for the rotor layout of slide 11, and explain why the yaw row has no arm length $d$ in it.
4. In the cascade controller, list the input and output of each of the four controllers and the mixer, and explain why the position controller outputs an *angle* rather than a force.
5. Why do attitude and rate loops run at 1,000–32,000 Hz on a microcontroller, while the position loop runs at 10–100 Hz on a companion computer? Mention the Nyquist criterion.

<details>
<summary>Answers</summary>

1. (a) Climb: speed up all four rotors equally, so lift exceeds weight with no net torque. (b) Roll: speed up one side rotor (say 4) and slow the opposite side rotor (2) by the same amount; the lift difference times the arm length $d$ gives $\tau_x = db(\varpi_4^2 - \varpi_2^2)$ while total thrust stays almost constant. (c) Yaw: speed up the pair 1 and 3 (which spin one way) and slow the pair 2 and 4 (which spin the other way). Total thrust is unchanged because two go up and two go down. No roll or pitch torque appears because the speeded-up rotors sit opposite each other, so their lift differences cancel about both horizontal axes. But the drag torques $Q_i = k\varpi_i^2$ of the two spin directions no longer cancel, so a net torque about $z$ remains: $\tau_z = k(\varpi_1^2 + \varpi_3^2 - \varpi_2^2 - \varpi_4^2)$.

2. In the world frame, $m\ddot{\mathbf x} = \text{gravity} + \mathbf R\,(0,0,T)^T$. Thrust is along the body $z$ axis; pitching the body by $\theta$ rotates that axis, so the thrust has a horizontal component $T\sin\theta$ and a vertical component $T\cos\theta$. Approximation 1 (small angle): $\sin\theta \approx \theta$, $\cos\theta \approx 1$, so vertical lift is almost unchanged and the horizontal force is $T\theta$. Approximation 2 (near hover): $T \approx mg$. Hence $m\ddot x \approx mg\theta$, or $\ddot x \approx g\theta$.

3. With rotor 1 front ($+x$), 3 back, 4 on $+y$ and 2 on $-y$, and the $z$-down convention of slide 12:
$\mathbf A = \begin{pmatrix} -b & -b & -b & -b \\ 0 & -db & 0 & db \\ db & 0 & -db & 0 \\ k & -k & k & -k \end{pmatrix}$ (in a $z$-up convention the first row is $+b$). Roll and pitch torques are produced by a lift *force* at a distance $d$ from the centre, so they are force times lever arm. Yaw torque is produced by each propeller's aerodynamic drag *torque* $Q_i = k\varpi_i^2$, which acts about the rotor's own spin axis (parallel to body $z$) and is transmitted directly to the body; no lever arm is involved, so $d$ does not appear.

4. Altitude controller: in $z^*, z^\#, \dot z^*, \dot z^\#$; out total thrust $T$ (PD plus hover feed-forward $T_0$). Position controller: in $x^*, y^*, x^\#, y^\#, \dot x^\#, \dot y^\#$; out angle references $\theta_p^*, \theta_r^*$ (position error → velocity reference → angle reference). Attitude controller: in $\theta_p^*, \theta_r^*$ and measured angles and rates; out $\tau_x, \tau_y$ (angle error → rate reference → torque). Yaw controller: in $\psi^*, \psi^\#, \dot\psi^*, \dot\psi^\#$; out $\tau_z$ (PD). Mixer: in $(T, \tau_x, \tau_y, \tau_z)$; out $\varpi_1^2 \ldots \varpi_4^2$ via $\mathbf A^{-1}$, then square-root to motor speeds. The position controller outputs an angle because a quadrotor has no way to push sideways directly: its only horizontal force comes from tilting the thrust vector, so the only "handle" the position loop can pull is the tilt angle, which the attitude loop then realises.

5. The drone frame vibrates at its natural frequencies, typically hundreds of hertz, and those vibrations are picked up by the IMU. By the Nyquist criterion you must sample at more than twice the highest frequency present or the vibration aliases into a fake low-frequency signal that the controller would react to. Sampling and controlling at kilohertz rates keeps the rate and attitude loops above that band; these loops are also what keeps the drone from falling, so they need hard real-time guarantees, which a dedicated microcontroller provides. Position changes slowly and the position loop sits outside the (fast, already stabilised) attitude loop, so 10–100 Hz is plenty, and missing an occasional deadline is not catastrophic. That allows the heavy computation (trajectory control, later planning and vision) to run on a general-purpose companion computer without hard real-time.

</details>
