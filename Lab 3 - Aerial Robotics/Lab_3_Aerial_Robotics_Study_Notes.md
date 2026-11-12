# Lab 3 – Aerial Robotics (Crazyflie) – Study Notes

> ⚠️ **No lab handout is in the workspace.** These notes are reconstructed from the two Simulink models (`LAB3_CrazyFlie_2024.slx`, `Lab3_Crazyflie_VirtualTemplate2023.slx`), the Python template `crazyflie_controller_template.py`, the message definitions in `mav_msgs.zip`, the course plan on Lecture 1 slide 5 ("LAB 3 – Experimentation with aerial robots") and the Lecture 6 notes. Get the official task list from Canvas and check it against this. Anything marked *inferred* is my best reading of the templates, not a statement from the lecturer.

## What this lab is about, and why it matters

In Lab 3 you make a real palm-sized quadrotor, the Crazyflie 2, hover and move where you tell it. You do that by writing the outer loops of the cascade controller from Lecture 6: your code reads the drone's position from a motion-capture system, decides how much to tilt and how hard to push, and sends those commands twenty times a second. It is the moment the aerial-robotics theory meets gravity, so getting the frames, the signs and the hover thrust right matters more than any clever control law.

## Roadmap: the main topics

1. What is in the folder and what each file is for.
2. The hardware and software setup you can infer: Crazyflie, Vicon, ground PC, ROS 1.
3. The command message: a `Twist` whose fields are re-purposed as pitch, roll, thrust and yaw rate.
4. Frame conventions: ENU, x forward, y left, z up, and the z-y'-x'' Euler sequence.
5. The 2024 Simulink model, block by block, including why the position error is rotated into the body frame.
6. The 2023 virtual template: a simulated follower drone tracking a leader.
7. The Python template, line by line.
8. The controller you are expected to build: altitude, position, yaw.
9. A starting-gain and tuning procedure.
10. Safety procedure (general background).
11. Likely tasks (inferred), plots and report checklist.
12. Debugging checklist and examiner questions.

---

## 1. What is in the folder

| File | What it is |
|---|---|
| `LAB3_CrazyFlie_2024.slx` | Simulink model for the **real** drone. ROS 1 blocks, Vicon odometry in, `cmd_vel` out. Your controller goes in the middle. |
| `Lab3_Crazyflie_VirtualTemplate2023.slx` | Simulink model for a **simulated** drone that follows a "leader". Has a Position Controller subsystem already wired. |
| `crazyflie_controller_template.py` | The same job in ROS 1 Python: subscribe to Vicon, publish a `Twist` at 20 Hz. "Implement your controller" is left blank. |
| `mav_msgs.zip` | A ROS message package (from the ETH Zürich RotorS project) that defines messages such as `RollPitchYawrateThrust`. It documents the **frame and rotation conventions** the drone software uses. |
| `Report Template.docx` | Same template as every lab: objective, methodology, results, about 1.5 pages plus plots, submitted as one zip with code and videos. |
| `../Crazy_Fly_screenshot.png` | A photo of Crazyflies in flight. Each carries small reflective balls: those are the motion-capture markers. |

---

## 2. The setup you can infer

**The drone.** The **Crazyflie 2** is a 27-gram open-source quadrotor from Bitcraze. Its onboard microcontroller runs the fast inner loops from Lecture 6 (attitude and rate control plus the mixer, at kilohertz rates). You never touch those. You only send it *what attitude and thrust to hold*.

**The eyes: Vicon motion capture.** A ring of infrared cameras on the ceiling tracks the reflective markers on the drone and computes its position and orientation at high rate with millimetre precision. Both templates subscribe to a topic named like `/pixy/vicon/demo_crazyflie3/demo_crazyflie3/odom`. "pixy" is the lab's Vicon computer, "demo_crazyflie3" is the name of the tracked object, and the message type is `nav_msgs/Odometry`, the same type the TurtleBot uses for its wheel odometry. So from your code's point of view, the drone has a perfect external GPS indoors.

**Everyday analogy:** Vicon is a referee standing above the pitch who shouts the exact coordinates of the ball twenty times a second. Your controller never has to guess where the drone is, it only has to decide what to do about it.

**The brain: a ground PC.** Your controller runs on a lab computer, not on the drone. It receives Vicon odometry over ROS, computes commands, and publishes them on `/demo_crazyflie3/cmd_vel`; a bridge node on the same PC forwards them to the drone by radio. The Python template's loop runs at **20 Hz**, which is the "high-level, 10–100 Hz" layer from Lecture 6, section 14.

**ROS version.** Both Simulink models use `robotlib` blocks and the Python uses `rospy`, so Lab 3 is **ROS 1**, unlike the current Lab 2 and Lab 4 templates which are ROS 2. ⚠️ Topic names contain a drone number (`demo_crazyflie3`, `demo_crazyflie6`); your drone will have its own, so expect to edit the Subscribe and Publish blocks.

---

## 3. The command message: a re-purposed `Twist`

A `geometry_msgs/Twist` normally means "linear velocity (x, y, z) and angular velocity (x, y, z)". The Crazyflie bridge does **not** read it that way. Both templates fill it like this:

| Twist field | Meaning for the Crazyflie | Symbol in the Python template |
|---|---|---|
| `linear.x` | **pitch command** (tilt nose down to go forward) | `u_p` |
| `linear.y` | **roll command** (tilt to go sideways) | `u_r` |
| `linear.z` | **thrust** (how hard all four rotors push) | `u_t` |
| `angular.z` | **yaw rate** (how fast to spin about the vertical) | `u_y` |

This is exactly the `RollPitchYawrateThrustCrazyflie` message from `mav_msgs` squeezed into a `Twist` for convenience. That message's own comments say: roll and pitch in **radians**, yaw rate in **rad/s**, and thrust "in PWM" for the Crazyflie variant.

> ⚠️ **The thrust scale is the single most important thing to confirm before flying.** The Python template hovers at `to_thrust = 0.6` (a unitless 0 to 1 fraction, it seems). The virtual Simulink template multiplies thrust by a **Gain of 10000**, which suggests the simulated drone wants raw PWM counts (the Crazyflie firmware's thrust range is 0 to 65535, and roughly 35000 to 45000 is a typical hover). The 2024 real-drone model has four unlabelled Gain blocks on the command path whose values I could not read. Ask the lab assistant: *is thrust 0 to 1, or 0 to 65535, and what value hovers this drone with this battery?* Getting this wrong by a factor of ten thousand is the difference between not lifting off and hitting the ceiling.

**Common mistake:** assuming `linear.x` is a velocity because that is what `Twist` means on the TurtleBot. On the drone it is an *angle*. If you feed it metres per second you will command enormous tilts.

---

## 4. Frame conventions

The `mav_msgs` definitions spell out the convention the whole stack uses, and it matches ROS:

- **Body frame:** x **forward**, y **left**, z **up**.
- **World frame:** ENU, that is x east, y north, z **up**. Vicon reports in this frame (its own origin and axes, calibrated in the room).
- **Rotation convention z-y'-x'':** yaw about the fixed z axis first, then pitch about the new y', then roll about the newest x''. In Lecture 1 language this is the **ZYX roll-pitch-yaw** sequence, $R = R_z(\psi) R_y(\theta) R_x(\phi)$.

Two consequences you must carry into your controller:

1. **Height is positive up.** The Lecture 6 slides 10 and 12 used NED (z down, gravity $+mg$). Everything here is ENU, so "go higher" is "increase z". See the Lecture 6 notes, section 15, for the NED to ENU swap $(x, y, z)_{ENU} = (y, x, -z)_{NED}$.
2. **Positive pitch and positive roll have fixed meanings.** With x forward and y left, a right-handed rotation about y (positive pitch) tips the nose **down**, which accelerates the drone **forward** (+x body). A right-handed rotation about x (positive roll) dips the **right** side... but wait, y is *left*, so positive roll (right-hand rule about the forward axis) lifts the left side and dips the right side, accelerating the drone to the **right**, which is **negative** y body. So: $\ddot x_B \approx g\,\theta$ and $\ddot y_B \approx -g\,\phi$. That minus sign is the same one that appears in Lecture 6 section 10, and it is the reason the roll gain in your position controller has the opposite sign to the pitch gain.

> ⚠️ Whether the *bridge* applies these signs as written, or flips one so that "positive roll command = move left", is firmware-dependent. Test it in the air at low height with tiny commands before trusting any sign.

**Everyday analogy:** riding a bicycle with no hands. To go forward you lean forward (pitch). To drift right you lean right (roll). Your controller decides *how much* to lean from *how far* you are from where you want to be.

---

## 5. The 2024 Simulink model, block by block

`LAB3_CrazyFlie_2024.slx` has 41 blocks. Reading its structure:

**Input side.**
- A `robotlib/Subscribe` on `/pixy/vicon/demo_crazyflie3/demo_crazyflie3/odom` receives the Vicon `Odometry` bus.
- A **Bus Selector** pulls out position and orientation fields.
- A **Conversion** subsystem takes the quaternion components `x y z w` and outputs `yaw`, `pitch`, `roll`. Inside it is an S-Function doing the quaternion-to-Euler maths (the same formula as the Python helper, section 7). A **Goto** block labelled "Send state" broadcasts the state to the rest of the model.

**The rotation block, the heart of it.**
- A **MATLAB Fcn** block computes `rot2(yaw)'` and a **Product** block labelled `{0} → {B}` multiplies it with a two-element vector. `rot2(ψ)` is the Lecture 1 two-dimensional rotation matrix; its transpose is its inverse. So this block takes a vector written in the world frame {0} and expresses it in the body frame {B}:

$$ \begin{pmatrix} e_x^B \\ e_y^B \end{pmatrix} = \begin{pmatrix} \cos\psi & \sin\psi \\ -\sin\psi & \cos\psi \end{pmatrix} \begin{pmatrix} e_x^W \\ e_y^W \end{pmatrix} $$

Why is this needed? Pitch and roll are *body* commands: pitch moves the drone along *its own* nose, not along world x. If the drone has yawed, "go east" is no longer "pitch forward". You must first ask "where is the goal, as seen from the drone?".

**Worked example.** The drone sits at the origin, yawed $\psi = 90°$ (nose pointing along world +y, north). The goal is 1 m away along world +x (east), so $e^W = (1, 0)$. Apply the matrix with $\cos 90° = 0$, $\sin 90° = 1$:

$$ e^B = \begin{pmatrix} 0 & 1 \\ -1 & 0 \end{pmatrix}\begin{pmatrix} 1 \\ 0 \end{pmatrix} = \begin{pmatrix} 0 \\ -1 \end{pmatrix}. $$

In the body frame the goal is at $(0, -1)$: zero ahead, one metre to the **right** (negative y is right, since y is left). So the drone must **not** pitch at all; it must **roll** to its right. Had you skipped the rotation and fed $(1, 0)$ straight into pitch, the drone would fly north, 90° away from the goal, and then correct in a spiral. This is the number one cause of "it drifts in a circle" in this lab.

**Output side.**
- Four **Gain** blocks scale the body-frame errors and the altitude/yaw errors into commands. ⚠️ I could not read their values; treat them as the proportional gains you must tune (section 9).
- A **Command Velocity Publisher** subsystem with inports `thrust`, `roll`, `pitch`, `yaw` fills a Blank `Twist` message via a Bus Assignment and a `robotlib/Publish` on `/demo_crazyflie3/cmd_vel`.

**Common mistake:** rotating with `rot2(yaw)` instead of `rot2(yaw)'`. The un-transposed matrix maps body to world, the opposite of what you need, and the drone will run away from the goal at any yaw other than zero.

---

## 6. The 2023 virtual template: leader and follower

`Lab3_Crazyflie_VirtualTemplate2023.slx` is for a **simulator**, not the real drone (no Vicon topic; it subscribes to `/crazyflie2/odometry` and publishes `/crazyflie2/cmd_vel`). Two things make it worth opening:

1. It subscribes to a **second** odometry, `/leader/odometry`. The follower's reference position comes from the leader's position (a "From" block delivers it as `xRef`, `yref`). So the task it models is **leader following**: fly in formation behind another drone or a moving target.
2. It contains a worked **Position Controller** subsystem with inports `xMeas`, `yMeas`, `xRef`, `yref` and outports `pitch`, `roll`, plus a gain called `D_xy1`. The "D" naming suggests a derivative (damping) term on the x–y loop, which is exactly what section 8 says you need. A `{0} → {B}` product block sits before it, so again errors are rotated into the body frame first.
3. Thrust passes through a **Gain of 10000** before publishing (see the thrust warning in section 3).

Use it as a reference wiring diagram: copy the structure into the 2024 model, then replace the simulator topics with the Vicon ones.

---

## 7. The Python template, line by line

```python
sub = rospy.Subscriber('/pixy/vicon/demo_crazyflie6/demo_crazyflie6/odom', Odometry, callback)
pub = rospy.Publisher('demo_crazyflie6/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(20)
```
Subscribe to Vicon for drone number 6, publish commands for the same drone, loop at 20 Hz.

```python
def callback(data):
    xpos = data.pose.pose.position.x   # likewise ypos, zpos
    qx..qw = data.pose.pose.orientation.{x,y,z,w}
    d_yaw = data.twist.twist.angular.z   # yaw rate
    [roll, pitch, yaw] = quaternion_to_euler(qx, qy, qz, qw)
    vx, vy, vz = data.twist.twist.linear.{x,y,z}
```
Every time Vicon publishes, the callback copies position, the three Euler angles, the three linear velocities and the yaw rate into global variables. **Note that velocities come for free** from the odometry twist; you do not have to differentiate position yourself, which would be noisy.

The quaternion helper is the standard ZYX extraction (Lecture 1 quaternion notes):
$\phi = \operatorname{atan2}(2(q_w q_x + q_y q_z),\ 1 - 2(q_x^2 + q_y^2))$, $\theta = \arcsin(2(q_w q_y - q_z q_x))$ clamped to $[-1, 1]$, $\psi = \operatorname{atan2}(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2))$.

```python
xref, yref, zref, yawref = 0, 4, 1, 0
to_thrust = 0.6
```
The setpoint is "a centre-point in the aerial lab": 4 m north of the Vicon origin, 1 m up, nose along x. `to_thrust` is the feed-forward hover thrust $T_0$.

```python
while not rospy.is_shutdown():
    # ---- your controller here: compute u_p, u_r, u_t, u_y ----
    cmd_vel.linear.x = u_p; cmd_vel.linear.y = u_r
    cmd_vel.linear.z = u_t; cmd_vel.angular.z = u_y
    pub.publish(cmd_vel); rate.sleep()
```
The pattern of every robot program again: read, compute, publish, wait.

⚠️ As shipped, the globals `u_p, u_r, u_t, u_y` are 0, so running the template unmodified publishes zero thrust: the drone stays on the floor (safe) but also proves nothing. Also note the template never declares `u_p` etc. as `global` inside `controller()`, so if you assign them inside the loop Python treats them as locals, which is fine, but if you compute them in a helper function you must return them.

---

## 8. The controller you are expected to build

This is the cascade from Lecture 6 section 12 with the two inner loops (attitude and mixer) removed because the Crazyflie firmware does them. You write three outer loops.

### 8.1 Altitude: height error to thrust

$$ u_t = T_0 + K_{p,z}\,(z^* - z) + K_{d,z}\,(0 - v_z) + K_{i,z}\int (z^* - z)\,dt $$

- $T_0$: the hover thrust (0.6 in the Python template, scale ⚠️). Without it the drone cannot lift off until the integrator winds up, which takes seconds and overshoots.
- $K_{p,z}(z^* - z)$: push harder when below the setpoint. Positive error (too low) must *increase* thrust, so $K_{p,z} > 0$ in ENU.
- $K_{d,z}(-v_z)$: damping, uses the measured vertical velocity from the odometry twist. Brakes the climb before it overshoots.
- The integral term removes the steady offset caused by battery sag: as the battery drains, the same PWM gives less thrust and the drone slowly sinks without it.
- **Saturate** $u_t$ to the valid range, and **clamp the integrator** (anti-windup) so it cannot grow while the drone sits on the floor.

**Worked example** (0 to 1 thrust scale, gains illustrative): $T_0 = 0.6$, $K_{p,z} = 0.3$, $K_{d,z} = 0.15$, drone at $z = 0.7$ m rising at $v_z = 0.4$ m/s, setpoint 1 m. Then $u_t = 0.6 + 0.3(0.3) + 0.15(-0.4) = 0.6 + 0.09 - 0.06 = 0.63$. Slightly above hover: keep climbing, but the damping term is already easing off.

### 8.2 Position: x–y error to pitch and roll

Step 1, world error: $e^W = (x^* - x,\ y^* - y)$.
Step 2, rotate into the body frame: $e^B = \text{rot2}(\psi)^T e^W$ (section 5). Do the same for the velocity: $v^B = \text{rot2}(\psi)^T (v_x, v_y)$.
Step 3, PD law:

$$ \theta^* = u_p = K_{p,xy}\, e_x^B - K_{d,xy}\, v_x^B, \qquad \phi^* = u_r = -\big(K_{p,xy}\, e_y^B - K_{d,xy}\, v_y^B\big). $$

The leading minus on roll comes from $\ddot y_B \approx -g\phi$ (section 4): to move left (+y body) you need *negative* roll. ⚠️ Confirm the sign with a tiny test flight; some bridges flip it for you.

Saturate both angles, for example to $\pm 0.2$ rad (about 11°). Small angles keep the linear model valid (Lecture 6 section 13's main con of cascades) and keep the drone inside the net.

**Worked example.** Setpoint $(0, 4)$, drone at $(0.5, 3.0)$, yaw $\psi = 0$, velocity $(0, 0.5)$ m/s, $K_{p,xy} = 0.4$, $K_{d,xy} = 0.2$. World error $(-0.5, 1.0)$; at zero yaw the body error is the same. Pitch $= 0.4(-0.5) - 0.2(0) = -0.2$ rad (nose up, fly backwards half a metre). Roll $= -(0.4(1.0) - 0.2(0.5)) = -(0.4 - 0.1) = -0.3$ rad, saturated to $-0.2$: roll left (negative) to move +y, with the damping already reducing the demand because it is moving that way.

### 8.3 Yaw: heading error to yaw rate

$$ u_y = K_{p,\psi}\,\operatorname{wrap}(\psi^* - \psi) $$

Wrap the error into $[-\pi, \pi]$ (use `angdiff` or atan2 of sine and cosine) or the drone will spin the long way round at $\pm 180°$. Yaw is the least critical loop; a small gain and no derivative is fine. Remember that every time yaw changes, section 5's rotation changes too, which is why the position loop must read the *live* yaw.

### 8.4 Where the hidden loops are

The Crazyflie firmware receives $(\theta^*, \phi^*, \dot\psi^*, T)$ and runs attitude PID, rate PID and the mixer at 500 Hz to 1 kHz. In Lecture 6 numbering you write 12.2, 12.3 and 12.5; the drone does 12.4 and 12.6.

---

## 9. Starting gains and a tuning procedure

Tune **inner to outer**, as Lecture 6 section 13 says, which for you means altitude first, then position, then yaw.

1. **Dry run on the floor.** Publish $u_t = 0$, confirm odometry arrives, print the yaw and check it changes sign correctly when you turn the drone by hand.
2. **Find $T_0$.** Slowly ramp a constant thrust by hand (no feedback) until the drone just gets light on its legs. Note that value. That is your hover feed-forward for *this* battery.
3. **Altitude only.** Hold pitch, roll and yaw rate at zero. Start with a small $K_{p,z}$, add $K_{d,z}$ when it bounces, add a small $K_{i,z}$ last. Fly at 0.5 m first.
4. **Position, one axis at a time.** Enable pitch with a small gain, then roll, with the derivative on the Vicon velocity. If it oscillates slowly it is a sign or yaw-rotation error, not a gain problem; if it oscillates fast, halve the gain.
5. **Yaw last.**
6. Always **saturate** every command and have someone's finger on the **emergency stop**.

**Common mistake:** differentiating Vicon position numerically for the D term. Use `twist.twist.linear` from the odometry message; it is already filtered.

---

## 10. Safety procedure (general background, ⚠️ not from the handout)

- Propellers spin at over 20 000 rpm. Keep fingers and hair away, wear eye protection if the lab asks you to.
- Fly inside the net or the marked flight volume only. Vicon cannot see the drone outside the camera volume, and your controller then acts on stale data.
- Check the markers are firmly attached and that Vicon shows one solid object, not a flickering one, before take-off.
- Know the kill switch: a zero-thrust command, Ctrl+C on the controller node, and the physical emergency stop.
- Charge Li-Po cells with the proper charger and never fly a battery that is puffed or below the warning voltage (the same Li-Po rules as the TurtleBot notes).
- One person flies, one watches, nobody stands under the drone.

---

## 11. Likely tasks (inferred), plots and report

> ⚠️ Inferred from the templates and the Lab 2 pattern; confirm on Canvas.

1. **Hover at a setpoint.** Reach $(0, 4, 1)$ and hold it. Plot $x, y, z$ against time with the references, and the commanded thrust.
2. **Step changes.** Move the setpoint by, say, 1 m in x, then in y, then 0.5 m in z. Report rise time, overshoot and steady-state error for each axis.
3. **Waypoints or a trajectory.** Fly a square or a circle at constant height, as in Lab 2 Task 2 but in the air. Plot the 3-D path with equal axes.
4. **Leader following** (from the virtual template). Track a moving reference at a fixed offset.

**Report checklist.** Block diagram of your cascade with gains; the sign conventions you verified; time plots of every axis with reference and measurement; the command signals with their saturation limits visible; a short note on battery sag and the integrator; what went wrong and what you changed. Video of at least one flight.

---

## 12. Debugging checklist and examiner questions

| Symptom | Most likely cause | Fix |
|---|---|---|
| Drifts steadily in one direction | wrong sign on pitch or roll, or missing yaw rotation | fly at 0.3 m, nudge one gain, watch direction |
| Circles or spirals around the goal | position error not rotated into the body frame, or `rot2` not transposed | section 5 |
| Oscillates slowly (1 s period) | latency or far too high $K_p$ with no damping | add $K_d$ on velocity, lower $K_p$ |
| Oscillates fast and buzzes | $K_d$ too high, or noisy numerical derivative | use odometry velocity, lower $K_d$ |
| Sinks slowly over a flight | battery sag, no integral term | add $K_{i,z}$ with anti-windup |
| Never lifts off | thrust scale wrong (0.6 sent where 40 000 was expected) or $T_0$ too low | section 3 warning |
| Shoots to the ceiling | thrust scale wrong the other way, or integrator wound up on the floor | clamp, saturate, zero the integrator until airborne |
| Spins at $\pm 180°$ | yaw error not wrapped | `angdiff` |

**Examiner questions to be ready for.** Why do you rotate the error by the transpose of `rot2(yaw)`? Why is there a minus sign on roll but not on pitch? What does the hover thrust feed-forward do, and why do you still need an integrator? Which loops run on the drone and which on your PC, and at what rates? What frame does Vicon report in, and how is it different from the NED frame in the slides? What happens if the drone leaves the Vicon volume?

---

## In a nutshell

Lab 3 puts you in charge of the outer loops of a quadrotor cascade: a Vicon motion-capture system tells you where the Crazyflie is, and twenty times a second you publish a `Twist` whose fields actually mean pitch, roll, thrust and yaw rate. Everything is in the ROS ENU convention with x forward, y left, z up and ZYX Euler angles, so height is positive up and positive roll moves the drone to its right. Before the x–y error becomes pitch and roll it must be rotated into the body frame with the transpose of the 2-D rotation by yaw, otherwise a yawed drone flies the wrong way. Altitude is a PID around a hover feed-forward, position is a PD with damping taken from the odometry velocity, yaw is a wrapped P loop, and every command is saturated. Confirm the thrust scale and the roll sign with tiny test flights, tune altitude before position, and keep a hand on the stop.

## Key terms

- **Crazyflie 2:** the 27-gram open-source quadrotor used in the lab.
- **Vicon motion capture:** ceiling infrared cameras that track reflective markers and publish the drone's pose as `Odometry`.
- **Odometry message:** ROS type carrying pose (position plus quaternion) and twist (linear and angular velocity).
- **Re-purposed Twist:** `linear.x` pitch, `linear.y` roll, `linear.z` thrust, `angular.z` yaw rate.
- **RollPitchYawrateThrust:** the `mav_msgs` message that defines that convention and its units.
- **ENU:** east-north-up world frame, z positive upward; the ROS and Vicon convention.
- **Body frame (x forward, y left, z up):** the frame pitch and roll commands act in.
- **z-y'-x'' (ZYX) Euler sequence:** yaw, then pitch, then roll, as in Lecture 1 roll-pitch-yaw.
- **`rot2(yaw)'`:** transpose of the 2-D rotation; maps world-frame vectors into the body frame.
- **Hover thrust $T_0$:** the feed-forward thrust that just balances weight.
- **Cascade control:** position error to angle reference to rate to torque to motors; you write the outer loops.
- **Saturation:** clamping a command to a safe range.
- **Anti-windup:** stopping the integrator from growing when the actuator is saturated or the drone is on the ground.
- **Battery sag:** falling voltage reduces thrust for the same command; the integrator compensates.
- **Leader–follower:** one drone's reference is another drone's position.

## Check yourself

1. Why must the position error be multiplied by the transpose of `rot2(yaw)` before it becomes pitch and roll commands?
2. The drone is yawed $-90°$ (nose along world $-y$) and the goal is 1 m along world $+x$. What is the error in the body frame, and does the drone pitch or roll?
3. Why does the altitude loop need both a hover feed-forward and an integral term?
4. Which four fields of the `Twist` message are used, and what does each mean on the Crazyflie?
5. The drone hovers fine but slowly drifts in a circle around the setpoint. Name two likely causes.

<details>
<summary>Answers</summary>

1. Pitch and roll tilt the drone along its own body axes. The goal error is measured in the world frame. The transpose of the rotation by yaw converts world-frame vectors into body-frame vectors, so the controller tilts towards where the goal actually is relative to the nose.
2. With $\psi = -90°$, $\cos\psi = 0$, $\sin\psi = -1$, so $e^B = [(0)(1) + (-1)(0),\ (1)(1) + (0)(0)] = (0, 1)$. The goal is 1 m to the drone's left; it must roll (negative roll in the convention of section 4), not pitch.
3. The feed-forward lets the drone leave the ground immediately with roughly the right thrust instead of waiting for the integrator; the integral then removes the slow offset caused by battery sag and small model errors that a P or PD loop would leave as a steady error.
4. `linear.x` pitch command, `linear.y` roll command, `linear.z` thrust, `angular.z` yaw rate.
5. The x–y error was not rotated into the body frame (or was rotated with the un-transposed matrix), or one of the pitch/roll signs is flipped so the correction is partly sideways.

</details>
