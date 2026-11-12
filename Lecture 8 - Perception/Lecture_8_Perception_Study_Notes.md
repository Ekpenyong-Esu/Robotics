# Lecture 8 – Perception (Sensors and Perception)

> **Source:** `08_R7010E_Lecture_8_Perception.pdf` (12 slides, course R7010E Robotics, LTU).
> ⚠️ The slide footers say "Lecture 7 – Perception" while the file and folder say Lecture 8. This is just a numbering mismatch in the deck; the content is the "Sensors and Perception" lecture from the course plan.

## What this lecture is about, and why it matters

This lecture is a tour of the hardware a robot uses to *sense*. Back in Lecture 1 you saw the definition of a robot as "a goal-oriented machine that can sense, plan and act autonomously", and the fundamental questions "Where am I?" and "What is the surrounding environment?". Sensors are how a robot gets any answer at all to those two questions. Without them the robot is blind, and nothing from the navigation or localization lectures can work.

The lecture itself is short and very visual: almost every slide is a photo of one sensor with a title, followed by a comparison of on-board computers and one dense slide about fusing many sensors together. So these notes do two jobs. They tell you exactly what was on each slide, and they fill in, in plain words, *what each sensor measures, how it roughly works, and what its weakness is*, because that is what you would be asked about.

⚠️ **Important honesty note:** slides 1 to 10 contain only a title and a photo. Everything written under "How it works" and "Strengths / weaknesses" in these notes is short background knowledge added to make the slide understandable. It is standard, textbook-level material (Corke's *Robotics, Vision and Control* covers most of it), but it was not spelled out in the deck. Check with your lecturer how deep the exam expects you to go.

## Main topics (roadmap)

1. Two big ideas you need first: proprioceptive vs exteroceptive sensing, and why every sensor is noisy.
2. External motion capture: Vicon MoCap (slide 2).
3. Inertial sensing: INS and IMUs (slide 3).
4. Ultrasound range sensors (slide 4).
5. Range finders: LiDARs (slide 5).
6. GPS (slide 6).
7. Optical flow sensors (slide 7).
8. Camera sensors (slide 8).
9. RGB-D sensors (slide 9).
10. Stereo camera sensors (slide 10).
11. Computational units: where the perception code runs (slide 11).
12. Sensor fusion with an Extended Kalman Filter, the MSF framework (slide 12).
13. Summary table, nutshell, key terms, self-check.

---

## 1. Two ideas to hold onto before we look at any hardware

### 1.1 Proprioceptive vs exteroceptive sensors

Robotics people sort sensors into two families, and the names sound scarier than they are.

A **proprioceptive sensor** measures something about the robot's *own body*: how fast its wheels turn, how it is accelerating, how it is rotating. The word comes from "proprio", meaning "one's own". Your inner ear is a proprioceptive sensor: with your eyes closed you still feel yourself tilting or spinning.

An **exteroceptive sensor** measures something about the *outside world*: how far away the wall is, what the scene looks like, where you are on the globe. Your eyes and ears are exteroceptive.

Why does this split matter? Because the two families fail in opposite ways. Proprioceptive sensors work anywhere (indoors, in the dark, in a tunnel) but their errors *pile up over time*, because you are integrating small measurements into a position estimate. Exteroceptive sensors give you an absolute fix on the world, but they depend on the environment cooperating: GPS needs sky, cameras need light, ultrasound needs a surface that bounces sound back. The whole point of the last slide (sensor fusion) is to marry the two so each covers the other's weakness.

⚠️ The slides never use the words proprioceptive and exteroceptive. They are standard vocabulary (used in Corke's book and in most robotics courses), so learn them, but the classification of each sensor below is my own.

### 1.2 Every sensor is noisy, and that is the whole reason Lecture 10 exists

No real sensor returns the true value. It returns the true value plus **noise** (random error that changes every reading) and often plus a **bias** (a steady offset that is wrong in the same direction every time). An analogy: a bathroom scale that jitters by half a kilo each time you step on it has noise; a scale that always reads two kilos heavy has bias.

In the localization lecture (Lecture 10) you will model this noise as a zero-mean Gaussian with a covariance matrix, and you will build an estimator, the Extended Kalman Filter (EKF), that weighs each sensor by how much you trust it. Keep that in the back of your mind as you go through the sensors: for each one, ask "what kind of error does it make?" That question is the bridge between this lecture and the next.

---

## 2. Slide 2 – Vicon MoCap (motion capture)

**What the slide shows:** a photo of a single Vicon camera mounted on the ceiling. It is a dark box with a ring of infrared LEDs around the lens (the pink ring) and a blue status light.

**What it is:** **Motion capture (MoCap)** is an *external* positioning system. Several of these cameras are bolted around a room, and small reflective balls (markers) are stuck onto the robot. Each camera flashes infrared light, the markers bounce it back, and the camera sees bright dots. Since each camera knows exactly where it is, the system triangulates the 3D position of every marker many times per second, and from a cluster of markers on one object it computes the full 6-DoF pose (position plus orientation).

**Why it is in a robotics course:** this is the "ground truth" system in a lab. It gives you millimetre-accurate pose at hundreds of hertz, with almost no drift, so you can test your controllers and localization algorithms against a reliable answer. The Crazyflie drones in your lab and the arena at LTU are typically flown under a MoCap system.

**Everyday analogy:** it is like a very precise indoor GPS that only works inside one room, because the "satellites" are the cameras on the ceiling.

**Limitations to remember:** it only works inside the volume the cameras can see, it needs markers on the robot, it is expensive, and it is cheating from the robot's point of view, since the robot itself is not sensing anything. In the field a robot has none of this, which is why the rest of the slides exist.

**Classification:** exteroceptive, but external to the robot (the sensor is in the room, not on the robot).

⚠️ The slide gives no description beyond the title. The explanation above is background.

---

## 3. Slide 3 – Inertial Navigation Systems (INS) / Inertial Measurement Units (IMUs)

**What the slide shows:** a small blue breakout board (it looks like the common MPU-6050 style module) with pins labelled VCC, GND, SCL, SDA, XDA, XCL, AD0, INT and little axis arrows printed on the board. SCL and SDA are I²C communication pins, which tells you the chip talks to a microcontroller over a simple two-wire bus.

### 3.1 What an IMU is

An **Inertial Measurement Unit (IMU)** is a chip that contains, at minimum, a three-axis **accelerometer** and a three-axis **gyroscope**. Many also include a three-axis **magnetometer**; then it is sometimes called a 9-axis IMU or MARG sensor.

- The **gyroscope** (gyro) measures **angular velocity**, how fast the body is rotating about each axis, in rad/s or deg/s. In Lecture 3 you saw ω and the skew-symmetric matrix in the derivative of a rotation matrix; the gyro is the sensor that gives you ω.
- The **accelerometer** measures **specific force**, which is linear acceleration *including gravity*. Sitting still on a table, it reads about 9.81 m/s² pointing up (it feels the table pushing it up against gravity). That is useful: the direction of gravity tells you which way is down, so you can get roll and pitch while stationary.
- The **magnetometer** measures the Earth's magnetic field, acting as a compass to give you heading (yaw).

An **Inertial Navigation System (INS)** is an IMU plus the mathematics that integrates its readings over time to track orientation, velocity and position. The IMU is the sensor; the INS is the sensor plus the estimation.

### 3.2 How an INS tracks motion (and why it drifts)

Here is the process, step by step.

1. Read the gyro: angular velocity ω.
2. Integrate ω over the time step Δt to update orientation (this is exactly the "incremental rotation" idea from Lecture 3, or the quaternion update `attitude = attitude * UnitQuaternion.omega(ω·Δt)` from the Lecture 3 MATLAB script).
3. Use the orientation to rotate the accelerometer reading into the world frame and subtract gravity, leaving true linear acceleration.
4. Integrate acceleration once to get velocity, and again to get position.

**Worked example of why this goes wrong.** Suppose the accelerometer has a tiny bias of 0.01 m/s² (it reads 0.01 too high all the time) and the robot is actually standing still.

- After 1 s, velocity error = 0.01 × 1 = 0.01 m/s, position error = ½ × 0.01 × 1² = 0.005 m. Harmless.
- After 10 s, velocity error = 0.1 m/s, position error = ½ × 0.01 × 100 = 0.5 m.
- After 60 s, position error = ½ × 0.01 × 3600 = 18 m.

The position error grows with the *square* of time because you integrate twice. This runaway is called **drift**. Orientation drifts too, but only linearly in time, because the gyro is integrated once.

**Everyday analogy:** walking with your eyes closed, counting steps and feeling your turns. You are fine for a few steps, but after a minute you have no idea where you are, and you have no way of noticing the error without opening your eyes. The "eyes" are the exteroceptive sensors that come next.

**Common mistake:** people say the accelerometer "measures acceleration". Strictly it measures *specific force*, acceleration plus the reaction to gravity. A body in free fall reads zero; a body sitting on a table reads +1 g. If you forget to subtract gravity you will think your drone is accelerating upward at 9.81 m/s² forever.

**Strengths:** very high update rate (hundreds to thousands of Hz), tiny, cheap, works anywhere, no external infrastructure. **Weaknesses:** drift, bias, sensitivity to vibration (relevant to drones; recall from Lecture 6 that low-level controllers run at 1 to 32 kHz partly to stay above frame-vibration frequencies), magnetometer confused by nearby metal and motors.

**Classification:** proprioceptive.

⚠️ The slide is a photo only. The description of gyro, accelerometer, magnetometer and drift is background.

---

## 4. Slide 4 – Ultrasound sensors

**What the slide shows:** a small green board with a single round black transducer on top, the typical hobby ultrasonic rangefinder (it resembles a MaxBotix module, which uses one transducer for both sending and receiving).

**What it measures:** the **range** (distance) to the nearest object in front of it.

**How it works: time of flight.** The sensor emits a short burst of sound above human hearing (ultrasound, about 40 kHz), then listens for the echo. Sound travels at roughly 343 m/s in air. The distance is

d = (c × t) / 2,

where c is the speed of sound and t is the round-trip time. You divide by 2 because the sound goes out *and* comes back.

**Worked example.** The echo returns after t = 5.8 ms. Then d = 343 × 0.0058 / 2 ≈ 0.99 m. The object is about one metre away.

**Everyday analogy:** shouting into a canyon and timing the echo. Bats and dolphins navigate this way.

**Strengths:** very cheap, simple, works in the dark and on glass or dark surfaces that fool cameras and some LiDARs. On drones a downward-facing ultrasound sensor is a classic way to measure height above ground (you will see one sitting next to a camera on the optical-flow board on slide 7).

**Weaknesses:**
- Wide beam (a cone of maybe 15 to 30 degrees), so you know *something* is about 1 m away but not exactly in which direction. This is poor angular resolution.
- Soft materials (curtains, foam) absorb sound and return weak echoes.
- Surfaces at a steep angle bounce the sound away instead of back, so the sensor sees nothing (specular reflection).
- Short range, typically a few metres.
- Slow compared with light: the sound needs a few milliseconds per reading, so update rates are low (tens of Hz).
- Speed of sound changes with temperature, which adds a small systematic error.

**Classification:** exteroceptive.

⚠️ The slide is a photo only. Operating principle and limitations are background.

---

## 5. Slide 5 – Range finder: LiDARs

**What the slide shows:** four devices side by side: a flat puck-shaped 2D scanning LiDAR (like the RPLIDAR used on the TurtleBot3 in your labs), a small Hokuyo URG scanner, a larger blue Hokuyo UTM scanner, and a two-lens LIDAR-Lite style single-beam rangefinder on the right.

**What LiDAR means:** **LiDAR** stands for **Light Detection and Ranging**. It does the same trick as ultrasound but with a laser pulse instead of a sound pulse: d = c × t / 2, now with c = the speed of light, 3 × 10⁸ m/s.

**Worked example.** An object 10 m away gives a round trip of 20 m, so t = 20 / (3 × 10⁸) ≈ 67 nanoseconds. That is why LiDAR electronics must be fast and precise, and also why LiDAR can take tens of thousands of measurements per second while ultrasound manages a few dozen.

⚠️ Background note: some LiDARs (including the cheap RPLIDAR) actually use *triangulation* (a laser dot viewed by a small camera at a known offset) rather than time of flight. The principle you should remember for exams is time of flight; triangulation is a cheaper variant with shorter range.

**Single beam vs scanning.** The device on the right measures distance along one line only. The other three spin that beam (a rotating mirror or the whole head rotates) and take a reading every fraction of a degree, so they produce a **2D laser scan**: a list of (angle, range) pairs covering, for example, 360° at 1° steps. That scan is exactly the data you plotted in the Lecture 5 MATLAB script with `rosReadCartesian(scan)` from the TurtleBot's `/scan` topic, and it is what the Lindqvist potential-field method in Lecture 7 used, treating each LiDAR point inside a radius as a repulsive force. 3D LiDARs stack many beams at different vertical angles to produce a **point cloud**.

**Converting a scan to points.** Each reading is a polar coordinate. To get a point in the robot frame you use

x = r cos θ, y = r sin θ.

So a reading of r = 2 m at θ = 30° gives x = 1.73 m, y = 1.0 m. The Lecture 10 script does exactly this with `pol2cart(theta, r)` on the Killian dataset scans.

**Strengths:** accurate (centimetres or better), narrow beam so excellent angular resolution, long range (tens to hundreds of metres), fast, independent of lighting. This is why LiDAR is the workhorse for the occupancy grids and SLAM you meet in Lectures 7 and 10.

**Weaknesses:** 2D scanners only see one slice of the world (an obstacle above or below the scan plane is invisible), glass and mirrors are trouble (the laser passes through or reflects away), rain, dust and fog scatter the beam, and 3D units are heavy and expensive, which matters for small drones.

**Classification:** exteroceptive.

⚠️ The slide is a photo only. Everything beyond "these are LiDARs" is background.

---

## 6. Slide 6 – GPS

**What the slide shows:** a Parallax GPS receiver board with a square ceramic patch antenna (the beige square) on top.

**What GPS is:** the **Global Positioning System**, one of several **GNSS** (Global Navigation Satellite Systems; GLONASS, Galileo and BeiDou are others). A constellation of satellites continuously broadcasts the time and its own position. The receiver measures how long each signal took to arrive, converts that to a distance (c × t, one way this time, since the satellite does the transmitting), and with four or more satellites it solves for its own latitude, longitude, altitude and clock offset. This is **trilateration**: finding where spheres of known radius around known centres intersect.

**Why four satellites, not three?** Three spheres pin down a point in 3D, but the receiver's cheap clock is not synchronised with the satellites' atomic clocks, so there is a fourth unknown (the clock error). Four equations, four unknowns.

**Everyday analogy:** three friends each tell you "you are exactly 5 km from me", "3 km from me", "7 km from me", and you know where they stand. Only one spot on the map satisfies all three.

**Strengths:** absolute position anywhere on Earth with a clear sky, no drift, cheap receivers.

**Weaknesses:** accuracy of only a few metres for a consumer receiver (differential or RTK GPS can get to centimetres but needs a base station), low update rate (1 to 10 Hz), useless indoors, in tunnels, underground (think of the mine reconstruction slides in Lecture 10) and degraded in cities where tall buildings block or reflect the signal (multipath). It gives position but not orientation, and a weak sense of velocity.

**Classification:** exteroceptive.

⚠️ The slide is a photo only. The operating principle is background.

---

## 7. Slide 7 – Optical flow

**What the slide shows:** a board with a camera lens on the left and a round ultrasonic transducer on the right, the classic PX4FLOW board used on drones.

**What optical flow is:** **optical flow** is the apparent motion of image features between two consecutive camera frames. If you film the floor from a drone and the carpet pattern slides 10 pixels to the left between frames, the drone has moved to the right.

**Why the ultrasound is on the same board.** Pixels tell you *angular* motion only. A shift of 10 pixels could mean you moved 1 cm at 10 cm height or 1 m at 10 m height. Multiply the pixel velocity by the distance to the surface, and you get true metric velocity. The ultrasound sensor supplies that distance. In formula terms,

v ≈ (pixel shift / focal length in pixels) × height / Δt.

**Worked example.** The image shifts 20 pixels between frames taken 0.02 s apart, the camera focal length is 400 pixels, and the ultrasound says the floor is 1.5 m below. Angular motion is 20/400 = 0.05 rad. Linear motion is 0.05 × 1.5 = 0.075 m. Velocity is 0.075 / 0.02 = 3.75 m/s.

**Everyday analogy:** looking out of a train window. Nearby fence posts whip past; far mountains barely move. Same speed, different pixel motion, because of distance.

**Where it sits in the picture:** optical flow gives *velocity*, not position. It is a drift-fighter for the IMU: the IMU's velocity estimate drifts, and the flow sensor keeps correcting it. Integrate flow velocity and you get position that drifts too, but much more slowly than double-integrated acceleration. On drones it enables stable hovering indoors without GPS.

**Weaknesses:** needs a textured surface (a plain white floor gives nothing to track), needs light, and rotation of the camera also moves pixels, so the IMU gyro must be used to subtract the rotational part before you read off translation.

**Classification:** exteroceptive (it looks at the world), but its output behaves like odometry.

⚠️ The slide is a photo only. The explanation of optical flow is background.

---

## 8. Slide 8 – Camera sensors

**What the slide shows:** a small board-level industrial camera, a bare circuit board with a lens, of the kind you bolt into a robot.

**What a camera gives you:** a **monocular camera** (one lens) captures a 2D array of brightness or colour values. Lecture 11 will treat this properly: how a pinhole camera projects a 3D world point onto a 2D image plane, how colour is sensed with a Bayer grid, and how you extract information such as detecting a person with HOG features.

**The single most important limitation:** a single image has **no depth**. A perspective projection throws away the distance along the viewing ray. A small object nearby and a large object far away can make the same picture. To recover depth you need motion (structure from motion, visual odometry), a second camera (slide 10), or an active depth sensor (slide 9).

**Strengths:** enormous information per reading (texture, colour, text, semantics), cheap, light, passive (emits nothing, so many robots can use cameras in one room without interfering). **Weaknesses:** needs light, motion blur, heavy processing load (this is why the computational units on slide 11 matter), no direct depth.

**Classification:** exteroceptive.

⚠️ The slide is a photo only; Lecture 11 is where the course actually explains cameras.

---

## 9. Slide 9 – RGB-D sensors

**What the slide shows:** a thin blue bar with an Intel logo, an Intel RealSense depth camera.

**What RGB-D means:** **RGB-D** = Red, Green, Blue plus **Depth**. The sensor delivers a normal colour image and, aligned with it pixel by pixel, a **depth image** where each pixel stores the distance to the surface it sees. Put the two together and every pixel becomes a coloured 3D point, so you get a dense coloured point cloud for free.

**How depth is obtained (background):** RGB-D cameras are *active*: they project their own infrared light. Two common principles exist.
- **Structured light:** project a known infrared pattern (dots or stripes) and measure how it deforms on the scene; the deformation reveals depth by triangulation. The original Microsoft Kinect worked this way.
- **Active stereo:** two infrared cameras plus an infrared projector that paints texture onto blank surfaces so stereo matching (next slide) works even on a white wall. Most RealSense models do this.
- Some devices instead use per-pixel **time of flight**, like a LiDAR with a whole image sensor instead of one beam.

**Everyday analogy:** the face-unlock on many phones. It shines invisible dots on your face and reads the 3D shape, which is why a flat photo does not fool it.

**Strengths:** dense depth plus colour, fast (30 fps), affordable, great for indoor 3D mapping and object manipulation. **Weaknesses:** short range (roughly 0.3 to 10 m), the infrared pattern is washed out by bright sunlight so outdoor use is poor, depth noise grows with distance, and the projector gives it away as *active* (several in one room can interfere).

**Classification:** exteroceptive, active.

⚠️ The slide is a photo only. Operating principles are background.

---

## 10. Slide 10 – Stereo camera sensors

**What the slide shows:** a long narrow board with two lenses at the ends, a stereo camera (it resembles a DUO or ZED style module). The distance between the two lenses is the **baseline**.

**The idea:** two cameras a known distance apart look at the same scene. A point in the world lands at slightly different horizontal positions in the left and right images. That horizontal difference is the **disparity**. The closer the object, the bigger the disparity. This is **depth from stereo**, and Lecture 11 lists the pipeline: camera calibration, image rectification, stereo matching, then disparity to depth by triangulation.

**The formula:**

Z = f × b / d,

where Z is depth, f is focal length in pixels, b is the baseline in metres and d is disparity in pixels.

**Worked example.** f = 700 px, b = 0.12 m, and a feature appears at column 350 in the left image and column 322 in the right image, so d = 28 px. Then Z = 700 × 0.12 / 28 = 3.0 m. If the same feature had d = 14 px it would be 6 m away. Notice that halving the disparity doubles the depth: at long range one pixel of disparity error means a big depth error, which is why stereo depth is accurate up close and poor far away.

**Everyday analogy:** hold a finger in front of your face and close one eye, then the other. The finger jumps a lot against the background. Now do it with a distant lamp post: it barely jumps. Your brain does Z = f·b/d constantly; your eyes are a stereo pair with a baseline of about 6.5 cm.

**Stereo vs RGB-D, a common confusion.** Both give depth images. Stereo is *passive*: it only uses ambient light, so it works outdoors in sunshine where RGB-D projectors fail, and at longer ranges if the baseline is large. But it needs *texture*: on a blank white wall there is nothing to match between the two images, so you get no depth. RGB-D solves the blank-wall problem by projecting its own texture, at the cost of failing in sunlight and at range. Lecture 11 lists the stereo failure modes explicitly: calibration errors, poor resolution, occlusions (a point visible in one camera but hidden in the other), violation of brightness constancy, and low-contrast regions.

**Classification:** exteroceptive, passive.

⚠️ The slide is a photo only. The formula and failure modes are background, consistent with Lecture 11.

---

## 11. Slide 11 – Computational units

This is the first slide with real text. It compares four on-board computers you might put on a robot.

| Board | Processor | RAM | Rough class |
|---|---|---|---|
| Raspberry Pi 3 B+ | Broadcom BCM2837B0, Cortex-A53 (ARMv8) 64-bit SoC @ 1.4 GHz | 1 GB LPDDR2 SDRAM | low-power ARM single-board computer |
| Up Board | Intel Atom x5 Z8350, 64-bit @ 1.44 GHz (up to 1.92 GHz) | 4 GB DDR3L | low-power x86 single-board computer |
| Odroid XU4 | Samsung Exynos5 Octa: ARM Cortex-A15 quad @ 2 GHz + Cortex-A7 quad @ 1.3 GHz | 2 GB LPDDR3 | faster ARM board (big.LITTLE 8-core) |
| Intel NUC | Dual-core Intel Core i7-5557U with multi-threading @ 3.10 GHz | 16 GB | small desktop-class PC |

**Why this slide is in a *perception* lecture.** Sensors produce data; something has to process it. A LiDAR scan is thousands of points per second, a camera is millions of pixels per frame, and stereo matching or HOG person detection is heavy arithmetic. The board you choose decides which perception algorithms you can run in real time.

Some things to understand about the table.

- **ARM vs x86.** The Pi and Odroid use ARM processors (the kind in phones): low power, low heat, but less raw speed and sometimes less software support. The Up Board and NUC use Intel x86 processors (the kind in laptops): more speed and the same software as your desktop, at the price of more power draw.
- **Two processors in the Odroid.** The Exynos5 Octa pairs four fast Cortex-A15 cores with four efficient Cortex-A7 cores. The system runs light tasks on the small cores and heavy tasks on the big ones to save battery.
- **RAM matters for perception.** Point clouds, maps and images are large. 1 GB on the Pi fills up fast; 16 GB on the NUC lets you run full SLAM and vision pipelines.
- **The trade-off is weight and power.** A NUC needs a bigger battery and is heavy for a small drone; a Pi fits on almost anything. Recall from Lecture 6: the low-level attitude controller runs on a tiny microcontroller at kHz rates, while the high-level position and perception code runs at 10 to 100 Hz on a board like these. The TurtleBot3 Burger in your labs carries a Raspberry Pi for exactly that high-level role.

**Common mistake:** comparing boards by clock speed alone. The Pi at 1.4 GHz and the Odroid at 2 GHz are far slower per core than the i7 at 3.1 GHz, because the i7 has a much more capable core design, multi-threading and far more memory bandwidth. Clock frequency is only one factor.

---

## 12. Slide 12 – Sensor fusion (Lynen et al., 2013, the MSF framework)

This is the dense slide, and it is the bridge to Lecture 10. The slide reproduces a figure from *Lynen, Achtelik, Weiss, Chli & Siegwart, "A robust and modular multi-sensor fusion approach applied to MAV navigation", IROS 2013*. MAV means Micro Aerial Vehicle, a small drone.

### 12.1 The problem it solves

You have now seen a drawer full of sensors, each with a different weakness: the IMU drifts, the camera is slow and has no scale, GPS is coarse and dies indoors. **Sensor fusion** is combining all of them into one estimate that is better than any single sensor. The standard tool is the **Extended Kalman Filter (EKF)**, which Lecture 10 derives. Here you just need the shape of it.

### 12.2 The two boxes on the slide

The figure has a left box called **ssf_core** (the filter itself) and a right box called **ssf_updates** (plug-in modules, one per sensor type). The split is the "modular" part of the title: the core never changes, and you add a small module for each new sensor (the slide lists `position_sensor`, `pose_sensor`, `*_sensor` as examples).

**The core runs two alternating steps.**

**Prediction** (driven by the IMU, which is the fast sensor):
- Input `u = [ω a]`: the gyro's angular velocity and the accelerometer's acceleration.
- State `x = [p_w^i, v_w^i, q_w^i, b_ω, b_a, λ, q_v^w, q_c^i, p_c^i]`. Read it as: position, velocity and orientation (as a quaternion) of the IMU in the world frame; the gyro bias and the accelerometer bias; a scale factor λ; and some calibration quantities (the rotation between the vision frame and the world frame, and the rotation and translation between the camera and the IMU).
- `ẋ = f(x, u)`: the nonlinear motion model, the INS integration from Section 3.2.
- `P_{k+1} = F P_k Fᵀ + Q`: the covariance grows. P is the uncertainty of the state, F is the Jacobian of f, and Q is process noise. In plain words: each time you predict with the IMU alone, you become *less* sure.

**Update** (driven by a slower exteroceptive sensor, such as a camera pose or GPS position):
- The sensor delivers a measurement `z`, and the module writes a linearised measurement model `z_lin = H x + R`, where H says how the measurement depends on the state and R is the measurement noise covariance.
- `y = z − h(x)`: the **innovation**, the difference between what the sensor actually said and what the filter predicted it would say.
- `S = H P Hᵀ + R`: the innovation covariance, how surprised you should be.
- `K = P Hᵀ S⁻¹`: the **Kalman gain**, how much to trust the sensor versus the prediction.
- `x̄ = x + K y`: correct the state.
- `P̄ = (I − K H) P`: shrink the uncertainty.

**Step-by-step in words.** Imagine a drone with an IMU at 200 Hz and a camera pose at 20 Hz. Between camera frames the filter predicts ten times using only the IMU, and P grows each time. When a camera pose arrives, the filter compares it to the prediction (y), decides how much to believe it (K), nudges the state toward it, and shrinks P. Then prediction resumes. The camera caps the IMU's drift; the IMU fills in the gaps between camera frames at high rate. That is sensor fusion.

**Everyday analogy:** the IMU is your sense of your own steps while walking with eyes closed; every few seconds you open your eyes briefly (the camera) and correct where you thought you were. The Kalman gain is how much you trust the glimpse versus your step-counting.

### 12.3 Why biases are *in the state*

Notice b_ω and b_a sit inside x. Instead of treating bias as a fixed number from the datasheet, the filter *estimates* it live. Every time the camera corrects the position, some of that correction flows into the bias estimate, so the IMU prediction gets better over time. This is the key trick that makes cheap IMUs usable.

### 12.4 The frame diagram on the right

The right half of the slide shows the frames involved: a **World** frame, a **Vision** frame (where the camera-based pose estimate lives), and a rigid **Camera-IMU body** carrying an IMU frame and a Camera frame. Coloured arrows show the transforms: red for world to IMU (the pose being estimated), green for IMU to camera (the fixed mounting offset `q_c^i, p_c^i`), blue for vision to camera, and grey for world to vision (with `p_v^w = 0`).

This is the Lecture 1 pose-graph idea in action: you can go from World to Camera two ways, World → IMU → Camera, or World → Vision → Camera, and both must agree. Writing that loop as an equation is exactly what h(x) does. The filter estimates the unknown links (IMU pose, camera-IMU offset, vision-to-world rotation) so that the loop closes.

**Why the scale λ?** A single camera cannot tell the size of the world (recall Section 8: no depth from one image). Monocular visual odometry therefore outputs a pose that is correct up to an unknown scale factor. The filter estimates λ by comparing the camera's motion with the accelerometer, which does know metric units.

**Common mistake:** thinking fusion means averaging the sensors. It is a *weighted* combination where the weights (via K) change every step according to the current uncertainties, and the sensors are used in different roles (IMU for prediction, the others for correction).

⚠️ The slide shows only the figure, with no narration. The explanation of each symbol follows the cited paper and the standard EKF, which Lecture 10 presents in full. If any of this feels heavy, read it again after Lecture 10.

---

## 13. Putting it together: the sensor comparison table

| Sensor | What it measures | Principle (background) | Type | Best at | Weak at |
|---|---|---|---|---|---|
| Vicon MoCap | 6-DoF pose of markers | Multi-camera IR triangulation | External, exteroceptive | Lab ground truth, mm accuracy, high rate | Only inside the camera volume, needs markers, not on the robot |
| IMU / INS | Angular velocity, specific force (+ magnetic field) | MEMS gyro, accelerometer, magnetometer | Proprioceptive | Very fast, works anywhere, orientation | Drift (position error grows with t²), bias, vibration |
| Ultrasound | Range to nearest object | Sound time of flight, d = ct/2 | Exteroceptive, active | Cheap, dark, glass, height above ground | Wide beam, soft/angled surfaces, short range, slow |
| LiDAR | Range at many angles (scan or point cloud) | Laser time of flight (or triangulation) | Exteroceptive, active | Accurate, long range, light-independent, mapping and SLAM | Glass/mirrors, dust and fog, 2D sees one slice, 3D heavy and costly |
| GPS / GNSS | Absolute position on Earth | Satellite trilateration, 4+ satellites | Exteroceptive | Global, drift-free position outdoors | Metre accuracy, 1–10 Hz, fails indoors and in canyons, no orientation |
| Optical flow | Velocity over a surface | Pixel motion × height / time | Exteroceptive (odometry-like) | Hover and velocity without GPS | Needs texture and light, needs height sensor, rotation must be removed |
| Camera | 2D image (colour/intensity) | Perspective projection | Exteroceptive, passive | Rich information, cheap, light | No depth, needs light, heavy processing |
| RGB-D | Colour + per-pixel depth | IR structured light / active stereo / ToF | Exteroceptive, active | Dense indoor 3D, 30 fps | Sunlight, short range, interference |
| Stereo camera | Per-pixel depth from disparity | Z = f·b/d | Exteroceptive, passive | Outdoor depth, scalable with baseline | Textureless surfaces, depth error grows with range, calibration |

---

## In a nutshell

A robot answers "Where am I?" and "What is around me?" only through its sensors, and this lecture is a catalogue of the main ones. Proprioceptive sensors such as the IMU measure the robot's own motion at very high rates but drift, because you integrate noisy readings over time. Exteroceptive sensors such as ultrasound, LiDAR, GPS, cameras, RGB-D and stereo measure the outside world and do not drift, but each only works when the environment cooperates: enough light, texture, sky, or a surface that reflects. Processing all that data needs an on-board computer, and the choice from a Raspberry Pi up to an Intel NUC trades speed and memory against weight and power. Because no single sensor is good enough, real systems fuse them with an Extended Kalman Filter, as in the MSF framework: the IMU drives fast predictions, slower sensors correct them, and even the sensor biases are estimated on the fly. Lecture 10 shows you the mathematics behind that filter.

## Key terms

- **Perception:** the robot's process of turning raw sensor readings into knowledge about itself and its surroundings.
- **Proprioceptive sensor:** a sensor that measures the robot's own body state (wheel speed, acceleration, rotation rate).
- **Exteroceptive sensor:** a sensor that measures the external world (distance to objects, images, global position).
- **Active vs passive sensor:** an active sensor emits energy (sound, laser, infrared) and reads the return; a passive one only receives what is already there.
- **Noise and bias:** noise is random error that varies each reading; bias is a steady offset in the same direction.
- **Drift:** the growth of position or orientation error over time when noisy measurements are integrated.
- **Motion capture (MoCap):** an external multi-camera system (for example Vicon) that tracks reflective markers to give precise 6-DoF pose in a lab.
- **IMU (Inertial Measurement Unit):** a chip with a 3-axis gyroscope and 3-axis accelerometer, often plus a magnetometer.
- **INS (Inertial Navigation System):** an IMU plus the integration mathematics that tracks orientation, velocity and position.
- **Gyroscope:** measures angular velocity (rate of rotation).
- **Accelerometer:** measures specific force, linear acceleration including the reaction to gravity.
- **Magnetometer:** measures the magnetic field, used as a compass for heading.
- **Time of flight:** measuring distance from the travel time of a pulse, d = c·t/2.
- **Ultrasound sensor:** an active range sensor using sound pulses above human hearing.
- **LiDAR (Light Detection and Ranging):** an active range sensor using laser pulses; scanning versions produce 2D scans or 3D point clouds.
- **Laser scan / point cloud:** a set of (angle, range) readings from a 2D LiDAR, or a set of 3D points from a 3D sensor.
- **GPS / GNSS:** satellite systems giving absolute position by trilateration from four or more satellites.
- **Trilateration:** finding a position from known distances to known points.
- **Optical flow:** the apparent motion of image content between frames, used with a height measurement to get velocity.
- **Monocular camera:** a single camera; gives a 2D image with no direct depth.
- **RGB-D sensor:** a camera giving colour plus a per-pixel depth image, usually by projecting infrared light.
- **Stereo camera:** two cameras with a known baseline; depth from disparity, Z = f·b/d.
- **Baseline:** the distance between the two cameras in a stereo pair.
- **Disparity:** the horizontal pixel shift of a feature between left and right stereo images.
- **Single-board computer:** a small complete computer (Raspberry Pi, Up Board, Odroid) used as the robot's brain.
- **Sensor fusion:** combining several sensors into one estimate that is better than any single sensor.
- **EKF (Extended Kalman Filter):** the standard fusion estimator for nonlinear systems; alternates prediction and update steps.
- **Prediction step:** propagating the state with the motion model (here from IMU data); uncertainty P grows.
- **Update step:** correcting the state with a measurement; uncertainty P shrinks.
- **Innovation (y):** measured value minus predicted measurement, z − h(x).
- **Kalman gain (K):** the weight given to the measurement in the correction, K = P Hᵀ S⁻¹.
- **Covariance matrix (P):** the matrix describing how uncertain each state element is and how the uncertainties are correlated.
- **MSF (Multi-Sensor Fusion framework):** Lynen et al.'s modular EKF where a fixed core is extended by per-sensor update modules.

## Check yourself

1. Explain the difference between a proprioceptive and an exteroceptive sensor, give one example of each from the lecture, and say how their typical errors differ.
2. An IMU's accelerometer has a constant bias of 0.02 m/s². Starting from rest, how large is the position error after 30 seconds if nothing corrects it? What does this tell you about using an IMU alone?
3. An ultrasound sensor hears its echo 3.5 ms after sending the pulse. How far away is the object (take the speed of sound as 343 m/s)? Why would a LiDAR give a much better reading of the obstacle's *direction*?
4. A stereo camera has focal length 800 px and baseline 0.10 m. A feature shows a disparity of 16 px. What is its depth? Why does stereo accuracy get worse for distant objects, and what weakness does an RGB-D sensor fix compared to stereo?
5. In the MSF sensor-fusion slide, which sensor drives the prediction step and which kinds of sensors drive the update step? What happens to the covariance P in each step, and why are the IMU biases included in the state vector?

<details>
<summary>Answers</summary>

1. A proprioceptive sensor measures the robot's own body (for example the IMU's gyroscope measuring rotation rate); an exteroceptive sensor measures the external world (for example a LiDAR measuring distance to walls, or GPS measuring global position). Proprioceptive readings are fast and work anywhere but their errors accumulate when integrated, producing drift. Exteroceptive readings give an absolute reference that does not drift, but they depend on the environment: light, texture, clear sky, reflective surfaces. Fusion combines them so each covers the other's weakness.

2. Position error from a constant acceleration bias is ½·b·t² = 0.5 × 0.02 × 30² = 0.5 × 0.02 × 900 = 9 m. After only half a minute the robot thinks it is 9 m from where it really is. The error grows with the square of time because acceleration is integrated twice, so an IMU alone is unusable for position over more than a few seconds and must be corrected by an exteroceptive sensor.

3. d = c·t/2 = 343 × 0.0035 / 2 ≈ 0.60 m. The ultrasound beam is a wide cone (around 15–30°), so you know something is 0.6 m away somewhere within that cone but not exactly where. A LiDAR's laser beam is very narrow and a scanner takes a reading at each small angular step, so it tells you the distance along a precise direction and builds a detailed outline of the obstacle.

4. Z = f·b/d = 800 × 0.10 / 16 = 5 m. Depth is inversely proportional to disparity, so at long range the disparity is only a few pixels and a one-pixel matching error changes the depth by a large amount. Stereo also fails on textureless surfaces (nothing to match). An RGB-D sensor projects its own infrared pattern onto the scene, so it can measure depth on a blank wall; in exchange it struggles in bright sunlight and at longer ranges.

5. The IMU drives the prediction step: its angular velocity and acceleration (u = [ω a]) are integrated through the motion model ẋ = f(x, u), and the covariance grows, P ← F P Fᵀ + Q, because predicting without external information makes you less certain. Exteroceptive sensors such as a camera pose estimate or a GPS position drive the update step: the innovation y = z − h(x) is weighted by the Kalman gain K = P Hᵀ S⁻¹, the state is corrected by K·y, and the covariance shrinks, P ← (I − K H) P. The gyro and accelerometer biases are in the state so the filter can estimate them online from the corrections; that makes subsequent IMU predictions more accurate and keeps drift small between updates.

</details>
