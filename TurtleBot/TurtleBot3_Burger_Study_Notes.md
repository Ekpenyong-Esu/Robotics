# TurtleBot3 Burger – Study Notes (the lab robot)

> Built from the ROBOTIS quick-start guide `TURTLEBOT3_Burger_QS_Ver1815_low(web).pdf` (42 pages; the identical file is also in the Lab 2 folder as `TURTLEBOT3_MANUAL.pdf`), the Lab 2 handout `R7010_lab2.pdf`, the Python template `turtle_controller.py` and the MATLAB script `TurtleBot3.m` from Lecture 5. The quick-start guide is mostly pictures (an assembly manual), so where these notes explain *how* a part works rather than *where it goes*, that is added background and is marked as such.

## What this is about, and why it matters

The TurtleBot3 Burger is the small two-wheeled ground robot you drive in Lab 2 and that Lectures 4 and 5 keep pointing at. These notes tell you what is physically inside it, how to handle it without breaking it (or the battery), how it is wired so you can understand where the `/odom`, `/scan` and `/cmd_vel` data come from, and how you bring it up and talk to it from MATLAB or Python. If you understand the robot as a machine, the control theory from Lecture 4 stops being abstract: every line of your controller ends up as two wheel speeds on this robot.

## Roadmap: the main topics

1. What the TurtleBot3 Burger is and what it is for.
2. Anatomy, layer by layer: motors, battery, OpenCR board, Raspberry Pi, LiDAR.
3. The full parts list (what is in the kit).
4. Handling the key parts: screws, rivets, cables, the Li-Po battery and charger, the laser sensor.
5. Safety precautions (from the guide and from the lab handout).
6. How it moves: differential drive, and how that ties back to Lecture 4.
7. How the hardware becomes ROS data: the signal chain from sensor to topic.
8. Bringing the robot up in Lab 2: Wi-Fi, SSH, `run`, `teleop`, Domain ID.
9. Talking to it from MATLAB (ROS 2) and from Python (ROS 1).
10. The Lab 2 tasks and which lecture idea each one uses.
11. Common mistakes.

---

## 1. What the TurtleBot3 Burger is

The lab handout describes it in one sentence: *a small, affordable, programmable mobile robot for education, research, hobbies and prototyping*. It is **modular**: a stack of plastic "waffle plates" held apart by metal pillars, so you can rearrange it or add parts (the handout mentions 3D printing and optional computers and sensors). The things that make it a *robot* rather than a toy car are:

- a **single-board computer** (SBC) running Linux and ROS, so it can run your algorithms;
- a **360° laser distance sensor** (a small LiDAR) so it can see the room;
- **smart servo motors** with built-in encoders, so it can measure how far its wheels have turned.

**Everyday analogy:** think of it as a three-layer cake on wheels. The bottom layer is the muscles and the food (motors and battery), the middle layers are the nervous system and brain (the two circuit boards), and the top layer is the eyes (the spinning laser).

In this course the Burger is the platform for **Lab 2 (Mobile Robot Motion and Control)** and it appears in Lecture 4 (the differential-drive model is its kinematics) and Lecture 5 (the ROS topics and the MATLAB bring-up script are written for it).

---

## 2. Anatomy, layer by layer

The guide's exploded drawing (page 8 of the assembly manual) shows four layers from bottom to top. The assembly instructions then build each layer in turn. Here is what sits on each.

### 2.1 First layer: motors, wheels, caster and battery

- **Two DYNAMIXEL XL430 servo motors.** A DYNAMIXEL is a "smart" servo: a motor, gearbox, position sensor and controller chip in one box, talking over a serial cable rather than being driven by raw voltage. The guide tells you to *notice the ID of each DYNAMIXEL and the orientation of the wheel* when you mount them. Each motor has a numeric ID (the drawing shows ID 1 and ID 2), and the controller board uses that ID to know which wheel is which. Swap them and the robot drives backwards when you ask it to go forwards.
- **Two wheels with rubber tyres**, each bolted straight onto a DYNAMIXEL output with four small M2×4 bolts. There is no gearbox or axle between the motor and the wheel: the motor *is* the axle.
- **One ball caster** underneath the first plate. This is a free-rolling ball that acts as the third contact point. Two driven wheels plus one passive caster means the robot always sits stably on three points, and it can spin on the spot.
- **The Li-Po battery** lying between the two motors, held by five right-angle brackets. The label in the guide's photo reads 11.1 V, 1800 mAh (a three-cell lithium polymer pack). The bolts for the second layer are inserted *from underneath* so that no bolt tips poke down into the battery.

### 2.2 Second layer: the OpenCR 1.0 board

- **OpenCR 1.0** is the blue circuit board. It is the robot's **embedded controller**: it receives the DYNAMIXEL cables from the two motors, takes power from the battery (through the **Li-Po battery extension cable**), and has the round power jack where the SMPS mains adapter plugs in. It talks to the Raspberry Pi over a USB cable.
- It sits on four **PCB supports** (little plastic arms with a captive nut) and is fixed with rivets. The guide tells you to pass the DYNAMIXEL cables through the plate's **cable passage** before fitting the supports, otherwise the cables cannot reach.
- The DYNAMIXELs are fastened from the top of this layer with longer T2.6×12 screws, so the motors are actually clamped between layers one and two.

> **Background (not in the guide):** the OpenCR also carries an inertial measurement unit (gyroscope and accelerometer). The robot's odometry fuses the wheel encoders with this gyro to get its heading. ⚠️ The quick-start guide does not mention the IMU at all; check the ROBOTIS e-manual or ask the lab assistant if you need that detail.

### 2.3 Third layer: the Raspberry Pi 3 and the LiDAR adapter

- **Raspberry Pi 3** is the single-board computer (SBC). It runs Linux and ROS, and it is what you SSH into in Lab 2. It has the Ethernet port and the four USB ports you see on the exploded drawing.
- **USB2LDS** is a small blue adapter board. The laser sensor speaks serial; this board turns that into USB so the Pi can read it. It is riveted to an **adapter plate** on the waffle plate.
- Cables on this layer: a **USB cable** from OpenCR to the Pi, a **USB cable** from the Pi to USB2LDS, and the **Raspberry Pi power cable** from the OpenCR to the Pi's GPIO header.
- ⚠️ **Caution printed in the guide:** the Pi power cable goes on GPIO **pin 4 (+5 V)** and **pin 6 (ground)**. Getting this wrong can destroy the Pi. In the lab you will never need to touch this, but it is the one wiring step the guide flags with a warning triangle.

### 2.4 Fourth layer: the 360° Laser Distance Sensor (LDS-01)

- The **LDS-01** is the black disc on top. It is a spinning **LiDAR** (Light Detection And Ranging): a laser range-finder that rotates to measure the distance to the nearest object in every direction, 360 degrees around the robot. The rotation is driven by a small **belt** you can see on the side.
- It sits on four **spacers** on top of PCB supports, fixed with longer M2.5×16 bolts, and its cable runs down through a cable passage to the USB2LDS board on the third layer.
- Being on top with nothing in the way is deliberate: the laser needs a clear view all the way round.

### 2.5 The skeleton: waffle plates, pillars and fixings

Every layer is two **waffle plates** clipped side by side (the kit has eight plates for four layers). The layers are separated by hollow metal pillars called **plate supports**: 35 mm ones between layers one and two, 45 mm ones higher up. Small screws (M3×8) go through each plate into the pillars. A few conventions from the guide are worth remembering because they are the sort of thing a lab assistant will ask about:

- Layers one, three and four take their M3 bolts from the **top**; layer two takes them from **underneath**, so nothing sharp faces the battery.
- Boards never sit directly on a plate. They sit on **PCB supports** (with an M2.5 nut pressed into the slot) and are fixed with **rivets** or small bolts.
- Cables always go through a **cable passage** *before* the part they feed is fitted.

---

## 3. The parts list (pages 29–30 of the assembly manual)

You will not assemble a robot in this course, but the parts list is the quickest way to learn the names you will hear in the lab.

| Part | Quantity | What it is for |
|---|---|---|
| Waffle-Plate | 8 | The structural plates, two per layer |
| OpenCR 1.0 | 1 | Embedded controller board, drives the motors, powers the Pi |
| Raspberry Pi 3 | 1 | Single-board computer running Linux and ROS |
| 360 Laser Distance Sensor LDS-01 | 1 | The LiDAR |
| DYNAMIXEL (XL430) | 2 | Smart servo motors, one per wheel |
| Wheel, Tire | 2 each | Drive wheels |
| Ball Caster | 1 | Passive third contact point |
| Li-Po Battery | 1 | 11.1 V, 1800 mAh power pack |
| USB2LDS | 1 | Serial-to-USB adapter for the LiDAR |
| USB Cable | 2 | OpenCR to Pi, Pi to USB2LDS |
| DYNAMIXEL to OpenCR Cable | 2 | Motor data and power |
| Raspberry Pi 3 Power Cable | 1 | OpenCR to Pi GPIO pins 4 and 6 |
| Li-Po Battery Extension Cable | 1 | Battery to OpenCR |
| PCB Support | 12 | Stand-offs for the three boards |
| Adapter Plate | 1 | Mount for USB2LDS |
| Bracket | 5 | Hold the battery in place |
| Plate Support M3×35 mm | 4 | Pillars between layers one and two |
| Plate Support M3×45 mm | 10 | Pillars for the upper layers |
| Spacer | 4 | Lift the LiDAR off its supports |
| Rivet | 14 + 2 | Tool-free fixings for boards |
| NUT_M2.5 / NUT_M3 | 20 / 16 | For PCB supports and plates |
| Screws PH_M2×4, M2.5×8, M2.5×16, M3×8, T2×6, T2.6×12 | 8 / 16 / 4 / 44 / 4 / 16 | See the 1:1 scale page to identify them |

Page 30 prints every screw and pillar at **1:1 scale**, so you can lay a loose screw on the page to identify it.

---

## 4. Handling the key parts (section 2 of the guide)

### 4.1 Screws and the screwdriver
Use **only the screwdriver supplied in the kit**. The screws are small and the plastic is soft; the wrong driver strips the heads or the threads.

### 4.2 The rivet tool
Rivets are the push-fit plastic pins that hold boards and the adapter plate. To remove one, grab the pin with the rivet tool and pull **upwards**. Do not lever them with a screwdriver.

### 4.3 Cables
Two rules: look at the **shape of the connector and the port** before pushing (they only fit one way), and when disconnecting **pull the connector, never the cable**. Pulling the cable rips the wires out of the connector, and a broken DYNAMIXEL cable means one dead wheel.

### 4.4 The Li-Po battery and charger
This is the part most likely to hurt you or the robot, so the guide spends three pages on it.

1. When the robot **beeps a low-battery warning**, stop, take the battery out and charge it. Running a Li-Po flat damages it permanently.
2. To take it out: disconnect the connector (step 1 in the photo), pull out the bracket (step 2), slide the battery out (step 3).
3. The battery has **two connectors**: a big one *for the robot* and a small multi-pin one *for the charger*. You cannot plug the wrong one in by accident, but you should know which is which.
4. Connect the charger to the **SMPS** (the black mains power brick; SMPS stands for switched-mode power supply) and the battery's small connector to the charger. There is no on-switch: charging starts when you plug in.
5. Charger LED **red** means charging, **green** means finished.
6. Useful trick from the guide: if you plug the SMPS **directly into the robot's OpenCR jack**, the robot stays powered and you can remove the battery to charge it without rebooting the Pi.

**Everyday analogy:** treat the Li-Po like a phone battery with the safety case taken off. It is light and powerful because it has no protection, so *you* are the protection.

### 4.5 The LDS (laser distance sensor)
Five rules: remove the protective tape before first use; after that **do not touch the sensing window** (fingerprints scatter the laser); **do not touch the belt**; do not disassemble it; do not knock it. The belt rule matters because the belt sets the rotation speed, and the sensor's angle measurements assume a steady spin.

---

## 5. Safety precautions

The guide lists 21 precautions (page 3). Grouped, they say:

- **Tools and force:** only the kit's tools, no knives, drills or soldering; do not overtighten; do not modify or disassemble parts.
- **The battery:** keep it away from water, heat, metal objects (hairpins, clips), do not bend or twist it or its connector, and (yes, it really says this) do not put it in a microwave.
- **Operating:** keep a safe distance when the robot is active, mind your fingers near wheels and joints, do not run it on a **desk or table** because it will drive off the edge, and falls are not covered by warranty.
- **Wear:** the gears in the XL430 motors wear out; rough handling and crashes increase gear **backlash** (the slack between gear teeth, which shows up as a wobble in heading).

The Lab 2 handout adds its own list, which overlaps and is specific to your sessions:

1. Always run your algorithms on your **local machine**, connected over Wi-Fi (section 8 below).
2. Always put the robot **on the ground** for experiments.
3. **Disconnect the battery** when you finish.
4. **Never run on low battery**; the robot beeps a warning.
5. **Never charge the battery while it is connected to the robot.**
6. In an emergency press the **STOP slider** in the Simulink model.

**Common mistake:** leaving the battery connected overnight. Even an idle Pi draws current, and a Li-Po drained below its minimum voltage will not charge again.

---

## 6. How it moves: differential drive

The Burger has two independently driven wheels on one axle and a caster. That is exactly the **differential-steered robot** from Lecture 4 (slides 19–20), so the kinematics you already know apply directly.

Call the left and right wheel ground speeds $v_L$ and $v_R$, and the distance between the wheels $W$. Then the forward speed and the turn rate of the robot are

$$ v = \frac{v_R + v_L}{2}, \qquad \omega = \frac{v_R - v_L}{W}. $$

In words: the robot moves forward at the *average* of the two wheel speeds, and turns at a rate set by their *difference* divided by the wheel separation. Going the other way, if your controller produces a desired $(v, \omega)$, the robot's firmware computes

$$ v_R = v + \frac{\omega W}{2}, \qquad v_L = v - \frac{\omega W}{2}, $$

and converts each to a motor rotation speed by dividing by the wheel radius $r$.

**Worked example.** Suppose you command $v = 0.1$ m/s and $\omega = 0.5$ rad/s, and the wheel separation is $W = 0.16$ m. Then $v_R = 0.1 + 0.5 \cdot 0.08 = 0.14$ m/s and $v_L = 0.1 - 0.04 = 0.06$ m/s. The right wheel goes faster, so the robot curves to the left (anticlockwise), which is the positive direction for $\omega$. With a wheel radius of $0.033$ m, the right motor must spin at $0.14 / 0.033 \approx 4.2$ rad/s.

> **Background, ⚠️ not in the files:** the numbers above (wheel separation 0.160 m, wheel radius 0.033 m) are the Burger's published values from the ROBOTIS e-manual, as are its speed limits of about 0.22 m/s forward and 2.84 rad/s turning. The firmware clips anything larger, so a controller that asks for 1 m/s will simply saturate. Confirm these with your lab assistant before you rely on them in a report.

Why this matters for Lab 2: every controller in the lab (move to a point, move to a pose, follow a line, follow a wall) produces exactly two numbers, $v$ and $\omega$, and sends them in a ROS message. The robot does the wheel arithmetic for you. Your job is the control law from Lecture 4; the robot's job is this conversion.

---

## 7. From hardware to ROS data: the signal chain

Putting sections 2 and 6 together, here is how each piece of hardware becomes a ROS topic you can subscribe to or publish on. (ROS topics, publishers and subscribers are explained in the Lecture 5 notes.)

```
LDS-01 (spinning laser)  --serial-->  USB2LDS  --USB-->  Raspberry Pi  ==>  /scan   (LaserScan message: ranges)

DYNAMIXEL encoders (+ OpenCR gyro) --> OpenCR --USB--> Raspberry Pi  ==>  /odom   (Odometry message: position, orientation)

your controller ==> /cmd_vel (Twist message: linear.x, angular.z) ==> Raspberry Pi --USB--> OpenCR --> DYNAMIXEL wheel speeds
```

Three topics carry everything you need in Lab 2:

- **`/scan`** carries a `LaserScan` message. The important field is `ranges`, an array of distances, one per angle step around the full circle. A range of zero or infinity means "nothing seen" or "out of range" at that angle.
- **`/odom`** carries an `Odometry` message. `pose.pose.position` holds $x, y, z$ and `pose.pose.orientation` holds the heading as a **quaternion** $(q_x, q_y, q_z, q_w)$, not an angle. You convert it to yaw yourself (section 9).
- **`/cmd_vel`** takes a `Twist` message. You set `linear.x` to the forward speed $v$ in m/s and `angular.z` to the turn rate $\omega$ in rad/s. The other four fields are ignored by a ground robot.

**Common mistake:** confusing *odometry* with *ground truth*. `/odom` is the robot's own estimate from counting wheel turns, and it drifts (that is the whole point of Lecture 10). It is good enough for Lab 2 over a few metres, but if the wheels slip, `/odom` believes the robot moved when it did not.

---

## 8. Bringing the robot up in Lab 2

This is the exact procedure from section 3.1 of the lab handout. Each robot has a number (its ID) written on it, used in the IP address and the ROS Domain ID below.

1. **Wi-Fi.** Connect your lab PC to the network `turtlenet` (password `R7010E robotics`).
2. **Power.** Connect the battery, then press the Raspberry Pi's power button (the handout's Figure 2 shows where it is).
3. **SSH in.** Open a command window and run, replacing `X` with the robot ID (robot 6 is `192.168.50.60`):

   ```
   ssh turtle@192.168.50.X0
   ```

   The password for every robot is `turtle`. SSH (Secure Shell) gives you a terminal that is running *on the Pi*.
4. **Bring-up.** In that SSH shell type

   ```
   run
   ```

   This starts the robot's basic functions (motor driver, LiDAR driver, odometry) and must stay running in the background the whole time. If something gets stuck, for example the odometry needs resetting, press Ctrl+C and type `run` again.
5. **Teleoperation (optional, for testing).** Open a *second* SSH window and type `teleop`. Then drive with the keyboard:

   | Key | Action |
   |---|---|
   | `w` | forward |
   | `x` | backward |
   | `a` | turn left |
   | `d` | turn right |
   | `s` | stop |
   | Ctrl+C | terminate |

   **Close teleop before running your own controller.** Both publish on `/cmd_vel`, and two publishers fighting over the same topic makes the robot jerk unpredictably.
6. **MATLAB side.** Open the task's Simulink file (`Lab2_Task1.slx` and so on), open **ROS Network** settings and set the **ROS Domain ID to 3X** (robot 6 uses 36). The Domain ID is the ROS 2 way of keeping robots on the same Wi-Fi from hearing each other's messages; the other network settings are leftovers from ROS 1 and can be ignored.

**Everyday analogy:** the Domain ID is a radio channel. Every robot and every laptop in the room is on the same Wi-Fi, but only the ones tuned to channel 36 hear robot 6.

---

## 9. Talking to the robot from code

### 9.1 From MATLAB with ROS 2 (`TurtleBot3.m`, Lecture 5)

This script does by hand what the Simulink blocks do for you. Reading it line by line:

```matlab
setenv("ROS_DOMAIN_ID","39")            % tune to robot 9's channel
ros2 topic list                          % what topics are on the air?
testNode = ros2node("/test_node");       % create a node, i.e. "a program on the ROS network"
odomSub = ros2subscriber(testNode,"/odom");   % listen to odometry
scanSub = ros2subscriber(testNode,"/scan");   % listen to the LiDAR
odomData = receive(odomSub);             % block until one odometry message arrives
odomData.pose.pose.position              % x, y, z of the robot
velocityPublisher = ros2publisher(testNode,'/cmd_vel');   % we will send speed commands
velmsg = ros2message(velocityPublisher); % an empty Twist message
velmsg.linear.x = 0.1;                   % 0.1 m/s forward
send(velocityPublisher,velmsg);          % go
pause(1)
velmsg.linear.x = 0.0;  send(velocityPublisher,velmsg);   % stop after one second
scan = receive(scanSub);                 % one LiDAR sweep
points = rosReadCartesian(scan);         % ranges and angles -> x,y points in the robot frame
plot(points(:,1),points(:,2),'rO'); axis image
```

The rest of the script repeats the LiDAR plot inside a `while toc < 10` loop so you see the scan update live, and then does the same while driving forward for thirty seconds. Notice the pattern that every robot program follows: **subscribe to sensors, compute, publish a command, repeat**.

`rosReadCartesian` is doing a conversion you could do yourself: for each range $r_i$ at angle $\theta_i$, the point is $(x_i, y_i) = (r_i \cos\theta_i,\ r_i \sin\theta_i)$ in the robot's frame. That is polar-to-Cartesian from Lecture 1.

### 9.2 From Python with ROS 1 (`turtle_controller.py`, Lab 2 folder)

This template shows the same pattern in ROS 1 Python. Three things it does are worth understanding:

**It converts the odometry quaternion to a heading.** The function `quaternion_to_euler` computes yaw as

$$ \psi = \operatorname{atan2}\big(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2)\big). $$

For a robot on a flat floor, $q_x \approx q_y \approx 0$, so this collapses to $\psi = \operatorname{atan2}(2 q_w q_z,\ 1 - 2 q_z^2)$. In Lecture 1 you saw that a rotation by $\psi$ about $z$ is the unit quaternion $(\cos\frac{\psi}{2},\ 0,\ 0,\ \sin\frac{\psi}{2})$; the formula is simply undoing that. **Worked check:** a heading of 90° has $q_w = \cos 45° = 0.707$, $q_z = \sin 45° = 0.707$. Then $2 q_w q_z = 1.0$ and $1 - 2 q_z^2 = 0$, so $\operatorname{atan2}(1, 0) = \pi/2$. Correct.

**It uses callbacks.** `rospy.Subscriber('/odom', Odometry, callback_odom)` means "whenever a new odometry message arrives, run `callback_odom` with it". The callback copies $x$, $y$ and heading into global variables that the main loop reads. The main loop runs at `rospy.Rate(20)`, twenty times per second.

**It publishes a Twist.** Your controller fills `u_d` (forward speed) and `u_h` (turn rate); the template puts them into `control_signal.linear.x` and `control_signal.angular.z` and publishes. Those are exactly $v$ and $\omega$ from section 6.

⚠️ **Bug in the template:** it subscribes to `/scan` with the `LaserScan` message type but never imports it (`from sensor_msgs.msg import LaserScan` is missing), so the script will crash as written until you add that line. Also note the template is **ROS 1** (`rospy`, no Domain ID) while the lab's Simulink models and the MATLAB script are **ROS 2**. Check with your lab assistant which one you are expected to use.

---

## 10. The Lab 2 tasks, and the lecture idea behind each

| Task | What you build | Where the theory is |
|---|---|---|
| 1a Move to a point | $v = K_v \cdot$ distance, $\omega = K_h \cdot$ (heading error), heading error via atan2 | Lecture 4 §8 |
| 1b Move to a pose | polar coordinates $\rho, \alpha, \beta$ and the three-gain law | Lecture 4 §11 |
| 2 Follow a trajectory (square, your group number, a sinusoid) | the point controller chasing a moving target ("donkey and carrot") | Lecture 4 §10 |
| 3 Follow a line $y = ax + b$ | rewrite as $ax - y + b = 0$, then the two-term steering law with signed distance | Lecture 4 §9 |
| 4 LiDAR registration | transform each scan from the robot frame into the world frame using the odometry pose | Lectures 1 and 10 |
| 5 Keep a distance from a wall | pick the wall points from the scan, control the distance to them | Lecture 7 Part I (wall following) |

Two hints the handout gives for Task 4 are worth decoding because they are pure Lecture 1:

- **"Convert ranges to x, y points and associate them with the robot pose."** A scan point $(x_i, y_i)$ is in the robot frame $\{B\}$. To put it on the world map you apply the robot's pose: $ {}^W p_i = R(\theta)\, {}^B p_i + (x, y)^T $, where $(x, y, \theta)$ comes from `/odom`. That is ${}^W p = {}^W \xi_B \cdot {}^B p$ from Lecture 1, slide 16.
- **"Turning distorts the LiDAR points, why?"** One full sweep takes a fraction of a second. If the robot rotates during that time, the first and last rays of the sweep were measured from different headings, but they are stamped with one pose, so straight walls come out curved. The fix is to register scans only when the robot is (nearly) still, or to interpolate the pose for each ray using its timestamp.

For Task 3, note the sign trap: the lab defines the line as $y = ax + b$, but the Lecture 4 controller wants the form $Ax + By + C = 0$. Moving everything to one side gives $A = a$, $B = -1$, $C = b$. For line (a), $a = 2, b = 0$, that is $2x - y = 0$; for line (b), $0.5x - y + 4 = 0$. The distance to the line must be **signed** (positive on one side, negative on the other) or the controller cannot tell which way to turn.

---

## 11. Common mistakes

- **Running with teleop still open.** Two publishers on `/cmd_vel` make the robot stutter. Ctrl+C the teleop window first.
- **Wrong Domain ID.** If MATLAB sees no topics, the first thing to check is that the Domain ID is 3X for robot X, both in Simulink and (for scripts) in `setenv("ROS_DOMAIN_ID", ...)`.
- **Treating the orientation as an angle.** `/odom` gives a quaternion. Convert it (section 9.2) before you compute a heading error, and wrap the error into $[-\pi, \pi]$ with atan2 or `angdiff`.
- **Forgetting to stop.** Your script ends but the last `/cmd_vel` message said "0.1 m/s", so the robot keeps rolling off the table. Always publish a zero Twist at the end, and keep the Simulink STOP slider in reach.
- **Testing on a desk.** The guide and the handout both say floor only.
- **Pulling cables, not connectors.** A ripped DYNAMIXEL cable is the most common hardware failure.
- **Battery left connected.** Disconnect when you finish; never charge it while it is in the robot.

---

## In a nutshell

The TurtleBot3 Burger is a four-layer, two-wheeled differential-drive robot. Two DYNAMIXEL servo motors and a ball caster carry it, an 11.1 V Li-Po powers it, an OpenCR board drives the motors and talks to a Raspberry Pi that runs ROS, and a 360° laser distance sensor on top gives it a view of the room. Every controller you write boils down to publishing a forward speed and a turn rate on `/cmd_vel`, while you read the robot's estimated pose from `/odom` (a quaternion you convert to a heading) and its surroundings from `/scan`. In Lab 2 you bring it up over Wi-Fi and SSH with `run`, set the ROS Domain ID to 3X, and then implement the Lecture 4 control laws in Simulink or code. Handle the battery and the laser with care, keep the robot on the floor, and close teleop before your controller takes over.

## Key terms

- **TurtleBot3 Burger:** the small modular two-wheeled ROS robot used in Lab 2.
- **Waffle plate:** the perforated plastic structural plate; two per layer.
- **Plate support:** the metal pillar that separates layers (35 mm or 45 mm).
- **DYNAMIXEL XL430:** a smart servo motor with built-in encoder and controller; one per wheel.
- **Ball caster:** the passive rolling ball that forms the third contact point.
- **Li-Po battery:** the 11.1 V, 1800 mAh lithium-polymer pack; needs careful handling.
- **SMPS:** the mains power brick; charges the battery or powers the robot directly.
- **OpenCR 1.0:** the embedded controller board that drives the motors and powers the Pi.
- **Raspberry Pi 3:** the single-board computer running Linux and ROS.
- **USB2LDS:** the adapter that connects the laser sensor to the Pi over USB.
- **LDS-01:** the 360° laser distance sensor (a small LiDAR) on the top layer.
- **PCB support / rivet:** stand-offs and push-fit pins that mount the boards.
- **Differential drive:** steering by driving the two wheels at different speeds.
- **Odometry (`/odom`):** the robot's own pose estimate from wheel encoders (and gyro).
- **`/scan`:** the LiDAR topic; a `LaserScan` message with an array of ranges.
- **`/cmd_vel`:** the command topic; a `Twist` message with `linear.x` and `angular.z`.
- **SSH:** a secure remote terminal onto the Pi.
- **`run` / `teleop`:** the lab's bring-up command and keyboard-drive command.
- **ROS Domain ID:** the "channel" (3X for robot X) that isolates robots on shared Wi-Fi.
- **Quaternion to yaw:** the atan2 formula that turns the odometry orientation into a heading.

## Check yourself

1. Name the four layers of the Burger from bottom to top and the main component on each.
2. You command $v = 0.2$ m/s and $\omega = -1$ rad/s with wheel separation 0.16 m. What are the two wheel speeds, and which way does the robot turn?
3. The robot stutters when your controller runs. What is the most likely cause and the fix?
4. Why does `/odom` give you four numbers for orientation instead of one angle, and how do you get the heading?
5. In Task 4, why do straight walls look curved when you register scans while turning?

<details>
<summary>Answers</summary>

1. Layer 1: DYNAMIXEL motors, wheels, ball caster and the Li-Po battery. Layer 2: the OpenCR 1.0 controller board. Layer 3: the Raspberry Pi 3 with the USB2LDS adapter. Layer 4: the LDS-01 360° laser distance sensor.
2. $v_R = 0.2 + (-1)(0.08) = 0.12$ m/s and $v_L = 0.2 - (-1)(0.08) = 0.28$ m/s. The left wheel is faster, so the robot curves to the right (clockwise), which matches a negative $\omega$.
3. The `teleop` node is still running and also publishing on `/cmd_vel`, so two publishers fight. Press Ctrl+C in the teleop window before starting your controller.
4. ROS stores orientation as a unit quaternion $(q_x, q_y, q_z, q_w)$ because it has no gimbal lock and works in 3D. On a flat floor you recover yaw with $\psi = \operatorname{atan2}(2(q_w q_z + q_x q_y),\ 1 - 2(q_y^2 + q_z^2))$.
5. One sweep takes time, and the heading changes during it, but all the rays are stamped with a single pose. Rays measured early and late in the sweep are therefore rotated into the world frame with the wrong angle. Register only when stationary, or interpolate the pose per ray using timestamps.

</details>
