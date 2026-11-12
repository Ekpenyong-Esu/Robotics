# Lecture 5 – Programming in Robotics
### Study notes: ROS, MATLAB/Simulink, and LabVIEW

**Sources used for these notes:** `R7010E_Lecture_5_Programming_Robotics.pdf` (32 slides), the MATLAB script `TurtleBot3.m` in this folder, and (for the "how this connects to Lab 2" section only) the Lab 2 handout and `turtle_controller.py`.

---

## What this lecture is about, and why it matters

So far in the course you have been doing maths: poses, rotation matrices, trajectories, and the motion models of mobile robots. This lecture answers a very practical question: **once you have an algorithm, how do you actually get it running on a real robot?** It introduces the three software tools you will meet in the labs and in industry: ROS (the "glue" that connects all the programs running on a robot), MATLAB/Simulink (where you design, simulate and test your algorithms, and from which you can talk to ROS), and LabVIEW (a graphical programming tool popular for lab experiments and testing). There is not much maths here, but there are a lot of *concepts and vocabulary* that you will need in Lab 2 and Lab 3, and that come up constantly in any robotics job.

### Roadmap: the main topics in this lecture

1. Objectives of the lecture and the landscape of robot programming tools
2. What ROS is (and, just as importantly, what it is *not*)
3. The four "layers" of ROS: plumbing, tools, capabilities, ecosystem
4. How ROS works internally:
   - the ROS Master
   - Nodes
   - Topics
   - Messages
   - Publishers and Subscribers
   - Launch files
   - Services
   - Packages
   - Workspaces
5. MATLAB and Simulink for robotics
   - what you can do with them
   - the Festo case study and Model-Based Design
   - low-cost hardware support (Arduino, Raspberry Pi, LEGO EV3)
   - the Robotics System Toolbox
   - connecting MATLAB to ROS with Simulink blocks
   - connecting MATLAB to ROS with commands in an m-file (and the `TurtleBot3.m` script line by line)
6. LabVIEW: what it is, how it works, where it is used
7. How this lecture connects to Lab 2

---

## 1. Objectives and the tool landscape

The slides list three objectives for this lecture:

- Combine theory with practice.
- Develop the capacity to design, simulate and implement basic robotic applications.
- Identify software tools, combined with proper hardware platforms.

That third point is the heart of it. A robot is never just one program. There are sensor drivers, motor drivers, a controller, a planner, maybe a camera pipeline, and they all have to talk to each other, often across several computers. Picking the right tools for that job is a skill in itself.

Slide 3 shows the logos of the tools that appear in robotics: **LabVIEW, ROS, MATLAB, C++, Fortran and Python.** You should know that these are not competitors in a simple sense. ROS is a framework that your code plugs into; C++ and Python are the languages you most often write ROS programs in; MATLAB and LabVIEW are environments where you design and test algorithms (and both can connect to ROS). Fortran is there mostly as a reminder that lots of old, very fast numerical code still exists.

---

## 2. What ROS is (and what it is not)

**ROS** stands for **Robot Operating System**, and this is the first common trap: *despite its name, ROS is not an operating system.* It is a **framework**, which just means a collection of ready-made software pieces and rules for how your programs should fit together. You still run it on top of a normal operating system, usually Linux (Ubuntu).

Here is what the slides say ROS is, one point at a time, in plain words:

1. **It is a framework, not an operating system.** You install it onto Linux; it does not replace Linux.
2. **A collection of tools, libraries and conventions.** Tools (things you run, like visualizers), libraries (code you call from your own programs), and conventions (agreed ways of naming and structuring things so that code from different people fits together).
3. **A message passing system based on individual programs.** Instead of one giant program, a ROS robot is made of many small programs that send each other *messages* (little packets of data).
4. **Peer-to-peer connections between programs.** When two programs talk, they talk *directly* to each other, not through a central hub. (More on this when we meet the Master.)
5. **Works efficiently with libraries like Eigen, NLopt, OpenCV.** Eigen is a C++ library for linear algebra (matrices and vectors), NLopt is a library for numerical optimization, and OpenCV is the standard computer vision library. ROS does not try to replace these; it lets you use them inside your ROS programs.
6. **Programs can run on multiple robots that communicate over the network.** The pieces of a ROS system do not have to be on the same computer. They can be spread over a laptop, a robot's onboard computer, and a second robot, all connected by WiFi or Ethernet.
7. **ROS modules can be written in any language with a client library** (C++, Python, MATLAB, Java, etc.). A *client library* is the piece of code for a given language that knows how to speak "ROS". Because of this, one program can be in Python and another in C++ and they still talk happily.

> **Everyday analogy.** Think of ROS like the postal system plus a set of standard envelope sizes. The postal system does not care whether you write your letter in English or Swedish (any programming language), and it does not write the letters for you (it is not your algorithm). It just makes sure that letters addressed in the standard way get from one house (program) to another, even if the houses are in different cities (different computers).

> **Common mistake.** People hear "Robot Operating System" and assume you boot a robot into ROS the way you boot a laptop into Windows. You don't. You boot into Linux and then start ROS programs. On the exam, if asked "Is ROS an operating system?", the answer is **no, it is a framework (middleware) that runs on top of an OS.**

---

## 3. The four layers of ROS

Slide 5 has a famous picture: a ROS "ship" that equals **Plumbing + Tools + Capabilities + Ecosystem**. These four words are a nice way of remembering everything ROS gives you.

| Layer | What the slide lists | In plain words |
|---|---|---|
| **Plumbing** | Sensors, Actuators, Process management | The low-level wiring: drivers that read sensors and drive motors, plus the machinery for starting, stopping and connecting programs. |
| **Tools** | Visualization, Data logging, Simulation | Things that help you *see* and *debug*: 3D visualizers (RViz), recording and replaying data (rosbag), and simulators (Gazebo). |
| **Capabilities** | Control, Planning, Perception, Multiple agents | Ready-made algorithms you can reuse: controllers, path planners, vision/perception pipelines, and support for multi-robot systems. |
| **Ecosystem** | Package organization, Package distribution | The community side: a standard way to bundle code into *packages* and a way to share and install them. |

> **Analogy.** If you were building a house: plumbing is the pipes and wires in the walls, tools are your spirit level and multimeter, capabilities are pre-built kitchen units you can just install, and the ecosystem is the hardware store and the community of builders who share plans.

---

## 4. How ROS works internally

This is the most exam-relevant part of the lecture. The slides build up a picture one piece at a time. We will do the same, and I will keep the same order.

### 4.1 The ROS Master

The **ROS Master** is a special program that acts like a *phone book* for everything else. The slides give three facts:

1. It **manages the communication between nodes** (we will define nodes in a second; for now, think "programs").
2. **Every node registers at startup with the Master.** When a program starts, it tells the Master "Hi, I exist, here is my name, here is what I want to publish or listen to."
3. **No data (topics) flow through the core; all communications are direct.** This is the key point. The Master only does the *introductions*. Once two programs know about each other, they send the actual data *directly* to each other, peer-to-peer.

> **Analogy.** The Master is like a receptionist at a conference. You arrive, you sign in and say "I'm the camera person, I have images to share." Someone else signs in and says "I'm looking for images." The receptionist puts you in touch. After that, you two talk directly; the receptionist does not stand between you repeating every word.

> **Common mistake.** Students often think the Master is a hub that all messages pass through, like a router. It is not. If it were, it would be a huge bottleneck. Remember: *registration goes through the Master; data does not.*

> ⚠️ **ROS 1 vs ROS 2.** The slides describe **ROS 1**, which has a Master. The newer **ROS 2** (which the `TurtleBot3.m` script in this folder and the Lab 2 handout use) *removes the Master entirely*; nodes discover each other automatically on the network, and you separate different robots using a **ROS Domain ID** instead. The concepts of nodes, topics, messages, publishers, subscribers and services are the same in both. If you are asked about "the Master" on an exam, answer in ROS 1 terms as in the slides. Check with your lecturer which version they expect.

### 4.2 Nodes

A **node** is a single running program that does one specific job. The slides say nodes are:

- **Individual executable programs.** Each node is its own program you can start and stop.
- **Dedicated to a specific task.** One node reads the laser scanner. Another runs the controller. Another drives the motors.
- **Individually compiled, executed and organized.** You build them separately, run them separately, and can replace one without touching the others.

In the slide diagram, Node A and Node B each have a "Registration" line to the ROS Master. That is them signing in.

> **Why split things into nodes?** Imagine the camera driver crashes. If everything were one giant program, the whole robot would stop. With nodes, only the camera node dies; the motor controller keeps running and you can restart the camera node on its own. It also means you can swap a simulated sensor node for a real one without changing the rest of the system.

### 4.3 Topics

A **topic** is a named channel that data flows through. Think of it as a labelled pipe, like `/scan` or `/cmd_vel`. The slides say:

- A topic is the **communication bridge among individual nodes.**
- A topic **transfers data streams**, meaning a continuous flow of data, not a one-off.
- Topics are **asynchronous, many-to-many, one-way communication streams.**

Let me unpack that last line, because every word matters for exams:

- **Asynchronous**: the sender does not wait for anyone to receive. It just drops the data on the topic and carries on. (Like posting on a notice board rather than making a phone call.)
- **Many-to-many**: several nodes can send to the same topic, and several nodes can listen to it.
- **One-way**: data flows from sender to listener only. The listener does not reply on the same topic.

In the diagram, Node A has an arrow going into a box labelled "Topic", and an arrow comes out of it to Node B. The Master is still drawn above, but notice there is *no* arrow from the topic to the Master. That is slide-speak for "data does not go through the Master."

> **Analogy.** A topic is like a radio station. The station broadcasts (one-way). It doesn't wait for listeners (asynchronous). Any number of people can tune in, and in ROS, more than one station can even broadcast on the same frequency (many-to-many).

### 4.4 Messages

A **message** (slides write "msg") is the actual data packet that travels on a topic. The slides say a message is:

- A **data structure defining the type of a topic.** Every topic carries exactly one message type. The `/scan` topic carries laser-scan messages; `/cmd_vel` carries velocity-command messages.
- **Streamed through topics.**
- **Defined in `.msg` files.** A `.msg` file is a tiny text file listing the fields in the message.
- **Predefined or custom.** ROS ships with hundreds of standard messages (positions, images, laser scans), and you can write your own.

The slide shows two example message definitions:

```
# msg type 1
float64 x
float64 y
float64 z
```

```
# msg type 2
bool data
```

So message type 1 is a 3D point: three double-precision numbers. Message type 2 is a single true/false value. That is genuinely all a `.msg` file is: a list of `type name` lines.

> **Analogy.** If the topic is the pipe, the message is the shape of the thing travelling down the pipe. A pipe for tennis balls cannot carry footballs. Likewise, a topic declared with message type 1 can only carry three-number points.

> **Common mistake.** Mixing up *topic* and *message*. The topic is the **name of the channel** (`/odom`); the message is the **type and content of the data** on that channel (an `Odometry` message with position and orientation fields). You *publish a message on a topic*.

### 4.5 Publisher and Subscriber

Now we name the two roles a node can play on a topic:

- A **publisher** is a node that **sends out data** on a topic.
- A **subscriber** is a node that **receives data** from a topic.
- There can be **multiple subscribers** to one topic (and multiple publishers too).
- Both sides **use `.msg` files to specify the data packets**, so they agree on the format.

The slide diagram now labels Node A as "Publisher" with an arrow "Publish" into the Topic, and Node B as "Subscriber" with an arrow "Subscribe" coming out of the Topic. This is called the **publish/subscribe pattern** (often shortened to "pub/sub"), and it is the main way ROS nodes talk.

**A step-by-step example.** Suppose you want your controller to drive a TurtleBot:

1. The laser driver node *publishes* `LaserScan` messages on the topic `/scan`, maybe 10 times a second.
2. The odometry node *publishes* `Odometry` messages (where the robot thinks it is) on `/odom`.
3. Your controller node *subscribes* to `/scan` and `/odom`. Every time a new message arrives, it recomputes what to do.
4. Your controller node *publishes* a `Twist` message (desired forward speed and turning speed) on `/cmd_vel`.
5. The motor driver node *subscribes* to `/cmd_vel` and turns the wheels.

Notice that at no point does any node know *who* is on the other side. The controller doesn't care whether `/scan` comes from a real laser or a simulator. That is exactly why ROS makes it easy to move from simulation to a real robot.

### 4.6 Launch files

When you have five or ten nodes, starting them one by one in separate terminals is painful. A **launch file** solves that:

- It is a **tool for executing multiple nodes** at once.
- Defined in **`.launch` files** (XML text files in ROS 1; Python files in ROS 2).
- It **makes the project portable**: someone else can start your whole system with one command.
- It makes it **easy to change settings** (parameters, topic names) **when others use your package.**

> **Analogy.** A launch file is like a recipe card that says "turn on the oven, start the kettle, put the pan on the hob", all in one go, instead of you doing each step by hand every time.

### 4.7 Services

Topics are great for streaming data, but sometimes you want a *question and an answer*. That is what a **service** ("srv") is for. The slides say:

- **Synchronous, one-to-one, bidirectional communication.**
- **Node A sends a request to Node B and waits for a response.**
- Defined in **`.srv` files.**
- **Predefined or custom** types, just like messages.

In the diagram, Node A is the **Service Client** and Node B is the **Service Server**. An arrow "Request" goes A to B; an arrow "Response" comes back B to A. The Master is not even drawn here, because this is the point: it's a direct, two-way conversation.

Unpacking the words again:

- **Synchronous**: the client *waits* (blocks) until the answer comes back.
- **One-to-one**: one client talks to one server for that call.
- **Bidirectional**: data goes both ways (request out, response back).

**Example.** A "reset odometry" service. Your controller node (client) sends a request "please reset to zero". The odometry node (server) does it and replies "done". You wouldn't use a topic for this, because you want to *know* it happened before you continue.

#### Topics vs services: the comparison you must know

| | **Topic** | **Service** |
|---|---|---|
| Direction | One-way | Two-way (request + response) |
| Timing | Asynchronous (fire and forget) | Synchronous (client waits) |
| Participants | Many-to-many | One-to-one |
| Defined in | `.msg` file | `.srv` file |
| Good for | Continuous data streams: sensor readings, velocity commands | Occasional questions or commands that need a confirmation: "reset", "compute this for me" |

> **Common mistake.** Using a service for sensor data. If the laser scanner had to wait for each consumer to acknowledge every scan, it would never keep up. Streams go on topics; one-off requests go through services.

### 4.8 Packages

A **package** is how ROS code is bundled. The slides say:

- Packages are **separate modules that provide different task services.**
- Packages are the **main unit for organizing software in ROS.** A package may contain **nodes** (runtime processes), a **ROS-dependent library**, **datasets**, **configuration files**, or anything else that is usefully organized together.

So a package is a folder with a standard structure. For example, a `turtlebot3_bringup` package might contain the nodes that start the robot, the launch files to run them, and config files with the robot's dimensions.

> **Analogy.** A package is like an app on your phone. It bundles the program, its icons and its settings into one thing you can install, share, and delete as a unit.

### 4.9 Workspaces

A **workspace** is the folder on your computer where your packages live and get built. Slides:

- **Contains all packages** (nodes, msgs, launch files, srvs).
- **Multiple task-oriented workspaces can be defined.** You might keep one workspace for your course labs and another for a research project.
- **Create workspace chains by extending workspaces.**

The slide diagram shows a chain: **ROS Indigo → extend → `catkin_ws` → extend → `devel_ws`**. "ROS Indigo" is the base ROS installation (Indigo was a 2014 release name). `catkin_ws` is a workspace built on top of it (catkin is the ROS 1 build tool). `devel_ws` is a further workspace built on top of that. Each workspace in the chain can see everything in the ones below it. This is also called **overlaying**: you *overlay* your workspace on top of the installed ROS so you can use both your own packages and the standard ones.

> **Analogy.** Think of transparent sheets stacked on a projector. The bottom sheet is the base ROS install. Each workspace you add is another sheet laid on top; you see the combined picture, and the top sheets can override what is underneath.

> ⚠️ The slide names "ROS Indigo" and `catkin_ws`, which are ROS 1 terms. In ROS 2 the build tool is `colcon` and the base install is "sourced" rather than "extended", but the idea of stacking workspaces is identical.

### 4.10 The ROS demo

Slide 15 just says **"DEMO"**. According to its position, the lecturer demonstrated a live ROS system at this point (most likely starting nodes, listing topics and watching messages).

> ⚠️ The demo content itself is not in the slides, so there is nothing to revise from it directly. Everything you would need to know is covered by the concepts above.

---

## 5. MATLAB and Simulink for robotics

Now we switch tools. **MATLAB** is a numerical computing environment (you have used it already with Peter Corke's Robotics Toolbox). **Simulink** is MATLAB's graphical, block-diagram environment for modelling and simulating systems that change over time. The lecture's point is that MATLAB is not just for maths on your laptop; it can talk to real robots.

### 5.1 What MATLAB and Simulink let you do

Slide 16 lists five things. With MATLAB and Simulink you can:

1. **Connect to and control your robot with the algorithms you develop.** Write the controller in MATLAB, run it against the real robot.
2. **Develop hardware-agnostic algorithms and connect to ROS.** *Hardware-agnostic* means the algorithm does not care which robot it runs on. You write it once, and ROS is the bridge to whichever robot you plug in.
3. **Connect to a range of sensors and actuators** to send control signals or analyse many types of data.
4. **Eliminate hand-coding by automatically generating code for embedded targets.** You draw a Simulink model, press a button, and MATLAB writes the C/C++ code and puts it on the hardware. An *embedded target* is a small computer inside a device (like the one in a drone's flight controller).
5. **Connect to low-cost hardware such as Arduino and Raspberry Pi.**

### 5.2 The three big uses: simulate, teach/learn, use ROS

Slides 17, 19 and 23 build up a diagram step by step with the MATLAB logo in the middle and three arrows out of it:

- **Simulate Robots** (slide 17)
- **Teach/Learn robotics** (slide 19)
- **Use ROS** (slide 23)

That is a tidy summary to remember: *MATLAB in robotics = simulate, teach, and talk to ROS.*

### 5.3 "Building robots" and the Festo case study

Slide 18 shows the classic feedback loop: **Input → (+/−) → Controller → Plant → Output**, with the output fed back to the summing point. Here *plant* is the control-engineering word for the thing being controlled (the robot or motor), and *controller* is the algorithm deciding what to do. What's new is that the "Controller" box has a photo of a **Raspberry Pi** inside it. The message: your Simulink controller block can literally *become* the code running on that board.

The slide then cites **Festo**, a company that developed an innovative robotic arm (the picture shows a soft, elephant-trunk-like gripper handing over an apple) using **Model-Based Design**. *Model-Based Design* means you build a simulation model of the system first, design and test the controller against the model, and then auto-generate the real code from that same model. The benefit is that you test far more in simulation and write far less code by hand.

### 5.4 Low-cost hardware support

Slide 20 is a collage about building robots with low-cost hardware:

- **No need for C/C++/Python**, and **drivers are provided**, through what MathWorks calls a **Hardware Support Package**.
- Free support packages exist for **LEGO MINDSTORMS EV3**, **Arduino**, and **Raspberry Pi**, each from both MATLAB (acquire data, control motors) and Simulink (run models directly on the board).
- There are **free lab workbooks** at `www.mathworks.com/hardware`.

Slide 21 zooms in on the **Raspberry Pi model**. It shows the Simulink Raspberry Pi block library (blocks for UDP send/receive, audio capture and playback, video capture, video display, GPIO read/write, an LED, text-to-speech) and an example model called **"Image Inversion"**: a *V4L2 Video Capture* block feeds an *Inversion Algorithm* block, which feeds an *SDL Video Display* block. You click **"Deploy to Hardware"** and the model runs on the Pi, showing an inverted camera image on screen. This is the code-generation promise from slide 16 made concrete.

### 5.5 Object detection demo and applications

Slide 22 says there was a **demo on object detection**, with three applications listed:

- **Visual servoing**: using camera images directly inside a control loop to steer the robot (you will meet this properly in the Vision lecture).
- **Exploration**: using vision to decide where to go in an unknown environment.
- **Pedestrian recognition**: detecting people, important for safety in self-driving and service robots.

> ⚠️ The demo itself is not in the slides; only this list of applications is.

### 5.6 The Robotics System Toolbox

Slide 24 is a screenshot of the MathWorks page for the **Robotics System Toolbox** (shown as of release R2018b). Reading the screenshot, the toolbox:

- Provides **algorithms and hardware connectivity** for autonomous robots: aerial and ground vehicles, manipulators, humanoids.
- Includes algorithms for **path planning and path following for differential-drive robots, scan matching, obstacle avoidance, and state estimation**; for manipulators, **inverse kinematics, kinematic constraints, and dynamics** using a rigid-body-tree representation.
- Provides an **interface between MATLAB/Simulink and ROS**, so you can test on ROS-enabled robots and simulators such as **Gazebo**.
- Supports **C++ code generation**: you can generate a **ROS node from a Simulink model** and deploy it to a ROS network. **Simulink external mode** lets you view signals and tune parameters while the deployed model runs.

The four capability tiles at the bottom are: **Coordinate System Transformations**, **Robot Operating System (ROS)**, **Ground Vehicle Algorithms** (mapping, localization, SLAM, path planning and following, state estimation), and **Manipulator Algorithms**.

> ⚠️ Note that this is distinct from Peter Corke's *Robotics Toolbox* that you used in Lectures 1 to 4. Same word "toolbox", different product. Corke's is a free academic toolbox for learning; MathWorks' Robotics System Toolbox is the commercial one with the ROS bridge. In recent MATLAB versions the ROS part has been moved into a separate **ROS Toolbox**.

**Slide 25** is the architecture diagram you should be able to sketch. On the left, inside MATLAB: **Built-in algorithms → MATLAB code**, and **Simulink models**. In the centre: a **ROS** box, connected to MATLAB code and Simulink models by two-way arrows labelled **"Networking (messages, services, etc.)"**. On the right: a **Robot**, a **Simulator**, and a **ROS node**, all connected to ROS. A dashed arrow goes directly from **Simulink models** to **ROS node**, labelled **"C++ code generation"**. In words: MATLAB can *talk to* ROS live over the network, or it can *generate* a standalone ROS node from a Simulink model.

**Slide 26** shows a realistic deployment. A **Main CPU** runs **MATLAB** and **Simulink** (both marked as ROS NODEs). Over **Ethernet** it talks to a **Robot (CPU 2)** that runs nodes for **Global Planner, Map server, Image processing, Local Planner, Kinematics & Control, Localization & Mapping**. Separate **Sensors** and **Actuators** nodes connect to the robot CPU, and other devices (a TurtleBot, a Raspberry Pi board) appear as further nodes. Every box is a node, and every envelope icon is a message. This is the "programs on multiple robots communicating over the network" claim from slide 4, drawn out.

### 5.7 Connecting MATLAB to ROS with Simulink

Slide 27 says there was a **demo: "Connect MATLAB to ROS with Simulink"** and shows the two essential Simulink blocks:

- **ROS Subscribe** block: you set the topic (here `/location`). It has two outputs: **IsNew** (a flag that is true when a fresh message has arrived) and **Msg** (the message itself).
- **ROS Publish** block: you set the topic (again `/location`). It has one input, **Msg**, the message you want to send.

So in Simulink, "talking to ROS" is literally: drop a Subscribe block to get sensor data in, wire your controller blocks, and drop a Publish block to send commands out. That is exactly what the Lab 2 Simulink templates do.

> ⚠️ The slide heading reads "Demo (more blocks from the demo?)", which looks like a note the lecturer left to themselves. Only these two blocks are shown. The demo itself is not in the materials.

### 5.8 Connecting MATLAB to ROS with commands (m-file)

Slide 28 lists the MATLAB **commands** for ROS, grouped by purpose. These are **ROS 1** commands:

| Command(s) | Purpose (from the slide) | Plain words |
|---|---|---|
| `rosinit( )` / `rosshutdown( )` | Start or shut down the global MATLAB node; connect to an existing ROS master | Open or close your connection to the ROS network. |
| `rosnode( )`, `rostopic( )`, `rosmsg( )` | "What's there? What can we communicate about?" | List the nodes, list the topics, list the message types. |
| `rossubscriber( )` then `receive( )` | Listening | Create a subscriber on a topic, then wait for a message. |
| `rospublisher( )` then `rosmessage( )` and `send( )` | Broadcasting | Create a publisher, build an empty message of the right type, fill it in, send it. |
| `robotics.ros.Node( )` | Connect to multiple ROS masters | Advanced: a MATLAB node that can talk to more than one ROS network at once. |

Slide 29 then says **"Demo: Connect MATLAB to ROS with m-file"**. ⚠️ The demo recording isn't in the slides, **but the script `TurtleBot3.m` in this folder is almost certainly what was shown (or its updated version).** It uses **ROS 2** commands, which have the same shape with a `ros2` prefix. Let me walk you through it line by line.

#### `TurtleBot3.m` explained

```matlab
clear all
clc
```
Clear all variables from memory and clear the command window. Standard housekeeping.

```matlab
%% set ROS Domain ID
setenv("ROS_DOMAIN_ID","39")
```
This is the ROS 2 replacement for "connect to the Master". ROS 2 has no Master; instead, all nodes that share a **domain ID** can see each other on the network. Setting it to 39 means "join the network of TurtleBot number 9" (the Lab 2 handout says to use `3X` where X is the robot number). If you get this wrong you will see no topics and nothing will work.

```matlab
%% check ros topics
ros2 topic list
```
Ask the network "what topics exist?" This is the ROS 2 version of `rostopic`. You should see things like `/odom`, `/scan`, `/cmd_vel`.

```matlab
%% Creat Node
testNode = ros2node("/test_node");
ros2 node list
```
Create a MATLAB node called `/test_node`. Now MATLAB is itself a ROS node, just like in the slide-26 diagram. `ros2 node list` shows it alongside the robot's own nodes.

```matlab
%% Subscribe to odmo
odomSub = ros2subscriber(testNode,"/odom")
scanSub = ros2subscriber(testNode,"/scan")
```
Create two subscribers, both belonging to our node: one listening to `/odom` (where the robot thinks it is) and one to `/scan` (the LiDAR readings). This is the "Listening" row of the slide-28 table.

```matlab
%% Read odom
odomData = receive(odomSub);
odomData.pose.pose.position
```
`receive` waits for the next message on `/odom` and hands it over. The message is a structure; `.pose.pose.position` digs down to the `x`, `y`, `z` fields. The commented-out `orientation` line would give you the quaternion (which you learned about in Lecture 1).

```matlab
%% Creat ros2 velocity publisher
velocityPublisher = ros2publisher(testNode, '/cmd_vel')
```
Create a publisher on `/cmd_vel`, the topic the motor driver listens to. This is the "Broadcasting" row.

```matlab
%% Publish velocity for 1 sec
velmsg = ros2message(velocityPublisher);
velmsg.linear.x = 0.1; % meters per second
send(velocityPublisher,velmsg);
pause(1)
velmsg.linear.x = 0.0;
send(velocityPublisher,velmsg);
```
This is the full publish pattern in four steps:
1. `ros2message` creates an empty message of the right type for this publisher (a `Twist`, which has `linear` and `angular` parts).
2. Fill in `linear.x = 0.1`, meaning 0.1 m/s forward.
3. `send` publishes it. The robot starts moving.
4. Wait one second, set the speed to zero, send again. The robot stops.

> **Common mistake.** Forgetting step 4. The robot keeps obeying the last command it received. If you send 0.1 m/s and your script ends, the robot keeps rolling until it hits something. Always send a zero at the end.

```matlab
%% plot LIDAR information for 10 seconds
tic
figure(1)
while toc<10
    scan = receive(scanSub);
    points = rosReadCartesian(scan);
    plot(points(:,1),points(:,2),'rO')
    axis image
end
```
For ten seconds (`tic`/`toc` is MATLAB's stopwatch), repeatedly: receive a laser scan, convert it from ranges-and-angles (polar) to x-y points with `rosReadCartesian`, and plot them as red circles. `axis image` keeps the aspect ratio correct so a square room looks square. You are watching the robot's view of the world update live.

```matlab
%% Moving forward and plot LIDAR for 30 second
velmsg.linear.x = 0.1;
send(velocityPublisher,velmsg)
tic
figure(1)
while toc<30
    scan = receive(scanSub);
    points = rosReadCartesian(scan);
    plot(points(:,1),points(:,2),'rO')
    axis image
end
velmsg.linear.x = 0.0;
send(velocityPublisher,velmsg)
```
Same idea, but now the robot drives forward while you plot, and after 30 seconds it is stopped. You should see the wall points slide towards the robot in the plot as it approaches them.

#### ROS 1 vs ROS 2 command cheat sheet

| ROS 1 (slide 28) | ROS 2 (`TurtleBot3.m`) |
|---|---|
| `rosinit` (connect to Master) | `setenv("ROS_DOMAIN_ID", ...)` (no Master; pick a domain) |
| `rostopic list` | `ros2 topic list` |
| `rosnode list` | `ros2 node list` |
| `rossubscriber(topic)` | `ros2subscriber(node, topic)` |
| `rospublisher(topic)` | `ros2publisher(node, topic)` |
| `rosmessage(pub)` | `ros2message(pub)` |
| `receive`, `send` | `receive`, `send` (same) |

> ⚠️ Because the slides show ROS 1 commands and the script uses ROS 2, be ready for either on an exam. The *concepts* (node, topic, subscriber, publisher, message) are identical; only the spelling changes.

---

## 6. LabVIEW

**LabVIEW** stands for **Laboratory Virtual Instrument Engineering Workbench**, made by National Instruments. It is the third tool in the lecture and the one you are least likely to have seen.

### 6.1 What it is

From slide 30:

- A **graphical-based programming language.** You do not type code; you wire blocks together on a canvas.
- The basic element is the **VI (Virtual Instrument).** A VI is one LabVIEW program. The name comes from the idea that your program behaves like a lab instrument (an oscilloscope, a signal generator) but made of software.
- Claimed to be the **No. 1 software in the world for experimentation and testing (rapid prototyping).** It is very widely used in test labs and industrial measurement.

### 6.2 Advantages (slide 30)

- **Extensive interface**: lots of ready-made knobs, dials, graphs and indicators for building user interfaces quickly.
- **Code reuse and platform independent**: VIs can be reused as sub-blocks inside other VIs, and run on different operating systems and NI hardware.
- **Parallel processing**: because of the dataflow model (next section), independent parts of a program naturally run in parallel.
- **Advanced hardware**: tight integration with National Instruments data-acquisition cards, FPGAs and real-time controllers.

### 6.3 Applications (slide 31)

- Machine monitoring and control
- Research and analysis
- Control design

### 6.4 How a VI is built: Front Panel and Block Diagram

Every VI has two windows, and the slide shows both:

- The **Front Panel** is the *user interface*: the knobs, switches, graphs and numeric displays the user sees and interacts with. It is what the instrument "looks like."
- The **Block Diagram** is the *code*: the wiring of functions and structures that does the work. Each control or indicator on the front panel appears as a terminal on the block diagram.

> **Analogy.** A real oscilloscope has a face with knobs and a screen (front panel) and circuit boards inside (block diagram). LabVIEW lets you build both halves in software.

### 6.5 How LabVIEW executes: three key facts (slide 31)

1. **Data Flow programming.** In a normal language, lines run top to bottom. In LabVIEW, a block runs *as soon as all of its inputs have data*. Execution is driven by data arriving on wires, not by line order. This is why parallelism comes for free: two blocks with no wire between them can run at the same time.
2. **It is not an interpreted language (it has an execution engine).** Your diagram is compiled to machine code, so it runs fast; it is not read and executed line by line at run time like a simple script.
3. **Execution is based on the Arbitrary Interleaving principle.** When several blocks are all ready to run and there is no wire forcing an order, LabVIEW may run them in *any* order (interleaved however it likes). You cannot rely on left-to-right position to control order; if order matters, you must wire it explicitly (for example with an error wire or a sequence structure).

> **Analogy for dataflow.** Think of a kitchen with several cooks. A cook starts a dish as soon as all the ingredients for it are on the counter. Two cooks with separate ingredients work simultaneously. If dish B needs the sauce from dish A, it waits; that dependency is the "wire." But if two dishes are independent, you can't predict which finishes first; that's arbitrary interleaving.

> **Common mistake.** Assuming that a block drawn to the left executes first. It does not, unless a wire connects them. This catches almost everyone new to LabVIEW.

### 6.6 LabVIEW demos (slide 32)

The slide lists two demos: **"How to add two numbers in LabVIEW"** (the classic first program: two numeric controls wired into an Add block wired to an indicator) and **"A Vortex Actuator robot in LabVIEW."**

> ⚠️ Neither demo is in the slides, and no details of the Vortex Actuator robot are given. If LabVIEW is examined, expect the conceptual points in 6.1 to 6.5 rather than demo details.

---

## 7. How this lecture connects to Lab 2

Lab 2 (Mobile Robot Motion and Control) is where you use all of this for real. Reading the lab handout alongside this lecture:

- You **SSH into the TurtleBot's Raspberry Pi** and run a bring-up command. That starts the robot's own ROS nodes (motor driver, LiDAR driver, odometry). This is the "Robot (CPU 2)" box of slide 26.
- You run a **teleop node** to drive with the keyboard; the handout warns to stop it before running your own controller, because two publishers on `/cmd_vel` fight each other. That is the "many-to-many" property of topics biting you.
- In Simulink, you set the **ROS Domain ID to 3X** (X = robot number). That is the `setenv("ROS_DOMAIN_ID", ...)` line of `TurtleBot3.m`, and the ROS 2 replacement for "connecting to the Master."
- The Simulink task templates use the **ROS Subscribe** block (on `/odom` and `/scan`) and **ROS Publish** block (on `/cmd_vel`) from slide 27, with your controller from Lecture 4 (move to point, move to pose, follow line, follow trajectory) in between.
- The Python file `turtle_controller.py` in the Lab 2 folder is the *same architecture in ROS 1 Python*: `rospy.init_node` (create a node), `rospy.Subscriber('/odom', Odometry, callback)`, `rospy.Subscriber('/scan', LaserScan, callback)`, `rospy.Publisher('/cmd_vel', Twist)`, then a loop at 20 Hz that fills a `Twist` with `linear.x` (forward speed) and `angular.z` (turn rate) and publishes it. Spotting that this is exactly the slide-10 publisher/subscriber picture is the whole point of the lecture.
- Task 4 of the lab (plotting LiDAR points and building a point-cloud map) is a direct extension of the last two cells of `TurtleBot3.m`.

---

## In a nutshell

ROS is not an operating system but a framework: a set of tools, libraries and conventions that lets many small programs (nodes) talk to each other by passing messages, directly and peer-to-peer, even across several computers and in different programming languages. The ROS Master (in ROS 1) is only a registry where nodes sign in; actual data flows over topics, which are asynchronous, one-way, many-to-many streams carrying messages of a fixed type defined in `.msg` files, while services provide synchronous, one-to-one request/response calls defined in `.srv` files. Code is organised into packages, packages live in workspaces that can be chained, and launch files start many nodes at once. MATLAB and Simulink let you simulate robots, learn robotics, and connect to ROS either live (subscribe/publish blocks or `rossubscriber`/`rospublisher` style commands) or by generating C++ ROS nodes from Simulink models, with free support for low-cost boards like Arduino and Raspberry Pi. LabVIEW is a graphical dataflow language whose programs (VIs) have a front panel for the user interface and a block diagram for the logic, run via a compiled execution engine, and execute independent blocks in arbitrary interleaved order.

---

## Key terms

- **ROS (Robot Operating System)**: a framework (not an OS) of tools, libraries and conventions for building robot software out of communicating programs.
- **Framework / middleware**: software that sits between the operating system and your application and handles communication and organisation for you.
- **Peer-to-peer**: programs send data directly to each other, with no central hub in the data path.
- **Plumbing, Tools, Capabilities, Ecosystem**: the four "layers" of what ROS provides (wiring and drivers; visualisation/logging/simulation; reusable algorithms; package organisation and sharing).
- **ROS Master**: the ROS 1 registry that nodes sign in with at startup; it introduces nodes to each other but no data flows through it.
- **Node**: a single running program dedicated to one task.
- **Topic**: a named, one-way, asynchronous, many-to-many data stream between nodes.
- **Message (.msg)**: the data structure (list of typed fields) that travels on a topic; every topic has one message type.
- **Publisher**: a node that sends messages on a topic.
- **Subscriber**: a node that receives messages from a topic.
- **Launch file (.launch)**: a file that starts several nodes with their settings in one command.
- **Service (.srv)**: a synchronous, one-to-one, two-way request/response call between a service client and a service server.
- **Package**: the basic unit for organising ROS software; a folder bundling nodes, libraries, config, data.
- **Workspace**: the folder where packages are built; workspaces can be chained (overlaid) on top of each other.
- **ROS Domain ID**: in ROS 2, the number that groups nodes into one network (replaces connecting to a Master).
- **Simulink**: MATLAB's graphical block-diagram environment for modelling and simulating dynamic systems.
- **Hardware-agnostic**: an algorithm written so it does not depend on which specific robot or board it runs on.
- **Model-Based Design**: design and test a controller against a simulation model, then auto-generate the real code from that same model (the Festo example).
- **Hardware Support Package**: a MathWorks add-on providing drivers and blocks for boards like Arduino, Raspberry Pi, LEGO EV3.
- **Robotics System Toolbox**: the MathWorks toolbox with robot algorithms and the MATLAB/Simulink to ROS interface, including C++ ROS-node generation.
- **ROS Subscribe / ROS Publish blocks**: the two Simulink blocks that receive from and send to a ROS topic.
- **`rosinit`, `rossubscriber`, `receive`, `rospublisher`, `rosmessage`, `send`**: the ROS 1 MATLAB commands to connect, listen and broadcast (ROS 2 versions: `ros2node`, `ros2subscriber`, `ros2publisher`, `ros2message`).
- **LabVIEW (Laboratory Virtual Instrument Engineering Workbench)**: National Instruments' graphical dataflow programming environment.
- **VI (Virtual Instrument)**: a LabVIEW program.
- **Front Panel**: the user-interface window of a VI.
- **Block Diagram**: the code/wiring window of a VI.
- **Dataflow programming**: execution model where a block runs as soon as all its inputs have data, rather than in line order.
- **Execution engine**: LabVIEW compiles diagrams rather than interpreting them, so they run fast.
- **Arbitrary interleaving**: when several blocks are ready and not wired in sequence, LabVIEW may run them in any order.

---

## Check yourself

1. A classmate says: "ROS is the operating system the TurtleBot boots into, and all sensor data passes through the ROS Master so it can route it to the right place." Find the two mistakes and correct them.

2. You need (a) a continuous stream of LiDAR readings to your controller and (b) a way to ask the odometry node to reset to zero and confirm that it did. Which ROS communication mechanism fits each, and name three properties of each that justify your choice.

3. In the ROS message/topic picture, what is the difference between a *topic*, a *message*, and a *publisher*? Give a one-line example of each using `/cmd_vel`.

4. Put these `TurtleBot3.m` steps in the right order and say what each does: `send`, `ros2message`, `ros2publisher`, `setenv("ROS_DOMAIN_ID", ...)`, `ros2node`. Why must you send a second message with zero velocity at the end?

5. In LabVIEW, you draw block A to the left of block B with no wire between them. Will A run before B? Explain using the two execution principles from the lecture, and say what the Front Panel and Block Diagram each contain.

<details>
<summary>Answers</summary>

**1.** Mistake one: ROS is *not* an operating system; it is a framework (middleware) that runs on top of a normal OS such as Linux. Mistake two: in ROS 1 the Master only handles *registration* (nodes sign in and get introduced to each other); *no data flows through it*. All actual communication is direct, peer-to-peer between nodes. (Bonus: ROS 2, which the lab uses, has no Master at all and uses a Domain ID.)

**2.** (a) LiDAR stream: a **topic**. Topics are asynchronous (the sensor doesn't wait for anyone), one-way (sensor to listeners), and many-to-many (several nodes can read the same scans). Defined by a `.msg` type. (b) Reset-and-confirm: a **service**. Services are synchronous (the client waits for the answer), one-to-one (one client, one server), and bidirectional (request goes out, response comes back). Defined by a `.srv` type.

**3.** The **topic** is the named channel: `/cmd_vel`. The **message** is the typed data packet that travels on it: a `Twist` message with `linear.x` (forward speed) and `angular.z` (turn rate). The **publisher** is the node that sends messages onto the topic: your controller node (in MATLAB, created with `ros2publisher(node, '/cmd_vel')`). The motor driver is the subscriber on that topic.

**4.** Order: (1) `setenv("ROS_DOMAIN_ID","39")` joins the robot's ROS 2 network (the ROS 2 replacement for connecting to a Master); (2) `ros2node("/test_node")` makes MATLAB a node; (3) `ros2publisher(node,'/cmd_vel')` creates the publisher; (4) `ros2message(pub)` creates an empty `Twist` of the right type, which you then fill (e.g. `linear.x = 0.1`); (5) `send(pub, msg)` publishes it. You must send a zero-velocity message at the end because the robot keeps obeying the *last* command it received; if the script ends after a 0.1 m/s command, the robot keeps driving.

**5.** No, not necessarily. LabVIEW uses **dataflow programming**: a block runs when all its inputs have data, not based on position on the screen. With no wire between A and B, both are "ready," and under the **arbitrary interleaving** principle LabVIEW may run them in either order or in parallel. To force A before B you must create a data dependency (a wire) or use a sequence structure. The **Front Panel** holds the user interface (controls like knobs and switches, indicators like graphs and numeric displays); the **Block Diagram** holds the code (the functions and wires that process the data, with a terminal for each front-panel control or indicator).

</details>
