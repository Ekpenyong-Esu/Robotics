# R7010E Robotics – Study Notes Index

These are friendly, read-on-your-own study notes built from every lecture file in this workspace (slides, MATLAB scripts, live scripts and handouts). Each lecture folder now contains a `..._Study_Notes.md` file. Open the notes in VS Code's Markdown preview so the formulas render.

## Order of the lectures (and why)

The folder names carry the lecture numbers, and the first slide deck lists the official course structure, so the order below follows that. Two things are worth knowing:

- **There is no separate "Lecture 2" file.** The course plan says Lecture 1 is "Intro and Representing Position and Orientation" and Lecture 2 is its continuation. The single Lecture 1 deck (35 slides) and the MATLAB file named `Matlab_Time_Lecture_1_2_live_2025.mlx` cover both, so the Lecture 1 notes cover Lectures 1 and 2 together.
- **There is no "Lecture 9" file.** The Lecture 7 deck (89 slides) explicitly spans two sessions: reactive navigation (this lecture) and map-based navigation (next lecture). The workspace has two folders, "Lecture 7 - Navigation" and "Lecture 7 - Navigation II", with identical files. The notes are therefore split in two: Part I in the first folder, Part II in the second.
- The Lecture 3 deck's own title page says "Lecture 2 – Time and Motion", but the folder and the course plan call it Lecture 3. The notes follow the folder.

| # | Topic | Notes file |
|---|-------|-----------|
| 1 (and 2) | Position and Orientation | [Lecture 1 notes](Lecture1/Lecture_1_Position_and_Orientation_Study_Notes.md) |
| 3 | Time and Motion | [Lecture 3 notes](Lecture3/Lecture_3_Time_and_Motion_Study_Notes.md) |
| 4 | Intro to Mobile Robotics | [Lecture 4 notes](Lecuture4/Lecture_4_Mobile_Robotics_Study_Notes.md) |
| 5 | Programming in Robotics (ROS, MATLAB, LabVIEW) | [Lecture 5 notes](Lecture%205%20-%20Programming%20in%20Robotics/Lecture_5_Programming_in_Robotics_Study_Notes.md) |
| 6 | Aerial Robotics | [Lecture 6 notes](Lecture%206%20-%20Aerial%20Robotics/Lecture_6_Aerial_Robotics_Study_Notes.md) |
| 7 (Part I) | Navigation: reactive methods, potential fields, Bug algorithms | [Lecture 7 Part I notes](Lecture%207%20-%20Navigation/Lecture_7_Navigation_Part_I_Reactive_Study_Notes.md) |
| 7 (Part II) | Navigation: map-based planning, A*, D*, PRM, RRT | [Lecture 7 Part II notes](Lecture%207%20-%20Navigation%20II/Lecture_7_Navigation_Part_II_Map_Based_Study_Notes.md) |
| 8 | Perception and Sensors | [Lecture 8 notes](Lecture%208%20-%20Perception/Lecture_8_Perception_Study_Notes.md) |
| 10 | Localization, Mapping and SLAM | [Lecture 10 notes](Lecture%2010%20-%20Localization/Lecture_10_Localization_Study_Notes.md) |
| 11 | Vision for Robotics | [Lecture 11 notes](Lecture%2011%20-%20Vision%20for%20Robotics/Lecture_11_Vision_for_Robotics_Study_Notes.md) |

## Lab notes

Each lab folder now has its own study notes too. Labs 1 and 2 are built from their handouts. Labs 3 and 4 have **no handout in the workspace**, so their notes are reconstructed from the Simulink models, code templates and the course plan, and say so at the top. Check them against the official task list on Canvas.

| Lab | Builds on | Notes file |
|---|---|---|
| Lab 1 Time and Motion (trajectories, PUMA 560, MAV paths) | Lectures 1 and 3 | [Lab 1 notes](Lab%201%20-%20Time%20and%20Motion/Lab_1_Time_and_Motion_Study_Notes.md) |
| Lab 2 Mobile Robot Motion and Control (TurtleBot3, Simulink, ROS 2) | Lectures 4 and 5 | [Lab 2 notes](Lab%202%20-%20Mobile%20Robotics/Lab_2_Mobile_Robotics_Study_Notes.md) |
| Lab 3 Experimentation with aerial robots (Crazyflie, Vicon) | Lecture 6 | [Lab 3 notes](Lab%203%20-%20Aerial%20Robotics/Lab_3_Aerial_Robotics_Study_Notes.md) |
| Lab 4 Navigation / 2D SLAM (TurtleBot3 with LiDAR) | Lectures 7 and 10 | [Lab 4 notes](Lab%204%20-%20Navigation/Lab_4_Navigation_Study_Notes.md) |

The `Report Template.docx` in every lab folder is the same file: Objective, Methodology, Results, about one and a half pages plus figures, submitted with code and video as one zip.

## Reference notes: the toolbox and the lab robot

Two reference documents in the workspace also have their own study notes. They are not lectures, so read them alongside the lecture that uses them rather than in sequence.

| Reference | Notes file | Read it with |
|---|---|---|
| Robotics Toolbox for MATLAB (Peter Corke, Release 10) | [Toolbox notes](Robotics_Tool_Box/Robotics_Toolbox_Study_Notes.md) | Lectures 1, 3, 4, 7 and 10, which all run toolbox commands |
| TurtleBot3 Burger quick-start guide and Lab 2 bring-up | [TurtleBot notes](TurtleBot/TurtleBot3_Burger_Study_Notes.md) | Lectures 4 and 5, and before Lab 2 |

The TurtleBot quick-start PDF in the `TurtleBot/` folder and the `TURTLEBOT3_MANUAL.pdf` in the Lab 2 folder are the same file.

## How to use these notes

1. Read the lectures in the order above. Later lectures assume earlier ones (poses before motion, motion before mobile robots, mobile robots before navigation, perception before localization).
2. Every tricky idea comes with an analogy and a worked example. Try covering the worked example and redoing it yourself.
3. Watch for the ⚠️ markers. They mark places where the slides were unclear, incomplete, or possibly wrong. Ask your lecturer about those.
4. Finish each lecture with the "Check yourself" questions before you look at the hidden answers.

## Things worth asking your lecturer about

Every notes file flags its own issues with ⚠️ in context. These are the ones that look like genuine slide errors rather than things that were simply left unexplained, so they are the ones most worth raising:

- **Lecture 1, slide 30:** the boxed ZYX roll-pitch-yaw formula ends in a rotation about z; it should end in a rotation about x. Slide 29 also says ZYZ is "commonly used in aeronautics and ground robotics", which conflicts with slide 30 and the toolbox default (vehicles use ZYX).
- **Lecture 3, lecture script:** the tumbling-body loop integrates orientation with angular acceleration times dt. The textbook uses angular velocity times dt, which is the physically correct update.
- **Lecture 3, slide 18:** the boundary-condition labels for t = T are printed as t = 0, and one acceleration term reads 2d instead of 2D.
- **Lecture 4, slide 3:** the quadrotor is marked holonomic despite having 6 degrees of freedom and 4 actuators. Slide 6 labels the rolling constraints "holonomic" when they are non-holonomic, and gives the singularity as heading θ = π/2 rather than steering angle γ = ±π/2. Slide 19's boxed differential-drive equations use the wheel-speed difference where the sum is needed (slide 20 has it right).
- **Lecture 5:** the slides describe ROS 1 (Master, rosinit, catkin), while the lecture script and Lab 2 use ROS 2 (no Master, Domain ID, ros2 commands). Check which the course expects you to know.
- **Lecture 6, slides 10 and 14:** two different z-axis conventions (NED z-down vs ENU z-up) appear in one deck. Slide 15 says "yaw neglected" but the crossed-out terms correspond to zero roll. Slide 23 appears to wire the pitch-rate error to the roll torque and vice versa.
- **Lecture 7, slides 14 to 16:** the repulsive potential is printed without its square, the attractive angle uses the inverse-tangent arguments the wrong way round, and the repulsive distance formula reuses the goal coordinates instead of the obstacle's. In the A* grid example, slide 58 skips a cost update that the algorithm's own flowchart requires.
- **Lecture 8:** slides 1 to 10 are photographs only. Everything about how each sensor works is added background in the notes, clearly marked, so confirm with the lecturer what depth is expected.
- **Lecture 10, slide 19:** the mapping assumption "sensor to detect features (x, y, θ)" is ambiguous. The sensor returns range and bearing; (x, y, θ) is the known vehicle pose.
