# The Robotics Toolbox for MATLAB – Study Notes (companion to the lectures)

> Built from `Robotics_Toolbox_Manual.pdf` (Peter Corke, Robotics Toolbox for MATLAB, Release 10, June 2017, 437 pages). This is the toolbox that every "MATLAB time" slide in the course uses, and it matches the second edition of the textbook *Robotics, Vision and Control* (RVC2). These notes are not a rewrite of the whole manual. They walk through the functions and classes the lectures and labs actually call, in the order you meet them in the course, and they teach you how to read the manual yourself for anything else.

## What this is about, and why it matters

The Robotics Toolbox (often shortened to RTB) is a free collection of MATLAB functions and classes for representing poses, generating trajectories, simulating mobile robots, planning paths, and estimating position with Kalman filters. Every lecture in this course has a "MATLAB time" where the lecturer types toolbox commands, and the labs expect you to use them. If you understand what each command takes in and gives back, the lecture scripts stop being magic and become a way to check your hand calculations.

## Roadmap: the main topics

1. Getting the toolbox running and how to read its manual (help, doc, page layout, the category index, `rtbdemo`).
2. Two ways of working: classic matrices versus the new pose classes (`SO2`, `SE2`, `SO3`, `SE3`, `UnitQuaternion`).
3. Pose in 2D and 3D: rotation matrices, homogeneous transforms, Euler and roll‑pitch‑yaw angles, quaternions, skew‑symmetric matrices, plotting frames (used in Lecture 1).
4. Time and motion: velocity transforms, the adjoint, trajectories with `tpoly`, `lspb`, `mtraj`, `mstraj`, `ctraj`, orientation interpolation (used in Lecture 3).
5. Mobile robots: `Vehicle`, `Bicycle`, `Unicycle`, `RandomPath`, the Simulink models (used in Lecture 4).
6. Navigation: `Navigation` superclass, `Bug2`, `DXform`, `Dstar`, `PRM`, `Lattice`, `RRT`, occupancy grids (used in Lecture 7).
7. Localization and SLAM: `LandmarkMap`, `RangeBearingSensor`, `EKF` in its four modes, `ParticleFilter`, `PoseGraph` (used in Lecture 10).
8. A short overview of the manipulator (robot‑arm) side of the toolbox.
9. Small utilities and graphics you will see in the scripts.
10. Common mistakes, a cheat sheet, and the usual end sections.

---

## 1. Getting started and reading the manual

### 1.1 What the toolbox is

The preface describes RTB as "over twenty five years of continuous development". It covers two families of robots: arm‑type manipulators (kinematics, trajectories, dynamics, control) and mobile robots (path planning, kinodynamic planning, localization, map building and SLAM). It also provides the plumbing for representing position and orientation: vectors, rotation matrices, quaternions, homogeneous transforms and twists. The code is deliberately written to be readable rather than fast.

**Everyday analogy:** think of the toolbox as a well‑organised toolbox in a workshop. The manual is the label on each drawer. You do not read it cover to cover; you learn where the drawers are and what each tool is for.

### 1.2 Installing it (Section 1.2 of the manual)

There are two ways:

1. **`.mltbx` file.** Since MATLAB R2014b you can download `robot.mltbx`, double‑click it in MATLAB, and it installs itself and fixes the path for every future session. This is the easy route.
2. **`.zip` file.** Unzip it, then every time MATLAB starts run

```matlab
addpath RVCDIR        % the folder that contains rvctools
startup_rvc           % adds subfolders to the path and prints the version
```

After installation the folder `rvctools` has subfolders `robot` (this toolbox), `vision` (the Machine Vision Toolbox), `common` (shared utilities), `simulink` (block diagrams and examples) and `contrib`.

**Common mistake:** forgetting to run `startup_rvc` after a zip install. The symptom is "Undefined function 'rot2'". Either run it each time, put it in your `startup.m`, or use `pathtool` and save the path.

The manual was tested with R2016b and R2017a. ⚠️ The course scripts in this workspace use much newer MATLAB features (for example the `ros2node` commands in the Lecture 5 script), so you may be running a newer MATLAB than the manual assumes. The toolbox functions themselves still work, but if you hit an odd error, check Peter Corke's site for the latest release.

### 1.3 How the manual is organised

- **Preface** (page 3).
- **Functions by category** (pages 5–7): a list grouped by topic, such as "Homogeneous transformations 3D", "Trajectory generation", "Mobile robot", "Localization", "Path planning", "Graphics", "Utility". This is the page to look at when you know *what* you want to do but not the function name.
- **Contents** (pages 9–13): every function alphabetically with its page.
- **Chapter 1, Introduction**: what changed in Release 10, how to install, the Simulink models, documentation, use in teaching and research.
- **Chapter 2, Functions and classes**: the bulk of the book, one entry per function or class in alphabetical order.

### 1.4 How to read one entry

Every function entry has the same shape, because it is generated automatically from the comments in the MATLAB source code:

1. The **name** and a one‑line description. For `rot2` this is "SO(2) Rotation matrix".
2. One or more **call signatures** in bold, each followed by a sentence saying what comes back, with matrix sizes in brackets. For example "R = rot2(theta) is an SO(2) rotation matrix (2 × 2) representing a rotation of theta radians." Read the sizes carefully; they tell you whether you need a row or a column vector.
3. **Options**: name–value pairs written like `'deg'` or `'covar', C`. You pass them as extra arguments.
4. **Notes**: the gotchas. Always read these.
5. **References** and **See also**: related functions, which is often the fastest way to find the thing you actually need.

A **class** entry (for example `Bicycle` or `EKF`) first lists all its **Methods** and **Properties** in a table, then an **Example**, and then each method appears as its own entry named `Class.method`, such as `Bicycle.step`.

**Everyday analogy:** the entry is like a recipe card: the title tells you the dish, the bold lines are the ingredients and what you get, the options are the optional spices, and the notes are the "don't open the oven door" warnings.

### 1.5 Getting help inside MATLAB

```matlab
help rot2          % prints the same text as the manual entry
doc rot2           % opens it in the help browser (under "Supplemental Software")
rtbdemo            % menu of demonstration scripts
rtbdemo(2)         % same, but auto-advances every 2 seconds
```

`rtbdemo` covers rotation matrices and transforms, quaternions, trajectories, arm kinematics and dynamics, mobile robot kinematics and control, path planning (D*, PRM, Lattice, RRT), localization (EKF, particle filter), SLAM (EKF and pose graph) and quadrotor control. Some demos need Simulink.

### 1.6 What changed in Release 10 (and why it matters for the lecture scripts)

Section 1.1 lists changes that explain several oddities you will meet:

| Release 9 name | Release 10 name | Why you care |
|---|---|---|
| `Vehicle` (was the bicycle model) | `Bicycle`; `Vehicle` is now an abstract parent of `Bicycle` and `Unicycle` | The lecture scripts write `Bicycle('covar', V)`. If you see `Vehicle(V)` in an old example it will not run. |
| `PointMap` | `LandmarkMap` | Same story. |
| `path` | `query` | Planners now use `plan` then `query`. |
| `jacobn` | `jacobe` | Arm Jacobian in the end‑effector frame. |
| `H_x, H_xf, H_w, G_x, G_z` | `Hx, Hp, Hw, Gx, Gz` | Sensor Jacobians for the EKF. |
| functions `se2`, `se3` (returned matrices) | classes `SE2`, `SE3` (return objects) | Lecture 3 uses `SE3(...)` objects. |

⚠️ The manual itself is not fully updated: the `EKF` and `ParticleFilter` examples still write `Vehicle(V)` and `PointMap(20)`. Treat those two examples as Release 9 code. The versions in your lecture script (`Bicycle('covar', V)`, `LandmarkMap(20)`) are the correct Release 10 forms.

New in Release 10 that the course uses: the 2D helper functions ending in `2` (`rot2`, `trot2`, `transl2`, `trplot2`), `trexp`/`trlog`, the `Twist` class, the `Lattice` planner, the `PoseGraph` class, and the `tripleangle` app for playing with Euler angles. The toolbox no longer depends on the Machine Vision Toolbox, except for one thing we will meet in Section 7.

---

## 2. Two ways of working: matrices or pose objects

RTB has always represented rotations and poses as plain MATLAB matrices: a 2×2 or 3×3 rotation matrix, a 3×3 or 4×4 homogeneous transform, and a trajectory as a 3‑dimensional stack such as 4×4×N. Release 10 added **classes** that wrap those matrices: `SO2`, `SE2`, `SO3`, `SE3` and `UnitQuaternion`, all children of `RTBPose`. They behave like matrices (you can multiply them with `*`) but carry extra methods, and a trajectory becomes a plain vector of objects, which is easier to read.

The manual's own example (page 16) shows both styles:

```matlab
T = transl(1,2,3);     % classic: a 4x4 matrix
trprint(T)             % function call
T = SE3(1,2,3);        % new: an SE3 object
trprint(T)             % the same name now works as a method
T.T                    % the 4x4 matrix inside the object
double(T)              % same thing
T2 = T*T;              % still an SE3 object
T3 = trinterp(T, 5);   % a vector of five SE3 objects
T3(1)                  % the first one
T3*T                   % multiplies every element: five SE3 objects
```

Lecture 1 mostly uses the classic functions (`rot2`, `transl2`, `trot2`, `rotx`, `eul2r`, `rpy2r`) plus `UnitQuaternion`. Lecture 3 switches to objects (`SE3`, `SO3`, `SO2`). Both are fine; just know which one you are holding.

**Common mistake:** mixing an `SE3` object with a raw 4×4 matrix in arithmetic, or passing an object to a classic function that expects a matrix. If a function complains, convert with `T.T` or `double(T)`, or wrap a matrix with `SE3(T)`.

**Everyday analogy:** the matrix is the raw number in a cell; the object is the same number in a spreadsheet cell that also knows it is a currency and can format itself, convert itself, and refuse nonsense operations.

---

## 3. Pose in 2D and 3D (Lecture 1: `Lecture_1_Position_and_Orientation_Study_Notes.md`)

### 3.1 2D rotation: `rot2`, `trot2`, `transl2`

**`R = rot2(theta)`** returns the 2×2 rotation matrix for `theta` radians. **`rot2(theta, 'deg')`** takes degrees.

Worked example: `rot2(0.3)`. Since cos 0.3 = 0.9553 and sin 0.3 = 0.2955,

```
R = [ 0.9553  -0.2955
      0.2955   0.9553 ]
```

Check it is a rotation: `det(R)` is 1 and `R*R'` is the identity.

**`T = trot2(theta)`** is the 3×3 homogeneous transform with that rotation and zero translation. **`T = transl2(x, y)`** is the 3×3 transform with zero rotation and translation `(x, y)`.

⚠️ The manual itself flags an "historical anomaly": `transl2` (and `transl`) work in both directions. `transl2(1, 2)` *builds* a transform; `transl2(T)` *extracts* the translation from one as a 2‑element column vector. The same function, two jobs. Read the shape of what you pass in.

Worked example from the Lecture 1 script:

```matlab
T1 = transl2(1, 2) * trot2(30, 'deg')
```

gives

```
T1 = [ 0.8660  -0.5000   1.0000
       0.5000   0.8660   2.0000
       0        0        1.0000 ]
```

Read it as "move to (1, 2), then turn 30°". The top‑left 2×2 block is the rotation; the right column is the translation.

Now compose with `T2 = transl2(2, 1)`:

- `T3 = T1*T2`: the new translation is `t1 + R1*t2 = (1,2) + (0.866·2 − 0.5·1, 0.5·2 + 0.866·1) = (2.232, 3.866)`. The displacement of (2, 1) was applied *in the rotated frame {1}*, which is why the result is not (3, 3).
- `T4 = T2*T1`: translation is simply `(1+2, 2+1) = (3, 3)` because `T2` has no rotation. Same rotation of 30°.

That is the non‑commutativity of pose composition that Lecture 1 stresses, demonstrated with numbers.

**Common mistake:** degrees versus radians. Every rotation function assumes **radians** unless you add `'deg'`. `rot2(30)` is a rotation of 30 radians, which is about 4.8 full turns, not 30 degrees.

### 3.2 Transforming points: `e2h`, `h2e`, `homtrans`

A point in the toolbox is a **column** vector, and several points are stored side by side as columns of a 2×N or 3×N matrix. To multiply a point by a homogeneous transform you need to add the "1" at the bottom, which is called converting from **Euclidean** to **homogeneous** form.

- **`H = e2h(E)`**: appends a row of ones. A 2×N matrix becomes 3×N.
- **`E = h2e(H)`**: removes it (and divides by the last element).
- **`p2 = homtrans(T, p)`**: does both for you. If `T` is 3×3 and `p` is 2×N it treats the points as Euclidean 2D points; if `p` is 3×N it treats them as homogeneous. Same logic in 3D with 4×4 and 3×N / 4×N.

Worked example (Lecture 1 script): where is the world point P = (3, 2) as seen from frame {1} above?

```matlab
P1 = inv(T1) * [3; 2; 1]       % manual homogeneous form
h2e( inv(T1) * e2h([3; 2]) )   % toolbox form
```

By hand: `inv(T1)` has rotation `R'` and translation `−R'*t`. So the point in {1} is `R'*(P − t) = R'*(2, 0) = (0.866·2, −0.5·2) = (1.732, −1.000)`. The negative y tells you the point is to the right of frame {1}'s x‑axis, which matches the picture in the lecture.

**Common mistake:** passing a **row** vector `[3 2]` where a column is expected. The toolbox convention is: points and vectors are columns; rows are reserved for time steps in trajectories and for the state `[x y theta]` of a vehicle. If you get a dimension error, transpose.

### 3.3 Plotting frames: `plotvol`, `trplot2`, `trplot`, `tranimate`, `plot_point`

- **`plotvol([xmin xmax ymin ymax])`** creates axes with those limits, equal aspect ratio, grid on, and `hold on`, so later plots stack. `plotvol(w)` is shorthand for ±w. Add two more numbers for 3D.
- **`trplot2(T, options)`** draws a 2D frame for the 3×3 transform. Useful options: `'frame', 'A'` labels it {A} and subscripts the axes; `'color', 'b'`; `'framelabel', 'A'` names it without subscripts. Returns a handle so you can later move it with `trplot2(H, Tnew)`.
- **`trplot(T, options)`** is the 3D version for 4×4 transforms or 3×3 rotation matrices, with many more options (`'rgb'` colours x red, y green, z blue; `'arrow'`; `'length', s`; `'view', 'auto'`; `'rviz'` for chunky axes).
- **`tranimate(T)`** animates a frame moving from identity to `T`; `tranimate(T1, T2)` moves between two poses; pass a 4×4×N stack to animate a trajectory. Options `'fps'`, `'nsteps'`, `'axis'`, `'movie'`.
- **`plot_point(p, options)`** adds markers at the columns of `p`. `'label', 'P'` writes a label; `'solid'`, `'ko'` style the marker.

The Lecture 1 script's rotation‑about‑a‑point example uses exactly these:

```matlab
plotvol([-5 4 -1 4.5]);
T0 = eye(3,3);             trplot2(T0, 'frame', '0');
X  = transl2(2, 3);        trplot2(X,  'frame', 'X');
R  = trot2(2);             % 2 rad, about 115 degrees
trplot2(R*X, 'framelabel', 'RX', 'color', 'r');   % rotates about the world origin
trplot2(X*R, 'framelabel', 'XR', 'color', 'r');   % rotates about X's own origin
C  = [1 2]';
RC = transl2(C) * R * transl2(-C);                % rotate about point C
trplot2(RC*X, 'framelabel', 'XC', 'color', 'r');
```

The recipe "translate C to the origin, rotate, translate back" is the standard way to rotate about any point, and the three frames on the plot show you the difference between pre‑ and post‑multiplying.

### 3.4 3D rotation: `rotx`, `roty`, `rotz`, `trotx`, `troty`, `trotz`, `transl`

- **`rotx(theta)`**, **`roty(theta)`**, **`rotz(theta)`** return 3×3 rotation matrices about one axis (radians, or add `'deg'`).
- **`trotx`**, **`troty`**, **`trotz`** return the 4×4 versions with zero translation.
- **`transl(x, y, z)`** is a 4×4 pure translation; `transl(T)` extracts the translation (same anomaly as `transl2`). `transl(p)` with `p` an M×3 matrix returns a 4×4×M stack, one transform per row.

Worked example: `rotx(pi/2)`.

```
[ 1   0   0
  0   0  -1
  0   1   0 ]
```

Read the columns: the new x‑axis is still (1,0,0); the new y‑axis points along old z (0,0,1); the new z‑axis points along old −y (0,−1,0). That is a 90° turn about x.

⚠️ The comment in `Matlab_Time_Lecture_1.m` says `rotx(pi/2)` is "45 degrees". It is 90 degrees. Trust the number, not the comment.

Non‑commutativity in 3D (script): `rotx(pi/2)*roty(pi/2)` and `roty(pi/2)*rotx(pi/2)` are different matrices. The lecture plots both with `trplot` and animates with `tranimate`.

Splitting and joining: **`R = t2r(T)`** pulls the rotation block out of a transform; **`T = r2t(R)`** pads a rotation into a transform with zero translation; **`[R, t] = tr2rt(T)`** and **`T = rt2tr(R, t)`** do both parts at once. They work for 2D and 3D by looking at the size.

Worked example from the script:

```matlab
T = transl(1, 0, 0) * trotx(pi/2) * transl(0, 1, 0)
```

Think of it as three moves applied in the moving frame: step 1 along x, turn 90° about x, step 1 along the *new* y, which is the old z. So the final translation is (1, 0, 1), and `transl(T)` returns `[1; 0; 1]`. `t2r(T)` returns the `rotx(pi/2)` matrix above.

### 3.5 Three‑angle representations: `eul2r`, `tr2eul`, `rpy2r`, `tr2rpy`, `tripleangle`

**Euler angles (ZYZ).** `R = eul2r(phi, theta, psi)` builds `rotz(phi)*roty(theta)*rotz(psi)`. You can also pass a 1×3 vector `eul2r([phi theta psi])`. `eul2tr` is the 4×4 version. `'deg'` option as usual. **`eul = tr2eul(R)`** goes back.

Two gotchas the manual states in the Notes, and the Lecture 1 script demonstrates:

1. **The inverse is not unique.** `R = eul2r(0.1, -0.2, 0.3)` then `tr2eul(R)` does not return `(0.1, −0.2, 0.3)`. It returns `(−3.0416, 0.2, −2.8416)`, that is `(0.1 − π, +0.2, 0.3 − π)`. Both angle sets give the same matrix; the toolbox always chooses θ ≥ 0. Use the `'flip'` option if you want the other branch.
2. **θ = 0 is a singularity.** `R = eul2r(0.1, 0, 0.3)` then `tr2eul(R)` returns `(0, 0, 0.4)`. When θ is zero the two z‑rotations merge, only their sum 0.1 + 0.3 = 0.4 can be recovered, and the toolbox sets φ to zero by convention. This is the gimbal lock of Lecture 1 showing up in code.

**Roll‑pitch‑yaw (ZYX by default).** `R = rpy2r(roll, pitch, yaw)` builds `rotz(yaw)*roty(pitch)*rotx(roll)`; `rpy2tr` is the 4×4 version; **`rpy = tr2rpy(R)`** inverts it. Options: `'xyz'` for the gripper convention (z‑axis forward), `'yxz'` for a camera, `'deg'`.

⚠️ The manual's sentence "These correspond to rotations about the Z, Y, X axes respectively" is easy to misread as "roll is about Z". It means the *sequence* is Z then Y then X, i.e. yaw about Z, pitch about Y, roll about X, with the roll applied first (rightmost). The Notes add that "Toolbox rel 8‑9 has the reverse angle sequence as default", so very old code may behave differently.

The singularity for roll‑pitch‑yaw is at pitch = ±π/2 (the manual: "R is arbitrarily set to zero and Y is the sum (R+Y)"). The lecture script notes this is "fortunately outside the range of feasible attitudes for most vehicles".

**`tripleangle`** opens an interactive app with three sliders and shows the frame after one, two and three rotations (red, green, blue). `tripleangle('euler')` switches to ZYZ. Angles are shown in degrees. Use it to *see* gimbal lock: set the middle angle to 90° and watch two sliders do the same thing.

### 3.6 Angle‑axis form: `angvec2r`, `tr2angvec`, `trexp`, `trlog`, `skew`, `vex`

A rotation can also be written as "angle θ about unit axis v".

- **`R = angvec2r(theta, v)`** builds it. **`[theta, v] = tr2angvec(R)`** inverts it (θ always in [0, π]; negative rotations flip the axis).
- **`S = skew(v)`** builds the skew‑symmetric matrix of a vector. For a scalar `skew(0.3)` gives `[0 -0.3; 0.3 0]`; for `v = [1 2 3]` it gives

```
[  0  -3   2
   3   0  -1
  -2   1   0 ]
```

A skew‑symmetric matrix is one whose transpose equals its negative. **`v = vex(S)`** is the inverse (it averages the two copies of each element). `skewa`/`vexa` are the "augmented" versions for 3‑ and 6‑element twists, adding a translation column.

- **`trexp`** is the matrix exponential for so(3)/se(3) and **`trlog`** the matrix logarithm. `trexp(skew(v))` equals `angvec2r(norm(v), v/norm(v))`. The Lecture 1 script shows the 2D version with MATLAB's own `logm` and `expm`:

```matlab
R = rot2(0.3);
S = logm(R)        % [0 -0.3; 0.3 0]  -- the angle reappears!
vex(S)             % 0.3
expm(S)            % back to R
R = expm(skew(0.3))% same as rot2(0.3)
```

**Everyday analogy:** `skew` and `vex` are a zip and unzip for the one (or three) numbers hidden inside a rotation‑rate matrix; `trexp`/`trlog` are the bridge between "how fast and about which axis" and "the resulting rotation matrix".

**`trnorm(R)`** re‑orthogonalises a rotation matrix that has drifted after many multiplications. Use it occasionally in long loops.

### 3.7 Quaternions: `Quaternion` and `UnitQuaternion`

The manual defines a quaternion as a scalar `s` plus a vector `v`, written `s <vx, vy, vz>`. A **unit quaternion** has `s² + vx² + vy² + vz² = 1` and represents a rotation of θ about unit axis v as `q = cos(θ/2) < v sin(θ/2) >`. In Release 10 these are two classes: `Quaternion` (general) and `UnitQuaternion` (rotations), and display differs: double brackets `<< >>` for a general quaternion, single `< >` for a unit one.

**Constructors** you will use:

```matlab
q = UnitQuaternion()              % identity, 1 <0,0,0>
q = UnitQuaternion(R)             % from a 3x3 rotation matrix
q = UnitQuaternion(T)             % from the rotation part of a 4x4 transform
q = UnitQuaternion(rpy2tr(0.1, 0.2, 0.3))   % the Lecture 1 script
q = UnitQuaternion.Rz(pi/2)       % rotation about z; also .Rx, .Ry, with 'deg'
q = UnitQuaternion.rpy(r, p, y)   % from roll-pitch-yaw
q = UnitQuaternion.eul(a, b, c)   % from ZYZ Euler angles
q = UnitQuaternion.angvec(th, v)  % angle and axis
q = UnitQuaternion.omega(w)       % rotation of |w| about the vector w
```

**Operations** (overloaded operators, from the manual's table):

| Expression | Meaning |
|---|---|
| `q1*q2` | Hamilton product, i.e. compose rotations (`q2` first, then `q1`, same as matrices) |
| `q*v` | rotate the 3×1 vector `v` (this is `q v q⁻¹` done for you) |
| `inv(q)` or `q.inv` | inverse; for a unit quaternion this is the conjugate |
| `q1/q2` | `q1 * inv(q2)` |
| `q.R`, `q.T` | convert to 3×3 rotation matrix or 4×4 transform |
| `q.torpy`, `q.toeul`, `q.toangvec` | convert to angles |
| `q.plot`, `q.animate` | draw / animate the frame |
| `q.interp(q2, s)` | spherical linear interpolation, see Section 4.5 |
| `q.norm`, `q.unit`, `q.conj` | length, normalise, conjugate |

Worked example: `q = UnitQuaternion.Rz(pi/2)`. Here θ = π/2, so `s = cos(π/4) = 0.7071` and `v = (0, 0, sin(π/4)) = (0, 0, 0.7071)`. Displayed as `0.70711 < 0, 0, 0.70711 >`. Now `q*[1 0 0]'` rotates the x‑axis unit vector by 90° about z and returns `[0; 1; 0]`, as it should. And `q*q` is a 180° rotation: `s = cos(π/2) = 0`, `v = (0, 0, 1)`, displayed `0 < 0, 0, 1 >`.

From the Lecture 1 script: `q*inv(q)` gives the identity `1 < 0, 0, 0 >`, and `q/q` is the same thing (a null rotation).

**Common mistakes.**
- `q1 + q2` and `q1 − q2` return a plain `Quaternion`, not a `UnitQuaternion`, because the element‑wise sum is not a valid rotation. Rotations are *multiplied*, never added.
- Only the `R` and `T` constructor forms are vectorised; `UnitQuaternion(s, v)` builds one at a time.
- `q.*q2` is the Hamilton product followed by re‑normalisation; use it in long loops to stop drift (the Lecture 3 tumbling‑body loop does exactly this).

### 3.8 The pose classes: `SO2`, `SE2`, `SO3`, `SE3`, `RTBPose`

`RTBPose` is the abstract parent that gives all four classes the same behaviour: `plot`, `animate`, `print`, `display`, `double` (to matrix), and operators `*` (compose, or apply to a vector), `/` (multiply by inverse), `==`. The manual notes `SO3` is "polymorphic with UnitQuaternion", so you can swap one for the other with minimal code change.

**`SE2`** holds a 3×3 transform. Constructors: `SE2()` null, `SE2(x, y)` translation, `SE2(x, y, theta)` translation plus rotation, `SE2(xyt)` from a 3‑vector. Methods include `inv`, `interp`, `theta`, `t` (translation column), `T` (the matrix), `R`, `xyt`, `transl2`, `trplot2`, `Twist`. The Lecture 10 script uses `SE2(x_m, y_m, theta_m)` symbolically to derive a pose‑graph Jacobian.

**`SE3`** holds a 4×4 transform. Constructors: `SE3()` null, `SE3(x, y, z)` translation, `SE3(R, xyz)` rotation plus position, `SE3(T)` from a matrix, and static constructors `SE3.Rx/Ry/Rz(theta)`, `SE3.rpy(r, p, y)`, `SE3.eul(...)`, `SE3.angvec(...)`. Methods you meet in Lecture 3:

| Method | What it does |
|---|---|
| `T.t` | translation as a 3×1 column (single object only) |
| `T.transl` | translation as a 1×3 row; for a vector of poses, one row each |
| `T.torpy`, `T.toeul` | angles |
| `T.R`, `T.SO3`, `T.UnitQuaternion` | the rotation part in three flavours |
| `inv(T)` | inverse, "formed explicitly, no matrix inverse required" |
| `T.Ad` | the 6×6 adjoint matrix (Section 4.1) |
| `T.velxform` | the 6×6 velocity transform (Section 4.1) |
| `T.interp(T2, s)` or `T.interp(T2, n)` | interpolate poses (Section 4.6) |
| `T.animate` | animate a vector of poses |

Worked example (Lecture 3 script): `TB = SE3(1, 2, 0) * SE3.Rz(pi/2)` is a body frame at (1, 2, 0) turned 90° about z. `TB.t` returns `[1; 2; 0]`, `TB.torpy` returns `[0 0 1.5708]`, and `TB.T` is the 4×4 matrix.

**Common mistake:** `.t` only works on a single pose. On a vector of poses it returns a comma‑separated list, which breaks assignments. Use `.transl` (rows) or `.tv` (columns) for vectors; the Lecture 3 script uses `Ts.transl` for this reason.

**`SO2`** and **`SO3`** are the rotation‑only classes. `SO2(30, 'deg')` appears in the Lecture 3 script to rotate a set of via points, and `SO3.Rz(-1)*SO3.Ry(-1)` builds a 3D orientation. `SO3.rpy(rpy)` with an N×3 matrix builds a vector of rotations, which the script then animates.

---

## 4. Time and motion (Lecture 3: `Lecture_3_Time_and_Motion_Study_Notes.md`)

### 4.1 Transforming velocities: `velxform` and `Ad`

A **spatial velocity** (also called a twist) is a 6‑vector `[vx vy vz wx wy wz]`: three linear and three angular velocity components. If you know the body's velocity in its own frame {B} and the pose of {B} in the world {A}, the manual gives two 6×6 matrices for converting:

- **`J = TAB.velxform()`** "maps velocity from frame B to frame A", where `TAB` is the pose of the body in the world. The Lecture 3 script: `va = TB.velxform * vb`.
- **`J = TCB.velxform('samebody')`** "maps velocity from frame C to frame B" when both frames are attached to the same rigid body. The manual adds: "This is also the adjoint of PCB", i.e. the same as **`TCB.Ad`**. The Lecture 3 script: `vc = TCB.Ad * vb` with `TCB = inv(SE3(0, 0.4, 0))`.

Worked example with the script's numbers: `vb = [0.2 0.3 0 0 0 0.5]'` in a body at `TB = SE3(1,2,0)*SE3.Rz(pi/2)`. The rotation part of `velxform` is a block‑diagonal of `R` and `R` (translation does not matter when both frames see the same body point), so the linear velocity rotates by 90°: `(0.2, 0.3)` becomes `(−0.3, 0.2)`, and the angular part stays `(0, 0, 0.5)`. So `va = [-0.3 0.2 0 0 0 0.5]'`. For the offset frame {C}, 0.4 m along y, the angular velocity 0.5 rad/s about z adds a linear velocity of `ω × r`, so the x‑velocity changes by `0.5 × 0.4 = 0.2` (sign depends on direction). That is why `Ad` differs from `velxform`: it includes the `[t]× R` coupling block.

Wrenches (force plus torque, 6‑vector) transform with the **transpose** of the adjoint: the script writes `WC = TBC.Ad' * WB`. The classic functions are `tr2jac(T)` and `wtrans`.

**Common mistake:** using `Ad` when you mean `Ad'`. Velocities use `Ad`; wrenches use `Ad'` (transpose). They are dual quantities.

### 4.2 A single smooth motion: `tpoly`

**`[s, sd, sdd] = tpoly(s0, sf, m)`** returns an m×1 column of positions that moves smoothly from `s0` to `sf` using a quintic (5th‑order) polynomial, plus velocity `sd` and acceleration `sdd`. With no output arguments it plots all three. **`tpoly(s0, sf, m, qd0, qd1)`** sets non‑zero start and end velocities. Pass a time vector `T` instead of `m` and the outputs are scaled to real time.

Worked example: `tpoly(0, 1, 50)`. The normalised curve is `s(τ) = 6τ⁵ − 15τ⁴ + 10τ³` sampled at τ = 0, 1/49, …, 1. It starts and ends with zero velocity and acceleration, peaks in velocity at the midpoint, and the manual's Note reminds you that with `m` the velocity is "distance per trajectory step, not per second". The script computes `mean(sd)/max(sd)`, which comes out near 0.52: the average speed is only about half the peak, so a quintic wastes time at the ends. That motivates the next function.

### 4.3 The trapezoidal profile: `lspb`

**`[s, sd, sdd] = lspb(s0, sf, m)`** is a "linear segment with parabolic blend": accelerate (parabola), cruise at constant velocity (straight line), decelerate (parabola). The velocity plot is a trapezoid. **`lspb(s0, sf, m, v)`** lets you set the cruise velocity.

The manual's Note: "For some values of v no solution is possible and an error is flagged." With `d = sf − s0` and `T = m − 1` steps, the velocity must satisfy `d/T < v ≤ 2d/T`. For `lspb(0, 1, 50)`: `1/49 = 0.0204 < v ≤ 0.0408`. The script tries `v = 0.025` (long cruise, short blends) and `v = 0.035` (short cruise, long blends); `v = 0.05` would error.

**Common mistake:** `lspb` is smooth in velocity but **not** in acceleration (the acceleration jumps at the blend points). If you need continuous acceleration use `tpoly`.

### 4.4 Several axes at once: `mtraj`, `jtraj`

**`[q, qd, qdd] = mtraj(tfunc, q0, qf, m)`** applies a scalar profile to every axis: `tfunc` is `@tpoly` or `@lspb`; `q0` and `qf` are 1×N rows; the result `q` is m×N with one row per time step and one column per axis. The script: `q = mtraj(@lspb, [0 2], [1 -1], 50)` moves axis 1 from 0 to 1 and axis 2 from 2 to −1 in the same 50 steps. The manual notes all axes reach the goal together.

**`jtraj(q0, qf, m)`** is the joint‑space equivalent with a quintic, slightly faster, and allows initial/final joint velocities. It is the arm‑robot favourite.

### 4.5 Through via points: `mstraj`

**`traj = mstraj(p, qdmax, tseg, q0, dt, tacc)`** builds a multi‑segment, multi‑axis trajectory through via points. The arguments, in order:

1. `p` (M×N): via points, one row each; the last row is the destination.
2. `qdmax` (1×N): speed limit per axis, **or** `[]`.
3. `tseg` (1×M): duration of each segment, **or** `[]`. "Only one of qdmax or tseg can be specified, the other is set to []."
4. `q0` (1×N): starting coordinates.
5. `dt`: sample time.
6. `tacc`: acceleration time at every via point (scalar), or a 1×M vector for per‑segment values.

Each segment is a straight line at constant speed, and the via points are rounded off with polynomial blends of duration `tacc`. A larger `tacc` cuts the corner more (the Lecture 3 script compares `tacc = 0` and `tacc = 2`). With `qdmax` given, the function finds, for each segment, the slowest axis given its speed limit and distance, sets the segment time from that, and slows the other axes so everyone arrives together.

Worked example (script):

```matlab
via = SO2(30, 'deg') * [-1 1; 1 1; 1 -1; -1 -1]';   % a tilted square, 2x4
q0 = mstraj(via(:,[2 3 4 1])', [2 1], [], via(:,1)', 0.2, 0);
```

The via points are the square's corners rotated by 30° (transposed to rows), the speed limits are 2 for x and 1 for y, no segment times, start at the first corner, 0.2 s steps, no blending. The output has one row per 0.2 s and two columns; `plot(q0(:,1), q0(:,2))` draws the square.

### 4.6 Interpolating orientation and pose: `interp`, `trinterp`, `ctraj`

Lecture 3's rule: **never linearly interpolate the elements of a rotation matrix**; the result is not orthonormal. Interpolate angles or quaternions instead.

- **`rpy = mtraj(@tpoly, rpy0, rpy1, 50)`** interpolates roll‑pitch‑yaw triples; then `SO3.rpy(rpy).animate` shows it.
- **`q = interp(q0, q1, 50)`** or `q0.interp(q1, 50)` does spherical linear interpolation (slerp) between two `UnitQuaternion`s, which the manual describes as "interpolation along a great circle arc on a sphere". The `'shortest'` option picks the shorter of the two arcs. The script's example `interp(UnitQuaternion.Rz(-2), UnitQuaternion.Rz(2), 50)` goes the long way round (4 rad) by default and the short way (2π − 4 ≈ 2.28 rad) with `'shortest'`.
- **`T0.interp(T1, s)`** interpolates `SE3` poses: translation linearly, rotation by slerp. `s` can be a scalar fraction, a count `n`, or a vector such as `lspb(0, 1, 50)` to shape the speed profile. The script's `T0.interp(T1, lspb(0,1,50))` makes the frame accelerate, cruise and decelerate.
- **`ctraj(T0, T1, n)`** is the compact form: Cartesian trajectory with a trapezoidal (`lspb`) profile and quaternion orientation interpolation. Classic form `trinterp(T0, T1, s)` works on 4×4 matrices.

**Common mistake:** `interp` errors if `s` is outside 0 to 1. Normalise your parameter first.

---

## 5. Mobile robots (Lecture 4: `Lecture_4_Mobile_Robotics_Study_Notes.md`)

### 5.1 The vehicle classes: `Vehicle`, `Bicycle`, `Unicycle`

`Vehicle` is an abstract parent. **`Bicycle`** models a car‑like (Ackermann) vehicle; **`Unicycle`** models a differential‑drive robot (like the TurtleBot). Both "update the true vehicle state and return noise‑corrupted odometry readings".

Constructor options (`Bicycle`): `'covar', C` odometry covariance (2×2, default 0), `'steermax', M` maximum steer angle (default 0.5 rad), `'speedmax', S` (default 1 m/s), `'L', L` wheelbase (default 1 m), `'x0', x0` initial state, `'dt', T` time step (default 0.1 s), `'accelmax'`, `'rdim'` drawing size.

Key methods:

| Method | Meaning |
|---|---|
| `odo = veh.step(speed, steer)` | advance one time step, return **noisy** odometry `[distance, heading_change]` |
| `veh.x` | true state `[x y theta]` (3×1 column) |
| `xn = veh.f(x, odo)` | predicted next state from a state and an odometry reading (the motion model the EKF uses) |
| `veh.Fx(x, odo)`, `veh.Fv(x, odo)` | Jacobians of `f` with respect to state (3×3) and odometry noise (3×2) |
| `veh.add_driver(d)` | attach a driver object that supplies speed and steer each step |
| `veh.run(n)` | simulate `n` steps with animation; `p = veh.run(n)` returns the n×3 history instead |
| `veh.plot_xy(ls)` | plot the true path from `x_hist` |
| `veh.x_hist` | the history of true states (N×3) |

Worked example (Lecture 10 script, first lines):

```matlab
V = diag([0.02, 0.5*pi/180].^2);   % variances: (2 cm)^2 and (0.5 deg)^2
veh = Bicycle('covar', V)
odo = veh.step(1, 0.3)             % speed 1 m/s, steer 0.3 rad, for dt = 0.1 s
```

With no noise the distance would be `1 × 0.1 = 0.1 m` and the heading change `dt·v/L·tan(0.3) = 0.1 × 0.309 = 0.031 rad`. Because `covar` is set, `odo` comes back close to `[0.1 0.031]` but not exactly: that is the simulated wheel‑encoder noise. `veh.x'` shows the true state has moved to about `[0.1 0 0.031]`. Then `veh.f([0 0 0], odo)` predicts where you would *think* you are from the noisy odometry: `x = 0 + 0.1·cos(0)`, `y = 0`, `θ = 0.031` plus the noise. The gap between truth and prediction is the whole reason Lecture 10 exists.

**Common mistakes.**
- The covariance option is named `'covar'`, and the matrix is a **covariance** (variances on the diagonal), so square your standard deviations: `diag([0.02, 0.5*pi/180].^2)`, not `diag([0.02, 0.5*pi/180])`.
- These are **handle** classes ("pass by reference semantics apply"). When `ekf.run(1000)` drives the vehicle, your `veh` variable changes too; there is no copy. That is why `veh.plot_xy` works after the EKF has run.
- The state `veh.x` is a column but `f` and `x_hist` use rows `[x y theta]`. Transpose with `veh.x'` to print it nicely, as the script does.

### 5.2 Drivers: `RandomPath`

**`d = RandomPath(dim, options)`** is a "driver" that steers the vehicle to random waypoints inside a region: `dim` scalar means ±dim in x and y; `[dx dy]` or `[xmin xmax ymin ymax]` for other shapes. Options `'speed', S` (default 1 m/s) and `'dthresh', d` (how close counts as "arrived"). Attach it with `veh.add_driver(RandomPath(10))`, and every `veh.step()` with no arguments asks the driver for speed and steer. It uses its own random stream so it does not disturb other randomised algorithms.

**`randinit`** resets MATLAB's default random stream so a simulation repeats exactly. The Lecture 10 script calls it before every experiment, which is why your plots should match the lecturer's.

### 5.3 Drawing a robot: `plot_vehicle`, `Vehicle.plotv`

**`plot_vehicle(x, options)`** draws an oriented triangle at pose `x = [x y theta]`; give an N×3 matrix and it animates. Options `'box'` for a rectangle, `'size', S`, `'scale', S`, `'fillcolor', F`, `'alpha', A`, `'fps', F`, `'retain'` to leave a trail, `'image', IMG` to use a picture. The Lecture 7 script: `plot_vehicle(p, 'box', 'size', [10 15], 'fill', 'r', 'alpha', 0.1)` sweeps a translucent red car along an RRT path. (`'fill'` is accepted as a synonym for `'fillcolor'`.) The manual advises setting `axis()` first because the vehicle is drawn relative to the axes size.

### 5.4 The Simulink models (Section 1.2.4)

The `simulink` folder holds block diagrams the course uses in Lectures 4, 6 and 7. The manual lists them:

| Model | What it does |
|---|---|
| `sl_braitenberg` | Braitenberg vehicle moving towards a source (Lecture 7) |
| `sl_lanechange` | lane‑changing control (Lecture 4) |
| `sl_drivepoint` | drive to a point (Lecture 4) |
| `sl_driveline` | drive to a line (Lecture 4) |
| `sl_drivepose` | drive to a pose (Lecture 4) |
| `sl_pursuit` | drive along a path, pure pursuit (Lecture 4) |
| `sl_quadrotor` | quadrotor control (Lecture 6) |
| `sl_quadrotor_vs` | quadrotor visual servoing |
| `roblocks` | the block palette |
| `sl_rrmc`, `sl_jspace`, `sl_ctorque`, … | arm‑robot control schemes |

Release 10 "tidied" them: red blocks are parameters you set, shaded boxes group parts. You run them from a script with `out = sim('sl_drivepoint')` after setting workspace variables such as `xg = [5 5]` and `x0 = [8 5 pi/2]` (the Chapter 4 script does this), then read results with `out.get('y')` or `r.find('y')`. **`mplot`** (Section 9) is built to plot such simulation output.

**`mdl_quadrotor`** is a script that creates a structure `quad` with the physical parameters the `sl_quadrotor` model needs: number of rotors, inertia matrix `J`, arm length `d`, rotor radius, blade mass, thrust and torque coefficients `Ct` and `Cq`, and so on. Lecture 6 explains what those parameters do.

---

## 6. Navigation and path planning (Lecture 7: `Lecture_7_Navigation_Part_I_Reactive_Study_Notes.md` and `Lecture_7_Navigation_Part_II_Map_Based_Study_Notes.md`)

### 6.1 The shared pattern: the `Navigation` superclass

All planners inherit from `Navigation`, which stores an **occupancy grid** (a matrix where 0 is free and non‑zero is occupied), a `goal`, a `start`, and the random seed `seed0` so you can rerun an interesting experiment. The shared methods are:

- **`plan(...)`**: do the goal‑dependent preparation (compute a cost map, build a roadmap or tree).
- **`query(start, goal)`** or `query(start)`: produce (and animate) a path from `start`. "Return/animate a path from start to goal." Give `[]` as start and you are asked to click on the map.
- **`plot()`** and `plot(p)`: show the grid, optionally with a path overlaid; option `'distance', D` shows a distance field behind it, `'inflated'` shows the inflated obstacles.
- Constructor options: `'goal', G`, `'inflate', K` (grow every obstacle by K cells, which is how you account for the robot's size), `'seed', S`, `'private'`.

Two conventions the manual repeats and students trip over:

**Common mistake 1:** coordinates are given as **[X, Y]**, "not as MATLAB row and column coordinates". So `[20, 10]` means x = 20 (column 20), y = 10 (row 10).

**Common mistake 2:** calling `query` before `plan` on a planner that needs a plan (`Dstar`, `DXform`, `PRM`, `Lattice`, `RRT`). `Bug2` is the exception: "The Bug2 algorithm is completely reactive so there is no planning method." You go straight to `query`.

**Everyday analogy:** `plan` is studying the map at home; `query` is actually walking. Bug2 never looks at a map, it just walks and feels for walls.

The maps used in the lectures come with the toolbox's `data` folder: `load map1` (a 100×100 grid), `load house` (a floor plan, with a struct `place` of named rooms such as `place.kitchen` and `place.br3`), and `load road`.

### 6.2 `Bug2`

```matlab
load map1
bug = Bug2(map);
bug.plot()
bug.query([20 10], [50 35], 'animate');   % animate the path
p = bug.query([20 10], [50 35]);          % or just get the N x 2 path
```

Options for `query`: `'animate'`, `'movie', M`, and `'current'` to show the robot as a black circle when it backtracks a lot. The manual cites Lumelsky and Stepanov (1986), the original bug paper. The house version is `Bug2(house)` with `bug.query(place.br3, place.kitchen)`.

### 6.3 `DXform` and `Dstar`

Both build a **distance or cost map**: every free cell gets a number equal to its shortest distance to the goal; obstacles are `NaN` (DXform) or `Inf` (Dstar). The path from any start is then found by walking downhill.

**`DXform(map)`** uses a distance transform (`distancexform`). `dx.plan(goal)` computes the map (this "may take many seconds"), `dx.query(start)` walks it, `dx.plot3d()` shows the distance function as a surface with obstacles cut out. Options `'metric', 'euclidean'` (default) or `'cityblock'`.

**`Dstar(map)`** implements D* (Stentz, 1994), "minimum distance paths and incremental replanning". The lecture sequence:

```matlab
load house
ds = Dstar(house);
ds.plot                      % the grid
c = ds.costmap();            % the cost map (unit cost per free cell, Inf for obstacles)
ds.plan(place.kitchen, 'animate');   % compute distances to the kitchen
p = ds.query(place.br3);     % path from bedroom 3
ds.plot(p)
ds.niter                     % how many iterations the plan took
ds.modify_cost([300,325; 115,125], 50);   % make a rectangle expensive
ds.plan                      % REPLAN: incremental, cheaper than a full plan
p = ds.query(place.br3);
ds.plot(p); ds.niter         % fewer iterations than the first plan
```

About `modify_cost(p, C)`: `p` is 2×N with one **point per column**. "For the particular case where p (2×2) the first and last columns define the corners of a rectangular region which is set to C." So `[300,325; 115,125]` means column 1 is (300, 115) and column 2 is (325, 125): a rectangle from x 300 to 325 and y 115 to 125. The slide used cost 5, the script uses 50; either way the planner routes around it. The manual's Note: "After one or more point costs have been updated the path should be replanned by calling DS.plan()", and `plan` without arguments replans towards the goal set earlier. `ds.reset()` forces a full plan instead of an incremental one.

⚠️ The lecture slide says "The goal cannot be changed!" and the manual's `Dstar.plan(goal)` signature does accept a goal. The point of D* is that the cost map is built *for one goal*; changing the goal means a brand‑new plan, so you lose the incremental‑replanning advantage. Changing the **start** is free.

⚠️ `ds.costmap()` is written as a method call in the lecture, but the manual lists `costmap` as a read‑only **property**. Both `ds.costmap` and `ds.costmap()` work in MATLAB for a property, so it is harmless, but do not look for a method entry.

### 6.4 `PRM` (probabilistic roadmap)

```matlab
load house
prm = PRM(house);
prm.plan('npoints', 200);     % sample 200 random free points and connect near ones
prm.plot()                    % the roadmap graph
prm.query(place.br3, place.kitchen);
prm.plot()                    % now with the path
```

Options: `'npoints', N` (default 100) and `'distthresh', D` (edges only connect nodes closer than D, default 0.3 of the grid size). The plan is **goal‑independent**: build the roadmap once, query many start–goal pairs. Because nodes are random, too few points can leave the start and goal disconnected, which is what the slide's "100 vs 150 nodes" comparison shows. Use `randinit` or the `'seed'` option to reproduce a specific roadmap.

### 6.5 `Lattice` (kinematically feasible paths)

A lattice planner builds a tree of short motion primitives (go straight, turn left, turn right) on a grid, so the resulting path respects a car's turning constraints and the state is a **pose** `(x, y, θ)`, not just a point.

```matlab
lp = Lattice();
lp.plan('iterations', 2);  lp.plot()       % a small tree
lp.plan('iterations', 8);  lp.plot()
lp.query([1 2 pi/2], [2 -2 0]);  lp.plot   % start and goal are 1x3 poses
lp = Lattice();
lp.plan('cost', [1 10 10], 'iterations', 8);   % straight is cheap, turns expensive
lp.query([1 2 pi/2], [2 -2 0]);  lp.plot()
load road
lp = Lattice(road, 'grid', 5, 'root', [50 50 0]);  % on a real map
lp.plan();  lp.query([30 45 0], [50 20 0]);  lp.plot()
```

Options: `'grid', G` spacing, `'root', R` where the lattice is anchored, `'iterations', N` (default `Inf`, which "iterates until the area defined by the map is covered"), `'cost', C` for `[straight left right]`, `'inflate', K`. The cost vector is the knob the lecture plays with: `[1 10 10]` makes the planner prefer long straight runs.

### 6.6 `RRT` (rapidly exploring random tree)

RRT is a **kinodynamic** planner: it grows a tree by repeatedly simulating the actual vehicle model towards random points, so every branch is a drivable path.

```matlab
car = Bicycle('steermax', 0.5);        % the vehicle model, with a turning limit
rrt = RRT(car, 'npoints', 1000);       % tree with 1000 nodes
rrt.plan();  rrt.plot();               % 3D plot: x, y and heading on the vertical axis
load map1
rrt = RRT(car, road, 'npoints', 1000, 'root', [50 22 0], 'simtime', 4);
rrt.plan();
p = rrt.query([40 45 0], [50 22 0]);
rrt.plot(p)
plot_vehicle(p, 'box', 'size', [10 15], 'fill', 'r', 'alpha', 0.1)
```

Constructor options: `'npoints', N` (default 500), `'simtime', T` how long to drive towards each random point (default 0.5 s), `'root', R` the tree's root pose (default `[0 0 0]`), `'goal', P`, `'speed', S`, `'revcost', C` penalty for reversing, `'range', R` workspace bounds (ignored if a map is given). `plan` options include `'ntrials', N`: at each step the planner tries N random steer/speed pairs and keeps the one that gets closest, which the manual admits is "computationally slow". In `query`'s plot, blue segments are forward motion and red are reverse.

⚠️ Two things in the manual entry are out of date or thin. The `RRT.query` text still says `x = R.path(start, goal)`: `path` was renamed `query` in Release 10, so type `query`. And the Note warns that "the path starts at the vertex closest to the start state, and ends at the vertex closest to the goal state. If the tree is sparse this might be a poor approximation" — so more points gives a better path, exactly the "more iterations cover the space better" message of the lecture.

⚠️ The lecture script's second RRT call passes `road` as the map but loads `map1` first; `road` comes from the earlier `load road`. If you run the sections out of order you will get "undefined variable road".

### 6.7 Helpers: `distancexform`, `bresenham`

**`d = distancexform(occgrid, goal)`** returns the distance‑to‑goal map that `DXform` uses; `NaN` marks obstacles; options `'euclidean'` or `'cityblock'`; `'show', d` animates the wavefront. The manual notes it is much faster if the Image Processing Toolbox or VLFeat is installed. **`p = bresenham(x1, y1, x2, y2)`** lists the integer grid cells on a straight line between two cells, which is how a planner checks whether a straight segment crosses an obstacle (the collision check in the RRT algorithm of the lecture).

---

## 7. Localization and SLAM (Lecture 10: `Lecture_10_Localization_Study_Notes.md`)

### 7.1 The cast of objects

The `EKF` entry explains the set‑up: an EKF is "used in conjunction with" a kinematic **vehicle** that produces odometry (a `Vehicle` subclass), a **driver** that moves it around (`RandomPath`), a **map** of landmark positions (`LandmarkMap`), and a **sensor** that measures landmarks relative to the vehicle (`RangeBearingSensor`). The EKF "invokes the state update methods of the vehicle object" each step and stores "the complete history of estimated state and covariance".

**Everyday analogy:** the vehicle is a walker counting steps in the dark (odometry), the map is the list of lamp posts, the sensor is a hand that occasionally touches a lamp post and reports how far and in which direction it is, and the EKF is the brain merging the two.

### 7.2 `LandmarkMap`

**`map = LandmarkMap(n, dim)`** creates `n` random point landmarks in the square ±dim (default dim from the manual example is 10 m). Properties: `map.map` (2×N coordinates, one column per landmark), `map.dim`, `map.nlandmarks`. Methods: `map.plot()` draws black diamonds and leaves `hold on`; `map.landmark(k)` returns the k‑th landmark as a 2×1 column. The Lecture 10 script: `map = LandmarkMap(20, 10); map.plot(); map.landmark(17)`.

### 7.3 `RangeBearingSensor`

**`sensor = RangeBearingSensor(veh, map, 'covar', W)`** mounts a sensor on the vehicle that observes landmarks in the map. Each reading is `z = [r, β]`: range and bearing, with Gaussian noise of covariance `W` (2×2). Options: `'range', xmax` or `[xmin xmax]`, `'angle', TH` for a field of view ±TH or `[THMIN THMAX]`, `'skip', K` return a reading only every K‑th call, `'fail', [TMIN TMAX]` simulate a sensor failure, `'animate'` draw a line from the robot to the observed landmark and shade the field of view.

Methods, and what Lecture 10 calls them in the equations:

| Method | Size | Meaning |
|---|---|---|
| `[z, k] = sensor.reading()` | 1×2, scalar | noisy range/bearing to a random *visible* landmark, and its id `k` (`z = []`, `k = 0` if nothing is visible) |
| `z = sensor.h(x, k)` | 1×2 | the **observation model**: range and bearing from pose `x` to landmark `k` (field of view not applied) |
| `sensor.Hx(x, k)` | 2×3 | Jacobian of `h` with respect to vehicle pose |
| `sensor.Hp(x, k)` | 2×2 | Jacobian of `h` with respect to landmark position |
| `sensor.Hw(x, k)` | 2×2 | Jacobian of `h` with respect to sensor noise |
| `p = sensor.g(x, z)` | 2×1 | the **inverse** model: where a landmark is in the world, given a pose and a reading (used to add new landmarks to the map) |
| `sensor.Gx(x, z)`, `sensor.Gz(x, z)` | 2×3, 2×2 | Jacobians of `g` with respect to pose and reading |
| `sensor.landmarklog` | — | which landmark was seen at each time step |

Worked example (script): `W = diag([0.1, 1*pi/180].^2)` means range noise of standard deviation 0.1 m and bearing noise of 1°. Then `[z, i] = sensor.reading()` might return `z = [7.84, −1.23]` and `i = 17`: landmark 17 is about 7.8 m away, 1.23 rad (70°) to the right. Compare with `map.landmark(17)` and the vehicle pose to check.

### 7.4 `EKF`: one class, four jobs

The constructor has two forms, and what you pass as `[]` selects the job (from the manual's Notes: "If map is [] then it will be estimated. If v_est and p0 are [] the vehicle is assumed error free and the filter will only estimate the landmark positions").

| Job (Lecture 10 section) | Constructor | What is estimated |
|---|---|---|
| Dead reckoning (6.1) | `EKF(veh, V_est, P0)` | vehicle pose only, from odometry |
| Localization with a known map (6.2) | `EKF(veh, V_est, P0, sensor, W_est, map)` | vehicle pose, corrected by landmark observations |
| Mapping with a known pose (6.3) | `EKF(veh, [], [], sensor, W_est, [])` | landmark positions only; vehicle assumed perfect |
| SLAM (6.4) | `EKF(veh, V_est, P0, sensor, W_est, [])` | vehicle pose **and** landmark positions together |

Arguments: `V_est` is the odometry covariance the filter *believes* (2×2), `P0` the initial pose covariance (3×3), `W_est` the sensor covariance the filter believes (2×2). Options: `'verbose'`, `'nohistory'`, `'joseph'` (a numerically safer covariance update, on by default), `'dim', D`.

Worked example of the dead‑reckoning run (script):

```matlab
P0  = diag([0.005, 0.005, 0.001].^2);   % we are quite sure where we start
ekf = EKF(veh, V, P0);
ekf.run(1000);                          % 1000 steps; the vehicle is driven by its RandomPath
veh.plot_xy()                           % true path (blue)
hold on; ekf.plot_xy('r')               % estimated path (red)
P700 = ekf.history(700).P;              % 3x3 covariance at step 700
sqrt(P700(1,1))                         % standard deviation of x, in metres
ekf.plot_ellipse('g')                   % 95% ellipses every 20 steps
ekf.plot_P()                            % covariance magnitude against time
```

`sqrt(P700(1,1))` is the standard deviation of the x estimate after 700 steps. It will be far larger than the 5 mm you started with, and `plot_P` shows the growth: dead reckoning drifts without bound. Add the sensor and map (job 2) and `plot_P` becomes a sawtooth: uncertainty grows between observations and drops at each one.

Methods and properties you will use:

| Name | Meaning |
|---|---|
| `ekf.run(n)` | run `n` time steps |
| `ekf.plot_xy(ls)` | estimated vehicle path |
| `ekf.plot_ellipse(ls)` | covariance ellipses along the path; options `'interval', I` (default 20), `'confidence', C` (default 0.95) |
| `ekf.plot_P()` | covariance magnitude versus time |
| `ekf.plot_map(ls)` | estimated landmarks with confidence ellipses, e.g. `plot_map('g', 'confidence', 0.97)` |
| `ekf.plot_error('confidence', 0.95, 'nplots', 4)` | estimation error with standard‑deviation bounds |
| `ekf.history(k)` | struct with the detailed state at step `k`, including `.P` |
| `ekf.x_est`, `ekf.P` | current estimated state vector and covariance |
| `ekf.landmarks` | 2×N table mapping each landmark id to its position in the state vector and how many times it was seen |
| `ekf.get_map()` | estimated landmark coordinates, `NaN` for unseen ones |
| `ekf.init()` | reset |

Worked example of reading the map estimate (script, mapping job): `ekf.landmarks(:,6)` returns something like `[19; 25]`, meaning landmark 6 lives at state‑vector positions 19 and 20 and has been observed 25 times. Then `ekf.x_est(19:20)'` is its estimated (x, y) and `ekf.P_est(19:20, 19:20)` its 2×2 covariance. The lecture's remark "observing one landmark provides no new information about any other landmark" is visible here: in pure mapping, the off‑diagonal blocks between different landmarks stay zero.

⚠️ The manual lists the covariance property as `P`, while the lecture script uses `ekf.P_est`. Both exist in the Release 10 code (`P_est` is the one the script relies on), but if one of them errors on your version, try the other.

⚠️ The `EKF` entry's example code is Release 9 (`Vehicle(V)`, `PointMap`, and `RandomPath(20, 2)` where the second argument is a speed). Use the Release 10 forms from the lecture script.

### 7.5 `ParticleFilter` (brief)

**`pf = ParticleFilter(veh, sensor, Q, L, nparticles)`** does Monte Carlo localization: a cloud of candidate poses ("particles") is moved by odometry with added noise `Q` (3×3), weighted by how well each one explains the sensor reading through the likelihood covariance `L` (2×2), and resampled. Methods `run`, `plot_xy`, `plot_pdf`; properties `x` (particles), `weight`, `x_est` (mean), `std`. It is in `rtbdemo` but not in your lectures; it is the main alternative to the EKF when the uncertainty is not Gaussian or the starting pose is unknown.

### 7.6 `PoseGraph`, laser scans and ICP

The Lecture 10 script uses:

```matlab
pg = PoseGraph('pg1.g2o');  pg.plot();  pg.optimize('animate');
pg = PoseGraph('killian-small.toro');  pg.plot();  pg.optimize('animate');
pg = PoseGraph('killian.g2o', 'laser');
[r, theta] = pg.scan(2580);            % one laser scan: ranges and angles
[x, y] = pol2cart(theta, r);  plot(x, y, '.')
p2580 = pg.scanxy(2580);  p2581 = pg.scanxy(2581);   % scans as 2xN point sets
T = icp(p2581, p2580, 'verbose', 'T0', transl2(0.5, 0), 'distthresh', 3);
pg.time(2581) - pg.time(2580)
map = pg.scanmap();  pg.plot_occgrid(map)
```

What the manual tells you: `PoseGraph` reads g2o‑format files, where each `VERTEX` line is a pose `id X Y THETA` and each `EDGE` line is a constraint between two vertices with a relative pose and an information matrix, and "vertex numbers start at 0". `optimize` does graph relaxation ("bundle adjustment and pose graph SLAM" in the Release 10 notes). The data files `pg1.g2o`, `killian-small.toro` and `killian.g2o` live in the toolbox `data` folder.

⚠️ The `PoseGraph` entry in this manual is almost empty: it documents only the constructor and an internal method `linear_factors`. The methods the lecture uses (`plot`, `optimize`, `scan`, `scanxy`, `scanmap`, `plot_occgrid`, `time`) are **not described** in this PDF. Use `help PoseGraph` in MATLAB, which reads the current source comments, or Chapter 6 of RVC2.

⚠️ **`icp` is not in this manual at all.** Iterated closest point lives in Peter Corke's *Machine Vision Toolbox* (`vision` folder), which the Lecture 10 slides assume is installed alongside. If `icp` is undefined, install the Machine Vision Toolbox (the same `.mltbx` mechanism, `vision.mltbx`). The call `icp(p2581, p2580, 'T0', transl2(0.5,0), 'distthresh', 3)` returns the 3×3 transform that best aligns scan 2581 onto scan 2580, starting from an initial guess of 0.5 m forward and ignoring point pairs further apart than 3 m. That transform is the robot's motion between the two scans: laser odometry.

---

## 8. The manipulator side, in brief

No lecture in this workspace covers robot arms, but the course goals mention "fundamental principles of robotic manipulators" and more than half the manual is about them, so here is the map of that territory.

- **`Link`**, **`Revolute`**, **`Prismatic`** describe one joint‑plus‑link with Denavit–Hartenberg parameters `theta`, `d`, `a`, `alpha` (and `RevoluteMDH`/`PrismaticMDH` for the modified convention), plus dynamic parameters such as mass `m`, centre of mass `r`, inertia `I`, gear ratio `G`. A **revolute** joint rotates; a **prismatic** joint slides.
- **`SerialLink`** chains links into a robot: `R = SerialLink([L1 L2 L3])` or simply `L1 + L2 + L3`. Key methods: `R.fkine(q)` **forward kinematics** (joint angles in, end‑effector pose out as an `SE3` object in Release 10); `R.ikine(T)` **inverse kinematics** by numerical optimisation (pose in, joint angles out; options `'mask'` for arms with fewer than 6 joints, `'q0'` initial guess); `R.jacob0(q)` the 6×N **Jacobian** mapping joint velocities to end‑effector spatial velocity in the world frame (`jacobe` for the end‑effector frame); `R.plot(q)` draws the arm; `R.teach()` adds sliders so you can drive it by hand; `R.rne(q, qd, qdd)` **inverse dynamics**, the joint torques needed for a given motion (with `'gravity'` and `'fext'` options).
- **Models**: `mdl_puma560` creates `p560` plus the handy configurations `qz` (zero), `qr` (ready), `qn` (nominal, non‑singular), `qstretch`; `mdl_twolink` creates a planar two‑link arm in the vertical plane under gravity; dozens of others (`mdl_ur5`, `mdl_baxter`, `mdl_kr5`, …) and `models` lists them.
- **`jtraj(q0, qf, m)`** makes a quintic joint‑space trajectory, and `R.plot(q)` with an m×N matrix animates it.
- **`DHFactor`** and **`ETS2`/`ETS3`** are two gentler ways in: write the arm as a string of elementary transforms such as `'Rz(q1).Rx(q2).Ty(L1)'` and let the toolbox factor it into DH parameters or just compute with it directly (`trchain` does the same for a one‑off chain).

A one‑minute taste:

```matlab
mdl_puma560
p560.plot(qn)            % draw it in the nominal configuration
T = p560.fkine(qn)       % where is the gripper? an SE3 object
q = p560.ikine(T)        % get joint angles back (one of several solutions)
p560.teach               % drive it with sliders
```

---

## 9. Utilities and graphics you will see in the scripts

| Function | What it does | Example |
|---|---|---|
| `about(x)` or `about x` | prints class and dimensions, e.g. `a [double] : 5x7 (280 bytes)` | the Lecture 3 and 10 scripts use it to check sizes |
| `angdiff(th1, th2)` | difference of two angles wrapped into [−π, π); `angdiff(th)` wraps one angle | use it for heading errors so 359° − 1° is −2°, not 358° |
| `unit(v)` | unit vector parallel to `v` | normalising an axis |
| `numrows(m)`, `numcols(m)` | readable `size(m,1)`, `size(m,2)` | the Lecture 3 script compares trajectory lengths with `[numrows(q0) numrows(q2)]` |
| `randinit` | reset the default random stream | before every stochastic experiment |
| `mplot(t, y)` or `mplot(out)` | plot time‑series data in stacked subplots; accepts a Simulink `sim` output directly | `mplot(t, q(:,2:4))` in the Chapter 4 script |
| `plot_ellipse(E, C, ls)` | draw the ellipse `x'Ex = 0` (2×2) centred at `C`; option `'confidence', C` scales a covariance matrix to a confidence region | what `EKF.plot_ellipse` calls |
| `plot_poly(p, options)` | polygon from 2×N columns; `'fill'`, `'alpha'`, `'animate'` | the Lecture 10 script fills the landmark‑seen log |
| `plot_box(...)`, `circle(C, R)`, `plot_point(p)` | boxes, circles, points | annotations |
| `plotvol(w)` | set 2D or 3D axes with equal aspect and hold on | before `trplot2` |
| `xaxis(min, max)`, `yaxis(min, max)` | set one axis range; call with no arguments to restore auto | tidy plots |
| `rtbdemo` | demo menu | explore |

---

## 10. Common mistakes, collected

1. **Radians by default.** Add `'deg'` when you mean degrees: `rot2(30, 'deg')`, `trot2(30, 'deg')`, `SO2(30, 'deg')`, `UnitQuaternion.Rz(90, 'deg')`.
2. **Order of multiplication matters.** `T1*T2` applies `T2` in the frame of `T1`. `R*X` rotates about the world origin; `X*R` rotates about X's origin.
3. **Columns for points, rows for time.** Points are 2×N or 3×N; trajectories and vehicle states are N×3 with one row per step.
4. **Homogeneous versus Euclidean.** Append the 1 (`e2h`) before multiplying by a transform, or use `homtrans`, which does it for you.
5. **Object or matrix?** `SE3(...)` gives an object; `transl(...)` gives a matrix. Convert with `.T`, `double()` or `SE3()`.
6. **`transl`/`transl2` go both ways.** With numbers in, a transform comes out; with a transform in, the translation comes out.
7. **`'covar'` takes a covariance.** Square your standard deviations: `diag([0.02, 0.5*pi/180].^2)`.
8. **Handle classes.** `Bicycle`, `EKF`, `RandomPath`, the planners and `LandmarkMap` are reference objects. There is one copy; everybody who holds it sees the changes.
9. **`plan` before `query`**, except for `Bug2`, which has no `plan`. After `modify_cost`, call `plan` again.
10. **Grid coordinates are [X, Y]**, not `[row, col]`.
11. **Euler angles are not unique** and `tr2eul` returns θ ≥ 0; θ = 0 (ZYZ) or pitch = ±π/2 (RPY) is a singularity.
12. **Rotations are multiplied, not added.** `q1 + q2` is not a rotation. Linear interpolation of rotation matrices is not a rotation either; use `interp`, `ctraj` or angle interpolation.
13. **`lspb` velocity window.** `d/T < v ≤ 2d/T`, otherwise an error.
14. **`mstraj`: give `qdmax` or `tseg`, not both.** Set the other to `[]`.
15. **`Ad` for velocities, `Ad'` for wrenches.**
16. **Reproducibility.** Call `randinit` (or pass `'seed'`) if you want the same random landmarks, roadmap or path as the lecturer.

---

## 11. Cheat sheet: task → function

| I want to… | Use |
|---|---|
| make a 2D rotation / transform | `rot2`, `trot2`, `transl2`, or `SE2(x, y, theta)` |
| make a 3D rotation / transform | `rotx/roty/rotz`, `trotx/troty/trotz`, `transl`, or `SE3(...)`, `SE3.Rz(...)` |
| angles ↔ rotation matrix | `eul2r/tr2eul` (ZYZ), `rpy2r/tr2rpy` (ZYX), `angvec2r/tr2angvec`, `tripleangle` app |
| quaternions | `UnitQuaternion(R)`, `.Rz`, `q*q2`, `q*v`, `inv(q)`, `q.R`, `q.interp` |
| split/join rotation and translation | `t2r`, `r2t`, `tr2rt`, `rt2tr`, `transl(T)` |
| transform points | `homtrans(T, p)` or `h2e(T*e2h(p))` |
| skew‑symmetric ↔ vector; exp/log | `skew/vex`, `skewa/vexa`, `trexp/trlog`, `logm/expm` |
| draw / animate a frame | `plotvol`, `trplot2`, `trplot`, `tranimate`, `T.plot`, `T.animate` |
| velocity / wrench between frames | `T.velxform`, `T.Ad` (velocity), `T.Ad'` (wrench), `tr2jac`, `wtrans` |
| one‑axis smooth motion | `tpoly` (quintic), `lspb` (trapezoid) |
| many axes, two points | `mtraj(@lspb or @tpoly, q0, qf, m)`, `jtraj` |
| many axes through via points | `mstraj(p, qdmax, tseg, q0, dt, tacc)` |
| interpolate orientation / pose | `q0.interp(q1, n, 'shortest')`, `T0.interp(T1, s)`, `ctraj`, `trinterp` |
| simulate a car / diff‑drive robot | `Bicycle('covar', V)`, `Unicycle(...)`, `.step`, `.f`, `.Fx`, `.Fv`, `.run`, `.plot_xy` |
| drive it randomly | `veh.add_driver(RandomPath(dim))` |
| Simulink demos | `sl_drivepoint`, `sl_driveline`, `sl_pursuit`, `sl_drivepose`, `sl_lanechange`, `sl_braitenberg`, `sl_quadrotor`, `mdl_quadrotor` |
| reactive navigation | `Bug2(map).query(start, goal, 'animate')` |
| grid planning with replanning | `Dstar(map)`, `.plan(goal)`, `.query(start)`, `.modify_cost`, `.plan`, `.niter` |
| distance‑transform planning | `DXform(map)`, `.plan`, `.query`, `.plot3d` |
| sampling‑based planning | `PRM(map).plan('npoints', N)` then `.query`; `RRT(veh, map, 'npoints', N)` then `.plan`, `.query` |
| kinematic lattice | `Lattice(...)`, `.plan('cost', [s l r], 'iterations', N)`, `.query(pose, pose)` |
| landmarks and a sensor | `LandmarkMap(n, dim)`, `RangeBearingSensor(veh, map, 'covar', W)`, `.reading`, `.h`, `.Hx`, `.Hw`, `.g` |
| dead reckoning / localization / mapping / SLAM | the four `EKF(...)` forms in Section 7.4; `.run`, `.plot_xy`, `.plot_ellipse`, `.plot_P`, `.plot_map`, `.history(k).P` |
| pose‑graph SLAM, laser scans | `PoseGraph(file)`, `.plot`, `.optimize`, `.scan`, `.scanxy`, `.scanmap`, `.plot_occgrid` (see ⚠️) and `icp` from the Machine Vision Toolbox |
| robot arms | `Link/Revolute/Prismatic`, `SerialLink`, `.fkine`, `.ikine`, `.jacob0`, `.plot`, `.teach`, `.rne`, `mdl_puma560`, `jtraj` |
| sizes, angles, randomness | `about`, `numrows/numcols`, `angdiff`, `unit`, `randinit` |
| plotting helpers | `mplot`, `plot_ellipse`, `plot_poly`, `plot_box`, `plot_point`, `plot_vehicle`, `circle`, `xaxis/yaxis` |
| learn by example | `rtbdemo`, `help <name>`, `doc <name>`, the "Functions by category" pages |

---

## In a nutshell

The Robotics Toolbox is Peter Corke's free MATLAB library behind every "MATLAB time" in this course, and its manual is an alphabetical, auto‑generated reference whose entries all follow the same layout: name, call signatures with matrix sizes, options, notes, see‑also. You can work with plain matrices (`rot2`, `transl`, `rpy2r`) or with the Release 10 pose classes (`SE2`, `SE3`, `UnitQuaternion`) that wrap them and add methods like `interp`, `Ad` and `velxform`. Trajectories come from `tpoly`, `lspb`, `mtraj`, `mstraj` and `ctraj`, and orientation is interpolated with quaternion slerp rather than by averaging matrices. Mobile‑robot work builds on handle classes: a `Bicycle` or `Unicycle` driven by a `RandomPath`, observed through a `RangeBearingSensor` on a `LandmarkMap`, and estimated by one `EKF` class that does dead reckoning, localization, mapping or SLAM depending on which arguments you leave empty. Planners share the `plan` then `query` pattern on an occupancy grid with [X, Y] coordinates, except the reactive `Bug2`. The big traps are radians versus degrees, multiplication order, column versus row vectors, and expecting the 2017 manual to document everything your newer scripts call (`PoseGraph` methods and `icp` are the gaps).

## Key terms

- **Robotics Toolbox (RTB):** Peter Corke's MATLAB library for poses, trajectories, mobile robots, planning, localization and arms.
- **RVC2:** the second edition of *Robotics, Vision and Control*, the textbook this release matches.
- **Homogeneous transform:** a 3×3 (2D) or 4×4 (3D) matrix holding rotation and translation together.
- **SO(2), SO(3):** the sets of 2D and 3D rotation matrices; also the names of the toolbox classes.
- **SE(2), SE(3):** the sets of 2D and 3D rigid‑body poses; also toolbox classes.
- **RTBPose:** the abstract parent of `SO2`, `SE2`, `SO3`, `SE3`.
- **UnitQuaternion:** a four‑number representation of 3D rotation with `s² + |v|² = 1`.
- **Hamilton product:** quaternion multiplication, i.e. composing rotations.
- **Slerp:** spherical linear interpolation between two rotations along a great‑circle arc.
- **Euler angles (ZYZ) / roll‑pitch‑yaw (ZYX):** three‑angle descriptions of orientation; each has a singular configuration.
- **Skew‑symmetric matrix:** a matrix equal to the negative of its transpose; `skew(v)` encodes a cross product with `v`.
- **Twist / spatial velocity:** a 6‑vector of linear and angular velocity.
- **Adjoint (`Ad`):** the 6×6 matrix that moves a twist between frames; its transpose moves a wrench.
- **Wrench:** a 6‑vector of force and torque.
- **Quintic polynomial (`tpoly`):** 5th‑order curve giving smooth position, velocity and acceleration.
- **LSPB (`lspb`):** linear segment with parabolic blends, the trapezoidal velocity profile.
- **Via point:** an intermediate waypoint a trajectory passes through (`mstraj`).
- **Odometry:** distance and heading change measured by the wheels, with noise.
- **Covariance matrix:** a matrix of variances (diagonal) and correlations (off‑diagonal) describing uncertainty.
- **Handle class:** a MATLAB object passed by reference; one shared copy.
- **Occupancy grid:** a matrix map with 0 for free cells and 1 for obstacles.
- **Cost map / distance map:** per‑cell cost or distance to the goal used by `Dstar` and `DXform`.
- **Roadmap (PRM):** a graph of random free points connected by collision‑free edges.
- **Kinodynamic planner (RRT, Lattice):** one that respects the vehicle's motion constraints.
- **Range‑bearing sensor:** a sensor returning distance and angle to a landmark.
- **Extended Kalman filter (EKF):** an estimator for nonlinear systems that linearises with Jacobians at every step.
- **Jacobian:** a matrix of partial derivatives (`Fx`, `Fv`, `Hx`, `Hw`, `Gx`, `Gz`, `jacob0`).
- **Pose graph:** nodes are robot poses, edges are relative‑pose constraints; `optimize` makes them consistent.
- **ICP:** iterated closest point, aligning two point clouds to find the motion between them (Machine Vision Toolbox).
- **Denavit–Hartenberg parameters:** four numbers per joint that describe a robot arm's geometry.
- **Forward / inverse kinematics (`fkine` / `ikine`):** joint angles to pose, and pose to joint angles.

## Check yourself

1. You type `R = rot2(45)` and the frame you plot looks nothing like a 45° turn. What went wrong, and what are two ways to fix it?
2. `T1 = transl2(1,2)*trot2(30,'deg')` and `T2 = transl2(2,1)`. Without MATLAB, what is the translation part of `T1*T2`, and why is it not (3, 3)?
3. Which `EKF(...)` call would you write to estimate landmark positions when the vehicle's pose is known exactly, and what does each `[]` mean?
4. You created `ds = Dstar(house)` and immediately called `ds.query(place.br3)`. It fails. Why, and what is the correct sequence? Does the same apply to `Bug2`?
5. The Lecture 10 script calls `icp(...)` and `pg.scanmap()`, but you cannot find either in this manual. Where do they come from?

<details>
<summary>Answers</summary>

1. The toolbox assumes **radians**, so you asked for a rotation of 45 radians (about 7.2 full turns). Fix it with `rot2(45, 'deg')` or `rot2(pi/4)`; the same applies to `trot2`, `rotx`, `SE3.Rz`, `UnitQuaternion.Rz` and friends.

2. The translation is `t1 + R1*t2`. With `R1` the 30° rotation, `R1*(2,1) = (0.866·2 − 0.5·1, 0.5·2 + 0.866·1) = (1.232, 1.866)`, so the result is `(1 + 1.232, 2 + 1.866) = (2.232, 3.866)`. It is not (3, 3) because the second displacement is applied in the *rotated* frame {1}, not in the world frame. Swapping the order, `T2*T1`, does give (3, 3) because `T2` has no rotation.

3. `ekf = EKF(veh, [], [], sensor, W_est, [])`. The first `[]` (in place of `V_est`) and second `[]` (in place of `P0`) tell the filter the vehicle is error‑free, so it does not estimate the pose. The last `[]` (in place of `map`) says the map is unknown, so landmark positions are added to the state vector and estimated as they are observed.

4. `Dstar` needs a cost map built for a goal before it can walk downhill from a start, so you must call `ds.plan(place.kitchen)` first (optionally with `'animate'`), then `ds.query(place.br3)`. The same `plan` then `query` order holds for `DXform`, `PRM`, `Lattice` and `RRT`. `Bug2` is the exception: it is purely reactive, has no `plan` method, and you call `bug.query(start, goal)` directly. After `ds.modify_cost(...)` you must call `ds.plan` again before querying.

5. `icp` belongs to Peter Corke's **Machine Vision Toolbox**, not the Robotics Toolbox, so it is not in this manual and needs that toolbox installed. `scanmap`, along with `plot`, `optimize`, `scan`, `scanxy`, `plot_occgrid` and `time`, are methods of `PoseGraph` whose entries are missing from this PDF (it documents only the constructor). Use `help PoseGraph` in MATLAB, where the help text is read from the current source code.

</details>
