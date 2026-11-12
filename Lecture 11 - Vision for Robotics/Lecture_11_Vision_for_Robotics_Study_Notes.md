# Lecture 11 – Vision for Robotics (Introduction to Machine Vision)

*Study notes written from the 2024 slide deck by Christoforos Kanellakis (58 slides) and the one‑page "Notes on Lecture 11" reading guide.*

---

## What this lecture is about, and why it matters

Up to now the robot has mostly "felt" its way around with wheel encoders, LiDAR and inertial sensors. This lecture gives the robot eyes. You will see how a camera turns light into a grid of numbers, how we describe the geometry of that process with a few simple equations, how we pull useful information out of the numbers (colour, edges, regions, and even "is there a person here?"), and finally how a robot can close a control loop using what the camera sees. This is called **visual servoing**, and it is the bridge between everything you have learnt about poses and control and the world of cameras.

### 📖 Reading map (from the lecturer's one‑page note)

The lecturer says this lecture maps onto these sections of Peter Corke's *Robotics, Vision and Control*:

| Topic in this lecture | Corke book sections |
|---|---|
| Light, colour, colour spaces | Chapter 10/11 → **11.1, 11.2** (image formation: perspective camera, lens, camera matrix) |
| Image processing (thresholding, spatial operators, segmentation) | **12.1, 12.2, 12.3, 12.4, 12.5** |
| Image feature extraction (regions, lines, point features) | **13.1, 13.2, 13.3** |
| Multiple views / stereo / 3‑D | **14.1, 14.2, 14.3, 14.4, 14.5** and **14.7.4** |

⚠️ The note lists section numbers but not the book edition. Section numbering shifts slightly between the 1st (2011) and 2nd (2017) editions. If a section does not match the topic above, check the other edition or ask the lecturer.

---

## Roadmap: the main topics in this lecture

1. Learning objectives and a reminder of what a robot is.
2. Why robots, and where they are used (sectors).
3. What machine vision is, and why vision in robotics is hard (the challenge list).
4. Image formation: from light source to digital camera to numbers.
5. Colour sensing: the Bayer grid and demosaicing.
6. What an image *is* mathematically: a function and an array.
7. Typical vision sensors used for visual servoing.
8. Colour spaces: RGB versus HSV (and the others on the list).
9. Camera geometry: pinhole camera, thin lens, perspective projection, the camera matrix, pixel coordinates.
10. Depth from stereo.
11. Image segmentation and thresholding.
12. Spatial operations: correlation, convolution, kernels (blur, sharpen, Laplacian, Sobel).
13. A full pipeline example: human detection with sliding windows, image pyramids, HOG and SVM.
14. Vision in control: visual servoing, camera placement, PBVS versus IBVS.

Let's take them in order.

---

## 1. Learning objectives and "what is a robot?"

The lecturer sets four goals for you (slide 2):

1. Broaden your horizons on what robots are used for.
2. Make the connection between robots and **visual sensors** (cameras) and their properties.
3. Get a high‑level description of what vision can do for a robot.
4. Walk through one real example: **person detection**.

Then comes a reminder you have seen in Lecture 1. A **robot** is "a goal‑oriented machine that can sense, plan and act automatically" (slide 3). Keep those three verbs in your head: **sense → plan → act**. Vision lives in the *sense* part, but, as you will see at the end, it can reach straight into the *act* part too.

---

## 2. Why robots, and where are they used?

Slide 4 (partly hidden behind an embedded video, but fully readable in the text layer) lists why industry likes robots:

- High precision, and they repeat tasks with high accuracy.
- Increased load capabilities (they lift more than you).
- Multi‑tasking, and not affected by the monotony of tasks.
- Robots can collaborate with each other and, to some extent, with human workers.
- They let humans avoid hazardous areas.
- Many can be re‑programmed for different tasks.
- Business outcomes: increased productivity, increased efficiency, cut production costs, safer work environment.

Slide 5 lists the industries "booming" because of robots: **manufacturing, agriculture, food preparation, search and rescue, pharmaceutical industry, maintenance/inspection/repair, mining and construction**. Slides 6–9 show pictures: collaborative arms and car assembly lines (manufacturing), a strawberry‑picking arm and a field weeding robot (agriculture), a pizza‑making arm (food), and bomb‑disposal tracked robots, a firefighting robot and an ambulance drone (search and rescue: "delivery of time‑critical services").

An exam question here would be a list question, so make sure you can name at least five sectors and five benefits.

---

## 3. What is machine vision, and why is it hard?

**Machine vision** (slide 10) is "the automatic extraction of *meaningful* information from images and videos." The word *meaningful* is doing the work. A camera gives you millions of numbers. Vision is about turning them into something a robot can use: "there is a pedestrian 4 m ahead", "the apple is red and ripe", "the weld seam is here".

Slide 11 shows three "focus on vision" examples that return at the end of the lecture: a field robot navigating crop rows by camera, a humanoid tracking an object with an **ArUco marker** (a black‑and‑white square tag that is easy for a camera to find and gives you its pose), and an underwater vehicle following a pipeline with an onboard camera.

### The challenge list (slide 12)

You should be able to explain each of these in one sentence:

| Challenge | What it means in plain words |
|---|---|
| **Sensor frame rate** | A camera gives you, say, 30 pictures per second. If the robot moves fast, that may be too slow to react in time. |
| **Image/depth processing** | Processing millions of pixels (and depth maps) takes a lot of computing power and time. |
| **Occlusion** | Something gets in the way and hides the object you were tracking. |
| **Viewpoint handling** | The same object looks completely different from the front, the side or above. |
| **Out‑of‑view events** | The target simply leaves the picture, and you have nothing to look at. |
| **Clutter** | A busy background full of other objects makes it hard to find yours. |
| **Camera positioning** | Where you mount the camera changes what it can see and how useful it is (we return to this with eye‑in‑hand vs eye‑to‑hand). |
| **Field of view** | The camera only sees a wedge of the world. A wide wedge sees more but each thing looks smaller. |

**Everyday analogy.** Imagine finding a friend in a crowded train station through a camera phone: the phone only shows a small window (field of view), people walk in front of your friend (occlusion), your friend might step out of the frame (out‑of‑view), there are hundreds of other people (clutter), and if you are running the video lags (frame rate, processing). Robots have exactly these problems.

---

## 4. Image formation: from light to numbers

### 4.1 The physical picture (slides 13, 14, 26)

Light leaves a **light source**, hits a **surface**, bounces off in many directions (the little arrows around the surface normal $\hat{n}$ in the figure), and some of that light travels through the **optics** (the lens) to form an **image plane**, which is physically captured by the **sensor plane**. Slide 14 adds the last step: the picture on the internal image plane is sampled into a grid of little squares, the **output (digitised) image**.

So the chain is: *illumination → scene → imaging system → internal image plane → digitised image*.

### 4.2 The digital camera pipeline (slide 15)

The block diagram shows three boxes:

1. **Camera body**: *Optics → Aperture → Shutter*. The lens gathers light, the aperture is the adjustable hole that controls how much light gets in, and the shutter controls for how long.
2. **Sensor chip**: *Sensor (CCD/CMOS) → Gain (ISO) → ADC*. The key sentence on the slide is **"the sensor chip converts photons into electrons."** Each tiny light‑sensitive element collects electric charge in proportion to how many photons landed on it. **Gain** (what photographers call ISO) amplifies that signal, and the **ADC** (analogue‑to‑digital converter) turns the voltage into an integer. The output here is the **RAW** image.
3. **Image signal processor (ISP)**: *Demosaic → Denoise and sharpen → White balance → Gamma/curve → Compress*, producing the familiar **JPEG**.

**Analogy.** Think of each sensor element as a bucket in a rainstorm. Photons are raindrops. After the shutter closes you measure how full each bucket is (the ADC) and write down a number. Lots of rain = bright pixel.

**Common confusion.** The RAW image is *not* yet a colour picture you would recognise. Colour and good looks come from the ISP steps. This matters for robotics because sometimes you want RAW‑like data (more faithful) and sometimes you want the processed image (nicer for humans but with hidden manipulation).

### 4.3 Colour sensing: the Bayer grid (slide 16)

A sensor element only counts photons; it does not know their colour. To get colour, a mosaic of tiny coloured filters is placed over the sensor: the **Bayer grid** (or Bayer filter). In each 2×2 group there is one red, one blue and **two green** filters (the figure shows the checkerboard with green dominating). So each pixel records only *one* of the three colours.

**Demosaicing** is "used to reconstruct a full colour image from the incomplete colour samples." The algorithm looks at a pixel's neighbours to *estimate* the two colours that pixel did not measure. The right‑hand figure shows the process: Bayer pattern → separate sparse red, green, blue planes → interpolated full planes → full RGB image.

**Analogy.** It is like a survey where each household was asked only one of three questions. To fill in the missing answers for a house you look at what its neighbours answered.

**Why two greens?** The slide does not say, so here is a short piece of background: the human eye is most sensitive to green, so sampling green more densely gives a sharper‑looking image. ⚠️ This reason is not on the slides; it is standard background knowledge.

### 4.4 What is an image, mathematically? (slide 17)

An image can be treated as a **function** $f(x, y)$ from $\mathbb{R}^2$ to $\mathbb{R}$. In words: you put in a location $(x, y)$ and the function gives back the **intensity** (brightness) at that spot. The image has finite boundaries over a rectangle.

For a computer the image is an **array** (a matrix) with $M$ rows and $N$ columns:

$$
f(x,y) = \begin{bmatrix}
f(0,0) & f(0,1) & \cdots & f(0,N-1) \\
f(1,0) & f(1,1) & \cdots & f(1,N-1) \\
\vdots & \vdots & & \vdots \\
f(M-1,0) & f(M-1,1) & \cdots & f(M-1,N-1)
\end{bmatrix}
$$

So a greyscale image of 480 rows by 640 columns is just a 480×640 table of numbers, usually integers from 0 (black) to 255 (white). A colour image is three such tables stacked (slide 20's figure shows the red, green and blue "planes" with $I(r, c, 1)$, $I(r, c, 2)$, $I(r, c, 3)$).

**Common confusion.** Many people mix up $(x, y)$ and (row, column). In the matrix, the *first* index is the row (vertical, like $y$) and the *second* is the column (horizontal, like $x$). Toolboxes differ in their convention, so always check.

### 4.5 Why interpreting images is hard (slide 18)

Slide 18 shows a road photo with a small green circle around a region, and next to it the actual numbers in that region: a big table of values like 239, 255, 0, 192… To you the photo obviously shows a toll station, but the computer only sees that table. Nothing in the numbers says "car" or "road". Every piece of meaning has to be computed from patterns in the numbers. That is the whole difficulty of vision in one picture.

### 4.6 Typical sensors for visual servoing (slide 19)

The slide shows four devices, without labels. From the pictures they are: a small board‑level monocular camera module, a **ZED** stereo camera (two lenses, long bar), another stereo/depth camera board with two lenses, and an **Intel RealSense**‑style depth camera (blue bar). The message is that in practice you choose between a single camera (cheap, no depth directly), a **stereo pair** (two cameras, depth by triangulation, covered in section 10), or an active depth camera.

⚠️ The slide only shows pictures with no names or specifications; the identifications above come from recognising the products, so do not quote them as lecture content.

---

## 5. Colour spaces

A **colour space** is just a way of writing a colour as a set of numbers. Slide 20 contrasts an **RGB** image (3 channels) with a **greyscale** image (1 channel) and lists other spaces: **CMY, YIQ, HSI, HSV, HLS, LAB, HMMD**. You only need to understand RGB and HSV in depth; be able to name a few of the others.

### 5.1 The RGB colour model (slides 21–22)

RGB is "based on a Cartesian coordinate system": three axes, Red, Green, Blue, each running from 0 to 1 (or 0 to 255). This gives a **cube**:

- The **primary** colours R, G, B sit at three corners (one unit along each axis).
- The **secondary** colours **cyan, magenta, yellow** sit at the three other corners (C = G + B, M = R + B, Y = R + G).
- **Black** (S in the figure) is at the origin (0, 0, 0).
- **White** (W) is at the corner farthest from the origin (1, 1, 1).
- The dashed diagonal from black to white is the line of greys.

Slide 22 shows a fruit basket split into its R, G and B planes. Notice the problem: the red apples are bright in the R plane *and* fairly bright in G and B, because they are also quite light. The slide's key sentence: **"Since any coordinate movement modifies colour tone, saturation and brightness all at once, colour selection in RGB space is difficult and quite non‑intuitive."**

**Analogy.** Describing "a slightly darker, less vivid red" in RGB means changing all three numbers in a way that is not obvious. It is like asking someone to make a soup "a bit less salty" by changing the amount of water, stock *and* vegetables at the same time.

### 5.2 The HSV colour space (slides 23–25)

HSV describes colour the way people think about it, with three separate knobs:

- **Hue (H)** is "the colour's position on a colour wheel": is it red, yellow, green, cyan, blue, magenta? It is an angle, 0° to 360°, around the cylinder in the figure.
- **Saturation (S)** is "the amount of hue, or departure from neutral": how vivid versus how grey/washed‑out. It is the distance from the central axis.
- **Value (V)** is "the maximum value among the red, green and blue components": roughly how bright. It is the height along the axis.

Slide 24 shows the same fruit basket as H, S and V planes. In the **H** plane all the apples have the same value regardless of shading, which is exactly what you want when you tell a robot "pick the red things". Slide 25 shows the RGB cube and the HSV cylinder side by side with a double arrow: you can convert back and forth.

The slide's conclusion: **"Colour selection is preferred in other than RGB colour spaces (e.g. HSV), because perceptual colour features such as saturation are represented individually and can be modified independently."**

**Exam tip.** The one‑liner to remember: *RGB is good for displays and sensors; HSV is good for picking out colours, because hue is separated from brightness.*

**Common mistake.** People say "value is brightness" and stop there. Be precise for the exam: the slide defines value as the *maximum* of the R, G, B components.

---

## 6. Camera geometry: how 3‑D becomes 2‑D

This is the mathematical core of the lecture. We will build it in four small steps.

### 6.1 Step 1: the pinhole camera (slides 27–28)

The simplest camera is a box with a tiny hole (**pinhole**) in one side and the image plane on the opposite side. Each point in the scene sends exactly one ray through the hole, so the image is sharp (but very dim, since so little light gets in). Because rays cross at the hole, the image is **inverted** (the candle appears upside down). Slide 27 also shows a **virtual image**: if you imagine an image plane *in front* of the pinhole at the same distance, you get the same picture the right way up. We use that virtual plane in the equations, which is why there is no minus sign.

### 6.2 Step 1b: the thin lens (slide 28)

A real camera uses a **lens** to collect much more light than a pinhole yet still focus it. An **ideal thin lens** has two **focal points** at distance $f$ on each side. Rays parallel to the axis bend through the far focal point; rays through the lens centre go straight. Where they meet, you get a sharp, inverted image. The slide's point is that the thin lens behaves like an **equivalent pinhole** at the lens centre, so the pinhole equations below still apply.

**Analogy.** A pinhole is like looking at the world through a tiny keyhole: sharp but dark. A lens is like a funnel that gathers a whole bucket of light and pours it onto the same spot.

### 6.3 Step 2: perspective projection (slide 29)

Set up the **camera frame** $\{C\}$ with its origin at the pinhole (the *camera origin*) and its $z_C$ axis pointing out along the **optical axis**. Put the image plane at $z = f$ in front of the origin (the virtual image). The optical axis hits the image plane at the **principal point**.

A world point has **world coordinates** $\mathbf{P} = (X, Y, Z)$ (expressed in the camera frame). It projects to **image coordinates** $\mathbf{p} = (x, y)$ on the image plane via **perspective projection**:

$$
x = f\,\frac{X}{Z}, \qquad y = f\,\frac{Y}{Z}
$$

In words: *the image position is the world position divided by the depth, scaled by the focal length.*

What each symbol means:

- $f$: **focal length**, the distance from the pinhole to the image plane (in metres or millimetres).
- $X, Y$: how far the point is sideways and up/down from the optical axis.
- $Z$: how far away the point is along the optical axis (its **depth**).

Two things fall straight out of this formula, and both are exam favourites:

1. **Far things look small.** Double $Z$ and the image coordinates halve. That is perspective.
2. **You lose depth.** Every point on the ray through $\mathbf{P}$ and the pinhole gives the *same* $(x, y)$. One camera cannot tell you $Z$. This is why we need stereo later.

**Worked example.** Let $f = 8$ mm and a point $\mathbf{P} = (0.2\ \text{m},\ 0.1\ \text{m},\ 2\ \text{m})$.

- $x = 8 \times 0.2 / 2 = 0.8$ mm.
- $y = 8 \times 0.1 / 2 = 0.4$ mm.

So the point lands 0.8 mm right and 0.4 mm below the principal point on the sensor (the $y$ axis points down in the figure). If the same point were 4 m away instead of 2 m, you would get 0.4 mm and 0.2 mm: half the size.

### 6.4 Step 3: homogeneous coordinates and the camera matrix (slide 30)

The division by $Z$ makes the projection **non‑linear**, which is awkward. The trick from Lecture 1 saves us: **homogeneous coordinates**. Write the world point with an extra 1,

$$
{}^{C}\tilde{\mathbf{P}} = (X, Y, Z, 1)^T,
$$

and let the image point also have an extra coordinate, $\tilde{\mathbf{p}} = (\tilde{x}, \tilde{y}, \tilde{z})$. Then define

$$
\tilde{x} = fX, \qquad \tilde{y} = fY, \qquad \tilde{z} = Z,
$$

which is now **linear**, so it can be written as a matrix product:

$$
\tilde{\mathbf{p}} = \begin{pmatrix} f & 0 & 0 & 0 \\ 0 & f & 0 & 0 \\ 0 & 0 & 1 & 0 \end{pmatrix} {}^{C}\tilde{\mathbf{P}}
$$

That 3×4 matrix is the **camera matrix** $\mathbf{C}$. To get back ordinary (**non‑homogeneous**) image coordinates you divide by the third element:

$$
x = \frac{\tilde{x}}{\tilde{z}}, \qquad y = \frac{\tilde{y}}{\tilde{z}}
$$

Check that this reproduces the original formula: $\tilde{x}/\tilde{z} = fX/Z$. Yes.

**Why bother?** Because now a whole chain of operations (move the camera, project, convert to pixels) is just a product of matrices, exactly like the pose compounding $\oplus$ you learnt in Lecture 1.

**Worked example (continued).** With $f = 8$ mm and $\mathbf{P} = (0.2, 0.1, 2)$ m (convert $f$ to metres: 0.008):

$$
\tilde{\mathbf{p}} = (0.008 \times 0.2,\ 0.008 \times 0.1,\ 2) = (0.0016,\ 0.0008,\ 2).
$$

Dividing by $\tilde{z} = 2$: $x = 0.0008$ m $= 0.8$ mm, $y = 0.0004$ m $= 0.4$ mm. Same answer as before.

### 6.5 Step 4: the discrete image plane, pixel coordinates (slide 31)

Real sensors are grids of pixels, $W$ wide and $H$ high. The pixel coordinate system $(u, v)$ has its origin in the **top‑left corner**, $u$ running right, $v$ running down. Each pixel has width $\rho_w$ and height $\rho_h$ (metres per pixel). The principal point sits at pixel $(u_0, v_0)$.

Converting from metres on the image plane to pixels:

$$
u = \frac{x}{\rho_w} + u_0, \qquad v = \frac{y}{\rho_h} + v_0
$$

In words: *divide the metric position by the pixel size to count pixels, then shift so that the principal point is at $(u_0, v_0)$.*

In homogeneous form, with $\tilde{\mathbf{p}} = (\tilde{u}, \tilde{v}, \tilde{w})$:

$$
\tilde{\mathbf{p}} =
\underbrace{\begin{pmatrix} 1/\rho_w & 0 & u_0 \\ 0 & 1/\rho_h & v_0 \\ 0 & 0 & 1 \end{pmatrix}}_{\mathbf{K}}
\begin{pmatrix} f & 0 & 0 & 0 \\ 0 & f & 0 & 0 \\ 0 & 0 & 1 & 0 \end{pmatrix}
{}^{C}\tilde{\mathbf{P}},
\qquad
u = \frac{\tilde{u}}{\tilde{w}},\ v = \frac{\tilde{v}}{\tilde{w}}
$$

The 3×3 matrix $\mathbf{K}$ holds the **intrinsic parameters** of the camera (pixel size and principal point). Together with $f$ these are what **camera calibration** finds (that word comes back in the stereo section).

**Worked example (finished).** Take pixels of $\rho_w = \rho_h = 10\ \mu\text{m} = 0.01$ mm and a 1280×1024 sensor with principal point $(u_0, v_0) = (640, 512)$. From before, $x = 0.8$ mm and $y = 0.4$ mm:

- $u = 0.8 / 0.01 + 640 = 80 + 640 = 720$.
- $v = 0.4 / 0.01 + 512 = 40 + 512 = 552$.

So the 3‑D point appears at pixel (720, 552), a bit right of and below the centre of the picture.

**Common mistakes.**
- Forgetting to divide by $\tilde{w}$ (or $\tilde{z}$) at the end. The homogeneous vector is *not* the answer; you must normalise.
- Mixing units: $f$ in mm but $\rho$ in µm. Convert everything to the same unit first.
- Thinking $(u_0, v_0)$ is always the exact centre of the image. It is *close* to the centre but calibration finds the real value.

---

## 7. Depth from stereo (slides 32–33)

One camera loses depth. Two cameras, like your two eyes, get it back.

### 7.1 The set‑up

Slide 32 assumes the simplest arrangement: **"both cameras are identical (same focal length) and are aligned with the x‑axis."** The two camera centres $O_L$ and $O_R$ are separated by the **baseline** $b$. A world point $\mathbf{p}$ projects to $\mathbf{p}_L$ in the left image and $\mathbf{p}_R$ in the right image. Because the cameras are side by side, $\mathbf{p}_L$ and $\mathbf{p}_R$ lie on the same horizontal line, called an **epipolar line**. That means to find the match for a left‑image pixel you only have to search along one row of the right image, not the whole picture.

### 7.2 The four‑step overview

1. **Camera calibration**: find each camera's intrinsics ($f$, $\rho$, $u_0$, $v_0$) and the relative pose between the two cameras.
2. **Image rectification**: warp the two images so that they are exactly row‑aligned (so epipolar lines really are horizontal rows).
3. **Stereo matching**: for each pixel in the left image, find the corresponding pixel in the right image along the same row.
4. **Disparity → triangulation**: the horizontal shift between the two matches is the **disparity** $d = u_L - u_R$ (in pixels). Near things shift a lot, far things shift little. Depth follows by triangulation.

### 7.3 The depth formula

⚠️ The slide gives the geometry but not the formula. The standard result (Corke chapter 14) for the parallel set‑up is:

$$
Z = \frac{f\, b}{d}
$$

where $Z$ is depth, $f$ is focal length *in pixels* (that is, $f/\rho_w$), $b$ is the baseline, and $d$ is the disparity in pixels. In words: *depth is inversely proportional to disparity.* Hold a finger near your face and blink left/right eye: it jumps a lot (big disparity, small $Z$). Look at a far tree: it barely moves.

**Worked example.** A stereo camera has $f = 800$ pixels and baseline $b = 0.12$ m. A pixel is found at $u_L = 650$ in the left image and $u_R = 610$ in the right. Then $d = 40$ pixels and

$$
Z = \frac{800 \times 0.12}{40} = 2.4\ \text{m}.
$$

If the disparity had been only 10 pixels, $Z$ would be 9.6 m. Notice how a 1‑pixel matching error matters far more for distant points (10 → 11 pixels changes $Z$ from 9.6 m to 8.7 m, while 40 → 41 changes 2.4 m to 2.34 m). Stereo gets worse with distance.

### 7.4 Problems with stereo (slide 32)

- **Camera calibration errors**: wrong $f$ or baseline give wrong $Z$ everywhere.
- **Poor image resolution**: coarse pixels mean coarse disparity steps, so coarse depth.
- **Occlusions**: a point seen by the left camera may be hidden from the right one, so there is nothing to match.
- **Violation of brightness consistency**: matching assumes the same point looks equally bright in both images; shiny surfaces and different exposures break this.
- **Low‑contrast image regions**: a blank white wall gives you nothing to match; every pixel looks like every other.

Slide 33 shows the outcome: a street scene and its **depth map**, where colour encodes distance (red near, blue far).

---

## 8. Image segmentation and thresholding (slides 34–35)

### 8.1 Segmentation

**Segmentation** "partitions an image into application‑meaningful regions." The aim is "to separate pixels that represent objects/areas of interest from all other pixels in the scene." Applications listed: medical imaging, traffic control systems, character recognition, autonomous robots (e.g. self‑driving cars), and surveillance in satellite images. The slide's example is an aerial photo of a forest/field, segmented into two classes (forest in purple, open ground in yellow).

**Analogy.** Segmentation is colouring in a map: each pixel gets a label saying which region it belongs to.

### 8.2 Thresholding: the simplest segmentation

**Thresholding** "is the simplest segmentation method: pixels are partitioned based on their intensity value."

**Global thresholding** uses one threshold $T$ for the whole image:

$$
g(x,y) = \begin{cases} 1 & \text{if } f(x,y) > T \\ 0 & \text{if } f(x,y) \le T \end{cases}
$$

In words: brighter than $T$ becomes white (object), otherwise black (background), giving a **binary image**.

How do you choose $T$? Look at the **histogram** of the image (the bar chart on the slide counting how many pixels have each intensity). If the image has a bright object on a dark background, the histogram has **two humps** (it is **bimodal**). Put $T$ in the valley between them. The slide's histogram has humps near 60 and 120 with a red line for $T$ at about 90.

**Worked example.** Suppose a tiny 3×3 greyscale image is

$$
f = \begin{bmatrix} 40 & 55 & 60 \\ 50 & 130 & 125 \\ 45 & 120 & 135 \end{bmatrix}, \qquad T = 90.
$$

Compare each pixel with 90:

$$
g = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 1 & 1 \\ 0 & 1 & 1 \end{bmatrix}
$$

The bright 2×2 block in the bottom right has been segmented out as "object".

**Variants** (slide 35):

- **Variable thresholding**: $T$ changes over the image.
  - **Local / regional**: $T$ depends on the *neighbourhood* of $(x, y)$ (e.g. the mean of the surrounding pixels), useful when lighting is uneven.
  - **Adaptive**: $T$ is a function of $(x, y)$ itself.
- **Multiple thresholding**: more than one threshold to get more than two classes, e.g. with $T_1 < T_2$,

$$
g(x,y) = \begin{cases} a & \text{if } f(x,y) > T_2 \\ b & \text{if } T_1 < f(x,y) \le T_2 \\ c & \text{if } f(x,y) \le T_1 \end{cases}
$$

The right‑hand histograms on the slide show the two cases: one valley → one $T$; two valleys → $T_1$ and $T_2$.

⚠️ On slide 35 the formulas are partly hidden behind the pasted images; only the fragments "$g(x,y) = \ldots$ if $T_1 < f(x,y) \le T_2$" and "$c$ if $f(x,y) \le T_1$" are visible. The full forms above are the standard ones (Gonzalez & Woods style) and match the visible fragments, but check them against the lecturer's own version.

**Common mistake.** Thresholding only works on *intensity*. If your object has the same brightness as the background but a different colour, threshold the **hue** channel of HSV instead (this is where section 5 pays off).

---

## 9. Spatial operations on images (slides 36–37)

A **spatial operation** computes each output pixel from a small **window** $\mathcal{W}$ of input pixels around the same location. Slide 36 shows it: a window of half‑width $h$ centred at $(u, v)$ in the input image $\mathbf{I}$, a function $f(\mathcal{W})$, and a single output pixel $\mathbf{O}[u, v]$. Slide the window over every pixel and you have the output image.

The most important spatial operations are **linear** ones, where $f$ is a weighted sum. The table of weights is called the **kernel** (or mask or filter) $\mathbf{K}$.

### 9.1 Correlation versus convolution

**Correlation** ($\otimes$):

$$
\mathbf{O} = \mathbf{K} \otimes \mathbf{I}, \qquad \mathbf{O}[u,v] = \sum_{(i,j)\in\mathcal{W}} \mathbf{I}[u+i,\ v+j]\ \mathbf{K}[i,j]
$$

**Convolution** ($*$):

$$
\mathbf{O} = \mathbf{K} * \mathbf{I}, \qquad \mathbf{O}[u,v] = \sum_{(i,j)\in\mathcal{W}} \mathbf{I}[u-i,\ v-j]\ \mathbf{K}[i,j]
$$

Look closely: the only difference is **$+$ versus $-$** inside the image index. In words: correlation lays the kernel directly over the image and multiplies‑and‑adds; convolution first **flips the kernel 180°** (both left‑right and up‑down) and then does the same multiply‑and‑add.

**Why does it matter?** For a **symmetric** kernel (blur, Laplacian) flipping changes nothing, so correlation and convolution give identical results. For an **asymmetric** kernel (Sobel) the results have opposite sign. Convolution is preferred in theory because it is **commutative and associative** ($\mathbf{K}_1 * (\mathbf{K}_2 * \mathbf{I}) = (\mathbf{K}_1 * \mathbf{K}_2) * \mathbf{I}$), which lets you combine filters. Many software libraries actually compute correlation and call it convolution, so always check.

**Analogy.** Correlation is a stencil you press onto the page as it is. Convolution is the same stencil turned upside down first.

### 9.2 Tiny worked example (3×3)

Take an input patch with a dark top and bright bottom (a horizontal edge):

$$
\mathbf{I} = \begin{bmatrix} 10 & 10 & 10 \\ 10 & 10 & 10 \\ 50 & 50 & 50 \end{bmatrix}
$$

and the **Sobel horizontal** kernel from slide 37:

$$
\mathbf{K} = \begin{bmatrix} -1 & -2 & -1 \\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{bmatrix}
$$

**Correlation** at the centre pixel: multiply element by element and add.

- Top row: $(-1)(10) + (-2)(10) + (-1)(10) = -40$.
- Middle row: all zeros → $0$.
- Bottom row: $(1)(50) + (2)(50) + (1)(50) = 200$.
- Total: $-40 + 0 + 200 = \mathbf{160}$.

**Convolution** at the centre pixel: flip the kernel first,

$$
\mathbf{K}_{\text{flipped}} = \begin{bmatrix} 1 & 2 & 1 \\ 0 & 0 & 0 \\ -1 & -2 & -1 \end{bmatrix},
$$

then multiply‑and‑add: $40 + 0 - 200 = \mathbf{-160}$.

Same magnitude (a strong edge is there), opposite sign (which way is "up"). On a flat patch of all 10s, both give $-40 + 40 = 0$: no edge, no response. That is exactly what an edge detector should do.

### 9.3 The kernel zoo (slide 37)

| Name | Kernel | What it does |
|---|---|---|
| **Identity** | $\begin{bmatrix}0&0&0\\0&1&0\\0&0&0\end{bmatrix}$ | Output = input. A sanity check. |
| **Sharpen** | $\begin{bmatrix}0&-1&0\\-1&5&-1\\0&-1&0\end{bmatrix}$ | Boosts the centre relative to its neighbours, so edges look crisper. Weights sum to 1, so brightness is preserved. |
| **Mean blur** | $\frac{1}{9}\begin{bmatrix}1&1&1\\1&1&1\\1&1&1\end{bmatrix}$ | Replaces each pixel by the average of its 3×3 neighbourhood; smooths noise, blurs detail. |
| **Gaussian blur** | $\frac{1}{16}\begin{bmatrix}1&2&1\\2&4&2\\1&2&1\end{bmatrix}$ | A blur that weights the centre more; smoother and more natural than mean blur. |
| **Laplacian** | $\begin{bmatrix}0&1&0\\1&-4&1\\0&1&0\end{bmatrix}$ | Second derivative; responds at edges in *all* directions, zero on flat regions (weights sum to 0). |
| **Sobel horizontal** | $\begin{bmatrix}-1&-2&-1\\0&0&0\\1&2&1\end{bmatrix}$ | Measures change in the **vertical** direction, so it highlights **horizontal edges** (as the astronaut image on the slide shows). Its transpose is the Sobel vertical kernel. |

**Rule of thumb.** If the weights sum to **1**, the filter preserves average brightness (blurs, sharpen). If they sum to **0**, the filter is a derivative‑type detector that outputs zero on flat areas (Laplacian, Sobel).

**Common confusion.** "Sobel horizontal" is named after the **edges** it finds (horizontal ones), which it does by taking the derivative in the **vertical** direction. Different sources name it the other way round, so when in doubt describe what it does rather than relying on the name.

---

## 10. A full pipeline: human detection with HOG + SVM (slides 38–45)

This is the lecture's worked example of how simple building blocks combine into something useful: deciding whether a person is in an image.

### 10.1 The sliding window method and image pyramids (slide 38)

You do not know where the person is or how big they are. Two tricks:

1. **Sliding window.** Take a fixed‑size **detection window** (64×128 pixels) and slide it across the whole image, step by step. At every position ask a classifier "person or not?" This handles *where*.
2. **Image pyramid.** Make copies of the image at several scales: **fine** ($l = 0$, the original), **medium** ($l = 1$, shrunk), **coarse** ($l = 2$, shrunk more). Slide the *same* 64×128 window over each level. A person who is too big for the window in the original becomes window‑sized in a shrunk copy. This handles *how big*.

The block diagram on slide 38 reads: *Image inputs → Pre‑processing (image level) → collect 3780 HOG features in a single detection window → SVM classification (window level) → Person?*

**Analogy.** It is like checking a huge group photo for your friend with a small cardboard frame: you slide the frame over every spot, and you also try it on smaller prints of the photo in case your friend is standing close to the camera and would not fit in the frame on the big print.

### 10.2 HOG: Histogram of Oriented Gradients (slides 39–44)

HOG "creates a vector with histograms of directions of gradients." The idea: what makes a person recognisable is the *shape* of their outline (head, shoulders, legs), and shape shows up as **edge directions**. HOG summarises, for small patches of the image, which edge directions are common. Here is each step and where every number comes from.

**Step 1: create a patch of 64×128.** Crop the candidate window (e.g. a 100×200 region around a runner in a 720×475 photo) and resize it to exactly **64 pixels wide by 128 pixels high**. The 1:2 aspect ratio suits a standing person.

**Step 2: calculate image gradients.** For each pixel compute how fast brightness changes horizontally and vertically (you can use Sobel kernels from section 9). The **gradient** is

$$
\nabla f = \left[\frac{\partial f}{\partial x},\ \frac{\partial f}{\partial y}\right] = [g_x,\ g_y]
$$

From it get the **gradient magnitude** (how strong the edge is)

$$
M(x,y) = \|\nabla f(x,y)\| = \sqrt{g_x^2(x,y) + g_y^2(x,y)}
$$

and the **gradient direction** (which way the edge points)

$$
\alpha(x,y) = \tan^{-1}\!\left[\frac{g_y(x,y)}{g_x(x,y)}\right]
$$

Slide 40 shows the result: the runner's outline glows while the flat background goes dark.

*Mini example.* If $g_x = 3$ and $g_y = 4$ at a pixel, then $M = \sqrt{9 + 16} = 5$ and $\alpha = \tan^{-1}(4/3) \approx 53°$.

**Step 3: histogram of (unsigned) gradients in 8×8 cells.** Divide the 64×128 patch into **cells of 8×8 pixels**. That gives $64/8 = 8$ cells across and $128/8 = 16$ cells down, so **128 cells**. Each cell holds 64 pixels, each with a magnitude and a direction (slide 41 shows the two 8×8 tables).

For each cell build a **histogram with 9 bins** covering directions **0°, 20°, 40°, …, 160°**. "Unsigned" means direction 190° is treated the same as 10°: an edge from dark‑to‑bright counts the same as bright‑to‑dark, so the range is 0–180° rather than 0–360°. Each pixel "votes" for the bin of its direction with a weight equal to its **magnitude**. If the direction falls between two bin centres, the vote is split proportionally.

*The slide's own example (slide 42):*
- A pixel with direction **80°** and magnitude **2** puts all 2 into the 80° bin.
- A pixel with direction **10°** and magnitude **4** sits exactly half‑way between the 0° and 20° bins, so 2 goes to each.
- A pixel with direction **165°** and magnitude **85** is between 160° and 180° (and 180° wraps round to the 0° bin). It is 5° from 160° and 15° from 180°, so the 160° bin gets $85 \times 15/20 = 63.75$ and the 0° bin gets $85 \times 5/20 = 21.25$. Those are exactly the numbers printed on the slide.

The output of step 3 is one 9‑number histogram per cell. The bar chart on slide 42 is one such histogram.

**Step 4: 16×16 block normalisation (robustness to illumination).** Gradient magnitudes depend on lighting: the same person under a bright lamp gives bigger numbers everywhere. To cancel this, group cells into **blocks of 16×16 pixels = 2×2 cells**. Concatenate the four 9‑bin histograms into a **36‑element vector** and divide it by its length (its norm), so the block vector has unit length. Now only the *relative* pattern of directions matters, not the overall brightness. Slide 43 shows one such block (blue square) in the top‑left of the cell grid.

**Step 5: concatenate.** The block slides across the patch with a step of 8 pixels (one cell), so blocks **overlap**. Horizontally there are $8 - 1 = 7$ block positions and vertically $16 - 1 = 15$, so **7 × 15 = 105 block positions**. Each gives a 36‑vector. Stacking them all: **36 × 105 = 3780 numbers**. That 3780‑dimensional vector *is* the HOG descriptor of the window, and it is the "3780 HOG features" in the pipeline of slide 38. Slide 44 shows the little arrow glyphs that visualise it: in each cell, the dominant directions are drawn as short lines, and together they sketch the runner.

**Summary table of the numbers:**

| Quantity | Value | Where it comes from |
|---|---|---|
| Patch size | 64×128 px | chosen for standing people |
| Cell size | 8×8 px | chosen |
| Cells | 8×16 = 128 | 64/8 and 128/8 |
| Bins per cell | 9 | 0°–160° in 20° steps (unsigned) |
| Block | 16×16 px = 2×2 cells | chosen |
| Values per block | 4 × 9 = 36 | four cell histograms |
| Block positions | 7×15 = 105 | (8−1) × (16−1) with 8‑px stride |
| Descriptor length | 36 × 105 = 3780 | everything concatenated |

**Common mistake.** Students write "128 cells × 9 bins = 1152" as the descriptor length. That forgets the overlapping block normalisation, which is what makes the length 3780. The overlap means each cell appears in up to four blocks.

### 10.3 SVM classification (slide 45)

Now each window is a point in a 3780‑dimensional space. A **Support Vector Machine (SVM)** is a classifier that finds a **separating hyperplane**

$$
\mathbf{w}^T \mathbf{x} + b = 0
$$

that best divides the training examples of "person" (positive) from "not person" (negative). Here $\mathbf{x}$ is the 3780‑vector, $\mathbf{w}$ is a learnt weight vector of the same size, and $b$ is a learnt offset. At run time you compute $\mathbf{w}^T\mathbf{x} + b$ for a window: positive means person, negative means no person. The slide's picture shows the "neg w" and "pos w" weight images with the green line between them.

The procedure: **"Train a window classifier; use the trained classifier to predict the object class in each window in the image."** "Best" for an SVM means the hyperplane with the largest **margin** (biggest gap to the nearest examples on each side), which is what makes it generalise well. ⚠️ The margin idea is background knowledge; the slide only shows the hyperplane equation.

**Analogy.** Imagine every window is a dot on a piece of paper, red for "person" and blue for "no person". The SVM draws the straight line that keeps the two colours apart with the widest possible empty corridor between them. A new dot is classified by which side of the line it lands on.

The references on slide 38–39 are Dalal & Triggs (2005), who invented HOG for human detection, and two hardware papers showing HOG+SVM running at 1080p and 60 frames per second.

---

## 11. Vision in control: visual servoing (slides 46–57)

Finally, we close the loop. Instead of only *perceiving*, the camera *drives* the robot.

### 11.1 The idea (slides 46–48)

**Vision in control** means you "identify the relative 3‑D position between an object inside the robot's workspace" and "extract, characterise and interpret information from images of the 3‑D world." Slide 46 shows an arm picking groceries, a ground vehicle following a path, and a drone tracking a gate.

The concept (slide 47) is a standard feedback loop: a desired value $\mathbf{s}^*$, the measured value $\mathbf{s}$ coming from a **Perception** block, their difference feeding a **Control** block, and the control output acting on the robot. "Visual sensor processing feeds a specific control law which provides actuator commands for the robot."

**Visual servoing** (slide 48) "refers to feedback control by using information from visual sensors." Three families:

1. **Pose control** (position‑based).
2. **Image‑based control**.
3. **Hybrid schemes** (a mix of the two).

**Advantages:** positioning accuracy; robustness with respect to modelling and calibration errors; reactive to changes (if the object moves, the robot follows).

**Disadvantages:** nonlinear perspective transformations (remember the division by $Z$); high sensitivity to ambient conditions (lighting, reflections).

**Analogy.** Threading a needle. You do not measure the thread and needle with a ruler and then move your hand blind; you keep *looking* and correcting until the thread goes through. That is visual servoing: continuous correction using what you see.

### 11.2 Camera positioning: eye‑in‑hand versus eye‑to‑hand (slides 49–50)

- **Eye‑in‑hand** (slide 49a): the camera $\{C\}$ is mounted **on the end‑effector** $\{E\}$. The pose chain is: base → $\xi_E$ → end‑effector → ${}^E\xi_C$ (fixed, known) → camera → ${}^C\xi_G$ (measured by vision) → goal $\{G\}$. The camera moves with the hand, so it always gets a close‑up of the target, but it can also lose sight of it.
- **Eye‑to‑hand** (slide 49b): the camera $\{C\}$ is **fixed in the workspace**, looking at both the robot and the goal. Chains: base → $\xi_C$ → camera → ${}^C\xi_G$ → goal, and base → $\xi_E$ → end‑effector → ${}^E\xi_G$ → goal. It sees the whole scene but with less detail, and the arm can occlude the target.

Slide 50 shows a video of a red arm servoing towards an object held by a person.

**Common confusion.** "Eye‑in‑hand" = camera *in* the hand (moving). "Eye‑to‑hand" = camera looks *at* the hand (fixed). Say it out loud and the preposition tells you which is which.

### 11.3 PBVS versus IBVS (slides 51–56)

**Position‑based visual servoing (PBVS)**:
- "Visual information is used to reconstruct the current **3‑D pose** (position/orientation) of an object."
- "Combined with the knowledge of a desired 3‑D pose, a **Cartesian pose error** signal that drives the robot to the goal is regulated."

**Image‑based visual servoing (IBVS)**:
- "Error is computed directly on the values of the **features extracted on the 2‑D image plane** (the 3‑D reconstruction part is omitted)."
- "The robot should move so as to bring the current image features to their desired values."

The comparison diagram (slide 52) puts them side by side:

- **Top (PBVS):** sensor → visual information → compute relative 3‑D pose (the pose graph with ${}^C\xi_G$, the desired ${}^{C^*}\xi_G$ and so on) → subtract the desired 3‑D pose → **3‑D Cartesian error actuates the robot**.
- **Bottom (IBVS):** current image (a square at one place in pixel coordinates) and desired image (the square where you want it) → subtract → **error in the 2‑D image plane** → processing → actuation command.

The block diagrams make the difference concrete:

**PBVS loop (slide 53):** camera image → *Feature extraction* gives features $\mathbf{f}$ → *Pose estimation* turns them into the estimated pose ${}^C\xi_G$ → compared with the desired pose ${}^C\xi_G^*$ → *PBVS control* → *Joint controller* (with encoder feedback) → robot. Notice the extra **Pose estimation** box.

**IBVS loop (slide 55):** camera image → *Feature extraction* gives $\mathbf{f}$ → compared directly with desired features $\mathbf{f}^*$ → *IBVS control* → *Joint controller* → robot. No pose estimation box at all.

**PBVS summary (slide 54):**
- Robot alignment in Cartesian coordinates (easy to reason about: "move 5 cm left").
- Requires extraction of 3‑D information of the target (so you need a model of the target or depth).
- Sensitive to calibration errors (a wrong $f$ gives a wrong 3‑D pose).
- Sensitive to reconstruction errors (noisy features give a noisy pose).

**IBVS basic components (slide 56):**
1. Image acquisition.
2. Feature extraction (e.g. corner points of the square).
3. Feature error function ($\mathbf{f} - \mathbf{f}^*$).
4. Generation of motion of the camera robot, which needs: perspective transformations (how camera motion moves image features, the division by $Z$ again), differential kinematics/dynamics (how joint velocities move the camera), and control laws for kinematics/dynamics.

**Everyday analogy for the two schemes.** Parking a car with a rear camera. *PBVS* is working out "the bay is 1.2 m back and 15° to the right" and then steering to that computed spot. *IBVS* is simply steering until the painted lines in the camera image line up with the guide lines drawn on the screen; you never compute the actual distance.

**Which is better?** Neither in general. PBVS gives nice straight‑line motion in 3‑D but depends on accurate calibration and 3‑D reconstruction. IBVS is robust to calibration error (it only needs features to end up in the right place in the image) but the robot's 3‑D path can be strange and the perspective nonlinearity can cause trouble for large motions. Hybrid schemes try to get the best of both.

### 11.4 Applications (slide 57)

The slide revisits the examples: visual‑servoing navigation along row‑crop fields, a drone aligning to a marker on a structure ("convergence is ensured"), a spacecraft docking approach with the Moon in the background, a humanoid tracking an ArUco marker, and an underwater vehicle following a pipeline by camera. All of them are "look → compare → move → look again" loops.

---

## In a nutshell

A camera is a device that turns light into a grid of numbers, and machine vision is the art of turning those numbers back into meaning a robot can act on. The geometry is simple once you see it: a 3‑D point is projected onto the image plane by dividing by its depth ($x = fX/Z$), which you can write as one camera matrix in homogeneous coordinates and then convert to pixels with the pixel size and principal point. One camera loses depth, so stereo uses two cameras and the disparity between them to get it back. To find things in an image you use colour spaces like HSV to isolate hue, thresholding to split pixels into regions, and kernels (correlation and convolution) to blur, sharpen or find edges. The HOG + SVM human detector shows how those pieces combine: gradients → cell histograms → normalised blocks → a 3780‑dimensional descriptor → a linear classifier, applied in a sliding window over an image pyramid. Finally, visual servoing closes the loop, either by reconstructing a 3‑D pose and regulating a Cartesian error (PBVS) or by driving image features straight to their desired pixel positions (IBVS).

---

## Key terms

- **Machine vision**: automatic extraction of meaningful information from images and videos.
- **Occlusion**: when one object blocks the camera's view of another.
- **Field of view**: the angular wedge of the world a camera can see.
- **CCD / CMOS sensor**: the chip that converts photons into electrons (electric charge).
- **ADC**: analogue‑to‑digital converter; turns the sensor's voltage into integer pixel values.
- **Bayer grid**: a mosaic of red, green and blue filters over the sensor so each pixel records one colour.
- **Demosaicing**: estimating the missing two colours at each pixel from its neighbours.
- **Image as a function**: $f(x,y)$ gives the intensity at location $(x,y)$; stored as an $M\times N$ array.
- **RGB**: colour as red, green, blue amounts; a cube with black at the origin and white at the far corner.
- **HSV**: colour as hue (position on the colour wheel), saturation (vividness), value (max of R, G, B).
- **Pinhole camera**: a box with a tiny hole; each scene point maps to one image point; image is inverted.
- **Thin lens**: ideal lens with focal length $f$ that behaves like an equivalent pinhole but gathers more light.
- **Focal length $f$**: distance from the camera centre to the image plane.
- **Principal point $(u_0, v_0)$**: where the optical axis pierces the image plane, in pixels.
- **Perspective projection**: $x = fX/Z$, $y = fY/Z$; far things look small; depth is lost.
- **Homogeneous coordinates**: adding an extra coordinate so projection becomes a matrix product.
- **Camera matrix $\mathbf{C}$**: the 3×4 matrix $\mathrm{diag}(f, f, 1)$ with a zero column that performs projection.
- **Intrinsic matrix $\mathbf{K}$**: 3×3 matrix with $1/\rho_w$, $1/\rho_h$, $u_0$, $v_0$ converting metres on the sensor to pixels.
- **Camera calibration**: measuring $f$, $\rho$, $u_0$, $v_0$ (and, for stereo, the relative camera pose).
- **Baseline**: the distance between the two camera centres in a stereo pair.
- **Epipolar line**: the line in one image along which the match for a point in the other image must lie; a horizontal row after rectification.
- **Rectification**: warping stereo images so epipolar lines are horizontal rows.
- **Disparity**: horizontal pixel shift of a point between left and right images; larger for near points.
- **Triangulation**: computing depth from disparity, $Z = fb/d$.
- **Segmentation**: partitioning an image into meaningful regions.
- **Thresholding**: labelling pixels by comparing intensity with a threshold $T$ (global, local/regional, adaptive, multiple).
- **Histogram**: a count of how many pixels have each intensity value; valleys suggest thresholds.
- **Kernel**: a small table of weights slid over the image in a spatial operation.
- **Correlation**: multiply‑and‑add with the kernel as it is ($\mathbf{I}[u+i, v+j]$).
- **Convolution**: multiply‑and‑add with the kernel flipped 180° ($\mathbf{I}[u-i, v-j]$).
- **Sobel kernel**: a derivative kernel that highlights edges in one direction.
- **Laplacian kernel**: a second‑derivative kernel that highlights edges in all directions.
- **Sliding window**: testing a fixed‑size window at every position in the image.
- **Image pyramid**: copies of the image at several scales so one window size can find objects of any size.
- **Gradient magnitude / direction**: $\sqrt{g_x^2 + g_y^2}$ and $\tan^{-1}(g_y/g_x)$; edge strength and orientation.
- **HOG**: Histogram of Oriented Gradients; a 3780‑long descriptor of edge directions in a 64×128 window.
- **Block normalisation**: dividing each 16×16 block's 36‑vector by its norm for lighting robustness.
- **SVM**: Support Vector Machine; a classifier that separates classes with a hyperplane $\mathbf{w}^T\mathbf{x} + b = 0$.
- **Visual servoing**: feedback control using information from visual sensors.
- **Eye‑in‑hand**: camera mounted on the robot's end‑effector.
- **Eye‑to‑hand**: camera fixed in the workspace, watching the robot and target.
- **PBVS**: position‑based visual servoing; reconstruct 3‑D pose, regulate a Cartesian pose error.
- **IBVS**: image‑based visual servoing; regulate the error between current and desired 2‑D image features directly.
- **Hybrid visual servoing**: schemes mixing PBVS and IBVS ideas.
- **ArUco marker**: a printed black‑and‑white square tag that is easy to detect and gives its pose.

---

## Check yourself

1. A camera has focal length $f = 4$ mm, square pixels of $5\ \mu$m and principal point $(u_0, v_0) = (320, 240)$. A point is at $(X, Y, Z) = (0.3, -0.15, 1.5)$ m in the camera frame. Where does it appear in the image, in pixels?
2. Why is HSV preferred over RGB when you want a robot to pick out "all the red objects"? Give the one‑sentence reason from the slides.
3. In HOG, explain where the numbers 105 and 3780 come from.
4. What is the single difference between the correlation and convolution formulas, and for which kind of kernel does the difference disappear?
5. Describe, in one or two sentences each, the difference between PBVS and IBVS, and name one weakness of PBVS from the slides.
6. A stereo rig has $f = 700$ pixels and baseline $0.1$ m. A point has disparity 35 pixels. What is its depth? What happens to the depth error if the matching is off by one pixel for a far point versus a near point?

<details>
<summary>Answers</summary>

**1.** First project: $x = 4 \times 0.3 / 1.5 = 0.8$ mm, $y = 4 \times (-0.15) / 1.5 = -0.4$ mm. Convert to pixels with $\rho = 0.005$ mm: $u = 0.8/0.005 + 320 = 160 + 320 = 480$; $v = -0.4/0.005 + 240 = -80 + 240 = 160$. The point appears at pixel $(480, 160)$, to the right of and above the principal point (negative $y$ is up in this convention).

**2.** Because in HSV "perceptual colour features such as saturation are represented individually and can be modified independently", so hue (the colour name) is separated from brightness and vividness, whereas in RGB "any coordinate movement modifies colour tone, saturation and brightness all at once." Thresholding the hue channel finds red regardless of shading.

**3.** The 64×128 patch is split into 8×8‑pixel cells, giving 8 cells across and 16 down. Blocks are 16×16 pixels (2×2 cells) and slide with an 8‑pixel stride, so there are $(8-1) \times (16-1) = 7 \times 15 = 105$ block positions. Each block concatenates four 9‑bin histograms, giving 36 values, and $36 \times 105 = 3780$.

**4.** Correlation uses $\mathbf{I}[u+i, v+j]$ and convolution uses $\mathbf{I}[u-i, v-j]$; in other words convolution flips the kernel 180° before the multiply‑and‑add. For a symmetric kernel (mean blur, Gaussian blur, Laplacian) the flipped kernel is identical, so both operations give the same result.

**5.** PBVS uses the image to reconstruct the object's 3‑D pose and regulates a Cartesian pose error between the estimated and desired pose. IBVS skips 3‑D reconstruction and regulates the error between current and desired feature positions directly in the 2‑D image plane. PBVS weaknesses from the slides: it requires extracting 3‑D information of the target, and it is sensitive to calibration errors and to reconstruction errors.

**6.** $Z = fb/d = 700 \times 0.1 / 35 = 2.0$ m. For a far point the disparity is small, so a one‑pixel error is a large *fraction* of the disparity and the depth error is large (e.g. $d = 5 \to 6$ changes $Z$ from 14 m to 11.7 m). For a near point the disparity is large, so one pixel barely matters (e.g. $d = 70 \to 71$ changes $Z$ from 1.0 m to 0.986 m). Stereo depth accuracy degrades with distance.

</details>
