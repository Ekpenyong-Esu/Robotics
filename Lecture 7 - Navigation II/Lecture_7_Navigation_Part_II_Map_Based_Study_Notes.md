# Lecture 7 (Part II) – Robot Navigation: Map-Based Path Planning

*Study notes written from the slide deck `R7010E_Lecture_7_Navigation_2023.pdf` (slides 42–89) and the MATLAB script `R7010E_Lecture_7_Navigation.m`. Part I (reactive navigation, slides 1–41) lives in the folder "Lecture 7 - Navigation". The two folders contain identical copies of the same deck; the deck itself announces on slide 11 that it covers two lectures.*

---

## What this lecture is about, and why it matters

In Part I the robot had no map and reacted to whatever it bumped into or sensed. That works, but it gives no guarantee of a short route and it can get stuck. In this part you assume the robot **has a map**, and you learn the classic algorithms that search that map for a good path: grid search with Dijkstra and A*, the dynamic version D*, lattice planners that respect how a car can actually turn, and the random sampling methods PRM and RRT. These are the algorithms running inside every self-driving car, warehouse robot, and the TurtleBot you use in the labs, so understanding how they search, what they guarantee, and what they cost is core exam material.

## Roadmap: the main topics in this part

1. Occupancy grids in 2-D and 3-D
2. Planning by searching the grid: the search tree, neighbours, and costs
3. Uninformed vs informed search: Dijkstra ($f = g$) vs A* ($f = g + h$)
4. The A* algorithm with open and closed lists, worked through on a graph and on a grid
5. Why you must not stop the moment you *see* the goal
6. Reading the plan back, and online re-planning
7. D* (Dynamic A*): cost maps, map updates, and the fixed goal
8. Lattice planners: planning in $(x, y, \theta)$ with motion primitives
9. Roadmaps and sampling-based planning: "advanced guessing"
10. Probabilistic Roadmaps (PRM)
11. Rapidly-Exploring Random Trees (RRT), including kinematic RRT and the ERRT exploration variant
12. The two-layer architecture: slow global planning plus fast collision avoidance

---

## 1. Occupancy grids (slides 43–44)

Before you can search a map, you need a map the computer can search. The standard choice is the **occupancy grid**: you chop the world into little square cells of a fixed size (the **resolution**) and store one number per cell.

**2-D binary occupancy grid (slide 43).** Each cell is either

- **0 = free** (drawn white): the robot may be here, or
- **1 = occupied** (drawn black): the robot can never enter.

The slide's example is a 13 m × 13 m room drawn as a binary grid of walls. Why is this representation so popular?

- It **discretises** the world to a chosen resolution, so a continuous, infinitely detailed world becomes a finite list of cells that a computer can loop over.
- **Collision checks are trivial**: "Is this cell 1?" Compare that with checking whether a curve intersects a polygon.
- It is used in **almost all iterative path planners**, including every algorithm in this lecture.

Think of a sheet of graph paper laid over a floor plan, with every square that touches a wall coloured in. That is an occupancy grid.

**3-D occupancy grid (slide 44).** The same idea extends to 3-D using little cubes called **voxels** (volume pixels). The slide shows a tunnel mapped by a drone, with voxels coloured by height. In 3-D the number of cells explodes, so real systems (such as the OctoMap library) store them in a tree that keeps big empty regions as one block. The slide does not go into that; just know that the concept is identical.

> **Common mistake.** The resolution is a trade-off, not a free choice. Finer cells capture narrow gaps but multiply the number of cells (and the planning time) by the square (2-D) or cube (3-D) of the factor. Coarser cells are fast but may swallow a doorway.

---

## 2. Planning by searching the grid (slides 45–47)

### The search tree

Once the world is a grid, path planning becomes a **graph search** problem. Every free cell is a **node**, and there is an **edge** between two cells if the robot can move directly from one to the other. Slide 45 shows how this unfolds: start at the start cell (the single red circle), look at its neighbours (the ring of red cells around it, the first level of the tree), then at *their* neighbours (the next ring), and so on. The search spreads outwards like ripples until a ripple reaches the goal (the green cell). Each "ring" is one level deeper in the search tree.

### Neighbours and movement costs (slides 46–47)

In a 2-D grid the usual choice is **8-connectivity**: from a cell you can move to its 4 side neighbours and its 4 diagonal neighbours (slide 46's yellow cell with eight arrows). Slide 47 fixes the notation: cell $x_1$ in the middle, $x_2 \dots x_9$ around it, and a **cost** $c(x_i, x_j)$ for moving between neighbours:

| Move | Cost |
|---|---|
| Straight (e.g. $x_1 \to x_2$) | $c = 1$ |
| Diagonal (e.g. $x_1 \to x_9$) | $c = 1.4$ (≈ $\sqrt 2$) |
| Straight into an obstacle cell | $c = 10000$ |
| Diagonal into an obstacle cell | $c = 10000.4$ |

The huge numbers for obstacle cells are a practical trick: rather than deleting obstacle nodes from the graph, you make them so expensive that no sensible path uses them. The "+0.4" just keeps the diagonal slightly worse than the straight move, as in free space.

### Heuristic distance vs actual distance (slide 47)

The left figure on slide 47 contrasts two things you must keep apart:

- The **actual** path cost: the sum of the move costs along the path the robot really takes on the grid (the L-shaped "actual" and "distance" lines in the figure).
- The **heuristic distance**: an *estimate* of the remaining cost, here the straight line from the current cell to the goal, which ignores obstacles and the grid.

The heuristic is always a *guess*. The whole art of informed search is choosing a guess that is cheap to compute and never over-estimates.

### Search, informed and uninformed (slide 46)

The slide defines **search** as the problem of finding a sequence of actions (a *path*) that leads to a desirable state (a *goal*).

- **Uninformed search** (exhaustive search) explores in all directions equally. It has no idea where the goal is; it just keeps expanding until it stumbles on it.
- **Informed search** (heuristic approach) uses extra domain knowledge, a **heuristic**, to say that one node is "more promising" than another, and explores the promising ones first. Example algorithms: greedy best-first search, **A\***, the many variants of A*, **D\***.

Analogy: looking for your keys with no clue is uninformed search, you check every room. Knowing you last had them in the kitchen is a heuristic, and checking the kitchen first is informed search.

---

## 3. Dijkstra vs A* (slide 48)

Both algorithms rank nodes with a priority value $f$ and always expand the node with the smallest $f$ first. The only difference is what goes into $f$.

**Dijkstra's algorithm (uninformed):**
$$ f(x) = g(x) $$

**A\* (informed):**
$$ f(x) = g(x) + h(x) $$

where

- $g(x)$ = **the cost incurred so far**, from the start node to $x$ (the sum of all arc costs $c$ along the path found to $x$).
- $h(x)$ = **the estimated cost from $x$ to the goal**, the **heuristic**.
- $f(x)$ = the estimated total cost of a path from start to goal that passes through $x$.

Dijkstra is the special case $h = 0$: it only looks backwards (how far have I come) and so expands in a perfect circle around the start. A* also looks forwards (how far do I think I still have to go) and so stretches that circle into an ellipse pointing at the goal. Same path quality, far fewer nodes expanded.

### Choosing a heuristic

For A* to give the optimal path, $h$ must be **admissible**: it must *never over-estimate* the true remaining cost. Two standard choices on a grid:

- **Euclidean distance** $h = \|x - g\| = \sqrt{\Delta x^2 + \Delta y^2}$: the straight line. Always admissible, because nothing is shorter than a straight line.
- **Manhattan distance** $h = |\Delta x| + |\Delta y|$: the number of straight moves if you could not move diagonally. Admissible on a 4-connected grid; on an 8-connected grid it can over-estimate (a diagonal move covers two Manhattan units for a cost of 1.4), so strictly speaking it is not admissible there. Many tutorial examples, including the grid example on slides 55–67, use it anyway because it is simple.

Slide 49 gives a concrete calculation with the Euclidean heuristic. For a cell three steps from the start with the goal 8 columns and 18 rows away:

$$ g(x) = 3, \quad h(x) = \sqrt{8^2 + 18^2} = 19.7, \quad f(x) = 22.7 $$

and for a cell one step further along but further from the goal:

$$ g(x) = 4, \quad h(x) = \sqrt{11^2 + 18^2} = 21.1, \quad f(x) = 25.1 $$

A* will expand the first cell ($f = 22.7$) before the second ($f = 25.1$). The slide's bullet points: use domain knowledge to bias the search, favour actions that might get closer to the goal, give each state a value $f(x) = g(x) + h(x)$, choose the state with the best (smallest) $f$.

> **Common mistake.** People confuse $g$ and $h$. $g$ is *known* and *exact*: you actually travelled it. $h$ is *estimated*: you have not travelled it yet. Dijkstra uses only the known part.

---

## 4. The A* algorithm (slide 50)

A* keeps two lists:

- **Open list** $O$: nodes that have been *discovered* but not yet *expanded*. These are candidates for the next step. (Implemented as a priority queue sorted by $f$.)
- **Closed list** $C$: nodes that have already been expanded. You never look at them again.

Each node also stores a **back-pointer** $b(x)$ to the node it was reached from, so that at the end you can trace the path backwards.

The flowchart on slide 50, written out:

```
Put the start node in O.
loop
    if O is empty:  stop, no path exists.
    Pick n_best from O with the smallest f (f(n_best) <= f(n) for all n in O).
    Remove n_best from O and add it to C.
    if n_best is the goal:  stop, path found.
    for every neighbour x of n_best that is not in C:
        if x is not in O:
            compute g(x) = g(n_best) + c(n_best, x), set b(x) = n_best, add x to O.
        else (x already in O):
            if g(n_best) + c(n_best, x) < g(x):
                update g(x) and set b(x) = n_best    (we found a cheaper way to x)
```

Notice three things:

1. The goal test happens when the goal is **popped** from $O$ (chosen as $n_{best}$), **not** when it is first added. This is the subject of Section 5.
2. A node already in $O$ can have its $g$ *improved* if a cheaper route to it shows up. That is the "update $b(x)$" box.
3. Nodes in $C$ are never revisited. With an admissible (and consistent) heuristic this is safe.

### Worked example on a graph (slides 51–54)

The slides use a small hand-drawn graph. Here is its structure. Numbers inside circles are $h(x)$; numbers on edges are arc costs $c$.

```
                     Start (h=0)
              c=1   /    |    \   c=1
                   A     B     C          h: A=3, B=2, C=3
                  /|\   /|\   /|\
           c:   1 1 3  4 1 2  1 1 1
               D  E F  G H I  J K L      h: D=3 E=1 F=3 G=2 H=1 I=2 J=3 K=2 L=3
                  |    |   |    |
             c=3  |  3 |   | 3  | 2
                 GOAL GOAL GOAL GOAL
```

Edges to GOAL: $E \to$ GOAL costs 3, $G \to$ GOAL costs 3, $I \to$ GOAL costs 3, $K \to$ GOAL costs 2. Priority $= g + h$, and $g$ is the sum of all arc costs from the start.

**Step 1. Expand Start.** Its children get $g = 1$ each.

| Node | $g$ | $h$ | $f$ |
|---|---|---|---|
| B | 1 | 2 | **3** |
| A | 1 | 3 | 4 |
| C | 1 | 3 | 4 |

Open list: B(3), A(4), C(4). Closed: {Start}.

**Step 2. Expand B** (smallest $f$). Children of B: H ($g = 1+1 = 2$, $h = 1$, $f = 3$), I ($g = 1+2 = 3$, $h = 2$, $f = 5$), G ($g = 1+4 = 5$, $h = 2$, $f = 7$).

Open list: H(3), A(4), C(4), I(5), G(7). Closed: {Start, B}.

**Step 3. Expand H.** H has no children ("no expansion", a dead end). Nothing is added.

Open list: A(4), C(4), I(5), G(7). Closed: {Start, B, H}.

**Step 4. Expand A** (tie with C; the slide takes A first). Children: E ($g = 2$, $h = 1$, $f = 3$), D ($g = 2$, $h = 3$, $f = 5$), F ($g = 1+3 = 4$, $h = 3$, $f = 7$).

Open list: E(3), C(4), D(5), I(5), F(7), G(7). Closed: {Start, B, H, A}.

**Step 5. Expand E.** Child: GOAL ($g = 2 + 3 = 5$, $h = 0$, $f = 5$).

Open list: C(4), GOAL(5), D(5), I(5), F(7), G(7). Closed: {Start, B, H, A, E}.

At this moment slide 53 shows the path **Start → A → E → GOAL** with cost 5 and asks in orange: **"Did we finish?????"**

### 5. Did we finish? (slide 54)

**No.** The goal is in the open list, but it is *not* at the front. C(4) has a smaller $f$. Slide 54 explains: *there might be a shorter path, but assuming non-negative arc costs, nodes with a lower priority than the goal cannot yield a better path*. Read that carefully: a node with $f$ **smaller** than the goal's $f$ *might* still lead to a better path, so you must expand it. Nodes with $f \ge 5$ cannot improve on a cost-5 path, so they can be **pruned** (the brown cells on the slide: D, I, F, G, and later L, J).

**Step 6. Expand C** ($f = 4 < 5$). Children: K ($g = 2$, $h = 2$, $f = 4$), L ($g = 2$, $h = 3$, $f = 5$), J ($g = 2$, $h = 3$, $f = 5$).

Open list: K(4), GOAL(5), L(5), J(5), D(5), I(5), … Closed: {…, C}.

**Step 7. Expand K** ($f = 4$). Child: GOAL via K with $g = 2 + 2 = 4$, $h = 0$, $f = 4$. That is cheaper than the GOAL(5) already in the open list, so its $g$ is updated to 4 and its back-pointer to K.

Open list: GOAL(4), L(5), J(5), … 

**Step 8. Pop GOAL** ($f = 4$, the smallest). Now it is $n_{best}$ and the goal test fires. **Finished.** Tracing back-pointers: GOAL ← K ← C ← Start. The optimal path is **Start → C → K → GOAL with cost 4**, not the cost-5 path you saw first.

> **Common mistake, and the moral of the example.** Stopping as soon as the goal appears in the open list gives you *a* path, not the *shortest* path. A* is only optimal if you stop when the goal is *popped* (has the smallest $f$ of everything left). Everything still in the open list then has $f \ge f(\text{goal})$, and with non-negative costs none of it can beat the goal.

### Worked example on a grid (slides 55–67)

The second example runs A* on an 8-connected grid with these conventions:

- Straight move cost **10**, diagonal move cost **14** (that is 1 and 1.4 scaled by ten to avoid decimals).
- $h$ = **10 × Manhattan distance** to the goal.
- Each cell shows $f$ (red, top), $g$ (green, bottom-left) and $h$ (blue, bottom-right).
- A yellow frame means "added to the closed list".

Let me set up coordinates (column, row), counting from 0 at the top-left. The **Start** is at (3, 4) and the **Goal** at (7, 3). A vertical wall occupies column 5 in rows 2–5, plus the extra cell (6, 5). So the robot must go *over the top* of the wall (row 1 or 0) or far under it.

**Step 1 (slide 55). Close Start, open its 8 neighbours.**

| Cell | from Start | $g$ | $h$ (10 × Manhattan to (7,3)) | $f$ |
|---|---|---|---|---|
| (4, 3) up-right | diagonal | 14 | 30 | **44** |
| (3, 3) up | straight | 10 | 40 | 50 |
| (4, 4) right | straight | 10 | 40 | 50 |
| (2, 3) up-left | diagonal | 14 | 50 | 64 |
| (4, 5) down-right | diagonal | 14 | 50 | 64 |
| (2, 4) left | straight | 10 | 60 | 70 |
| (3, 5) down | straight | 10 | 60 | 70 |
| (2, 5) down-left | diagonal | 14 | 70 | 84 |

Check one by hand: (4, 3) is 3 columns and 0 rows from the goal, so $h = 10 \times 3 = 30$; reached diagonally so $g = 14$; $f = 44$. 

**Step 2 (slide 56). Close (4, 3), $f = 44$.** Its right-hand neighbours (5, 2), (5, 3), (5, 4) are wall. New opens: (4, 2) with $g = 14 + 10 = 24$, $h = 40$, $f = 64$; (3, 2) with $g = 14 + 14 = 28$, $h = 50$, $f = 78$. Cells already open keep their values because the new routes are not cheaper (e.g. (3, 3) via (4, 3) would cost 24 > 10).

**Step 3 (slide 57). Close (4, 4), $f = 50$** (tie with (3, 3); the slide picks (4, 4)). Its right neighbour (5, 4) is wall; (5, 5) is wall; nothing new is opened.

**Step 4 (slide 58). Close (3, 3), $f = 50$.** New open: (2, 2) with $g = 10 + 14 = 24$, $h = 60$, $f = 84$.

> ⚠️ Strictly, closing (3, 3) should also *update* (3, 2): the route Start → (3,3) → (3,2) costs $10 + 10 = 20$, cheaper than the 28 it currently has, so its $f$ should drop to 70. The slide leaves it at 78. The same slides *do* perform such an update later (step 8, cell (1, 4) drops from 98 to 90), so this looks like a small inconsistency in the drawing rather than a different algorithm. Follow the flowchart on slide 50: always update when a cheaper route appears.

**Steps 5–7 (slides 59–61).** Close (4, 5) $f = 64$ (opens the row below: (3,6) 98, (4,6) 84, (5,6) 78); close (4, 2) $f = 64$ (opens the row above: (3,1) 98, (4,1) 84, (5,1) 78); close (2, 3) $f = 64$ (opens (1,2) 98, (1,3) 84, (1,4) 98). Notice how the search is spreading up, down and left symmetrically, because the wall blocks the direct route and the heuristic alone cannot tell which way round is better.

**Steps 8–9 (slides 62–63).** Close (2, 4) and (3, 5) at $f = 70$, then (4, 1) at $f = 84$, which opens row 0. The frontier is now above the wall.

**Steps 10–12 (slides 64–66).** Close (5, 1) $f = 78$: it is the first cell *past* the wall, and opens (6, 0) 92, (6, 1) 78, (6, 2) 72. Close (6, 1) and (6, 2), opening (7, 0) 92, (7, 1) 78, (7, 2) 72. Close (7, 2) $f = 72$, whose neighbour is the Goal.

**Step 13 (slide 67).** The goal is reached and the slide highlights the solution corridor in yellow. Tracing back-pointers gives the path

**Start (3,4) → (4,3) → (4,2) → (5,1) → (6,2) → Goal (7,3)**

with cost $14 + 10 + 14 + 14 + 14 = 66$ (four diagonals and one straight move). Every other way around the wall is longer, so this is optimal under these costs.

> ⚠️ A few numbers on slides 63–66 (for instance the 38 in cell (2, 1)) do not follow from any neighbour by the 10/14 rule. Treat the slides' grid as an illustration of the *process*; if you need to reproduce it in an exam, apply the rules above consistently and your numbers will be right even where the slide's are not.

### Online re-planning (slide 68)

In the real world the map is never perfect. Slide 68 wraps A* in a loop:

```
Start
  → Construct a path from the known map using A*
  → Move the robot along the path
  → Reached the goal?  yes → End
  → Detected a discrepancy (sensor sees something the map did not)?
        no  → keep moving
        yes → Update the map → re-run A* from the current position → continue
```

This simple "plan, move, check, re-plan" cycle is called **online planning** or **re-planning**. It works, but re-running A* from scratch every time a new obstacle appears is wasteful, which leads to D*.

### Reading the plan back (slide 69)

Once the search has reached the goal you have a **tree** that contains a path from the start (the root) to the goal (some leaf). You **follow the parent pointers** (the back-pointers $b(x)$) from the goal back to the root, keeping track of every state you pass through. That set of states, reversed, *is* your plan.

The slide also flags a general truth with a star: **there is a trade-off between the computational complexity you can afford and how good a plan you need.** Exhaustive search gives optimal plans but is slow; cheaper methods (coming up) give "good enough" plans fast.

---

## 7. D* – Dynamic A* (slides 69–71)

**A\*** works when you have *perfect knowledge* of the map. **D\*** ("Dynamic A*") allows you to *correct knowledge errors efficiently*: when the robot discovers that a cell is blocked (or more expensive) than the map said, D* repairs only the affected part of the plan instead of starting over.

### How D* thinks about the problem

D* turns the search around. Instead of searching from the start to the goal, it plans **from the goal outwards** and computes, for *every* cell in the map, the cost of the cheapest path from that cell to the goal. This table is the **cost map** (the grey-scale images on slides 70–71: dark means close to the goal, light means far, with the colour bar in "distance to goal (cells)"). Once you have the cost map, finding the path from *any* start is trivial: always step to the neighbour with the lowest cost, downhill to the goal.

Why plan backwards? Because the robot moves, the goal does not. If the goal is the root of the search, then when the robot finds a surprise obstacle somewhere, only the costs of cells "behind" that obstacle (on the robot's side) need fixing. The part of the cost map near the goal is still valid.

### D* in MATLAB (slides 70–71 and the `.m` file)

```matlab
load house                         % the textbook house map with named places
ds = Dstar(house);                 % create a D* planner on this occupancy grid
ds.plot                            % show the map
c  = ds.costmap();                 % the per-cell traversal costs (1 for free, Inf for walls)
ds.plan(place.kitchen, 'animate'); % plan: compute cost-to-goal for every cell, goal = kitchen
p  = ds.query(place.br3);          % query: path from bedroom 3 to the kitchen
ds.plot(p)                         % draw the path on the cost map
ds.niter                           % number of iterations the planning took
```

The slide's figure sequence: plain map → cost map (dark bowl centred on the kitchen) → green path from br3 descending into the bowl.

### Changing the map (slide 71)

Now pretend a region becomes harder to cross, perhaps a wet floor or a door that is now half closed:

```matlab
ds.modify_cost([300, 325; 115, 125], 5);   % make the cells x=300..325, y=115..125 five times more expensive
ds.plan                                    % incremental re-plan: only the affected costs are repaired
p = ds.query(place.br3);
ds.plot(p)
```

(The `.m` file uses a factor of 50 instead of 5; any value above 1 just penalises that region.) The resulting path (bottom picture on slide 71) now swings around the modified rectangle instead of through it. The key observation on the slide: **the cost map can be re-formed based on map updates and the path re-calculated**, and `ds.niter` after the second `plan` is far smaller than after the first, because most of the cost map was still valid.

### The catch

> **The goal cannot be changed!** Because the entire cost map is built relative to one goal, if you want a different goal you must re-plan from scratch. D* is ideal for "drive to *this* place through a world that keeps changing", not for "drive to a different place every minute".

Summary: **A\*** = perfect map, plan once. **D\*** = imperfect map, plan once, repair cheaply as surprises arrive, same goal throughout.

---

## 8. Lattice planners (slides 72–73)

Everything so far treated the robot as a point that can hop to any of its 8 neighbouring cells. A car cannot do that: it cannot move sideways and it cannot turn on the spot. A **lattice planner** fixes this by building the graph out of motions the robot can *really* perform.

From slide 72:

- **The kinematics of the robot are taken into consideration.**
- It still **uses a grid**, but...
- **the configuration is $(x, y, \theta)$**, position *and* heading, so the planning happens in 3 dimensions.
- A **cost** is applied to each motion, and **A\*** is used to search the graph.

### Motion primitives

From any grid node the car is allowed exactly three short motions, called **motion primitives**: drive straight one cell ahead, or drive a quarter-circle arc to the cell one ahead and one to the left (ending turned 90°), or the mirror image to the right. Slide 72 shows how the lattice grows: after 1 iteration (#1) you have 3 arcs leaving the origin; after 2 iterations (#2) each endpoint sprouts 3 more; after 8 iterations (#8) the arcs tile the whole grid in a pattern of circles. The 3-D plot underneath shows the same lattice with $\theta$ as the vertical axis: every arc that turns also climbs or descends in $\theta$.

### Costs and MATLAB (slide 73 and the `.m` file)

```matlab
lp = Lattice();                  % an obstacle-free lattice planner
lp.plan('iterations', 2);        % grow the lattice for 2 iterations
lp.plot()
lp.plan('iterations', 8);        % 8 iterations: dense coverage
lp.query([1 2 pi/2], [2 -2 0]);  % from (1,2) facing +y  to  (2,-2) facing +x
lp.plot

lp = Lattice();
lp.plan('cost', [1 10 10], 'iterations', 8);   % cost of [straight, turn-left, turn-right]
lp.query([1 2 pi/2], [2 -2 0]);                % A* path cost 35
lp.plot()
```

The cost vector is `[straight, left, right]`. With equal costs (slide 73a) the path is an S-shaped curve that turns as often as it likes. With `[1 10 10]` (slide 73b) turning costs ten times more than going straight, so A* chooses a path with as few arcs as possible: long straight runs with two tight turns, total cost 35. This is how you encode preferences like "a real car should not wiggle".

The `.m` file also shows a lattice on an occupancy map:

```matlab
load road
lp = Lattice(road, 'grid', 5, 'root', [50 50 0]);   % grid spacing 5 cells, lattice rooted at (50,50,0)
lp.plan();
lp.query([30 45 0], [50 20 0]);
lp.plot()
```

Any arc that touches an occupied cell is simply not added to the lattice, so collision checking is built in.

> **Common mistake.** A lattice path is drivable by construction; an 8-connected grid path is *not* (it has 45° and 90° corners a car cannot take). If an exam asks "why a lattice planner?", the answer is: because the output respects the vehicle's turning constraints.

---

## 9. Roadmaps and sampling-based planning (slides 74–75)

### Roadmaps

A **roadmap** is a graph of safe "highways" through free space, built *once*. Afterwards, any query is answered by connecting the start and goal to the nearest highway and searching the small graph. Slide 74 shows the classic construction on the lecture's 100×100 map:

- (a) the original binary map,
- (b) its **skeleton**: the set of points that are equally far from two or more obstacles (a generalized Voronoi diagram). Driving along the skeleton keeps you as far from walls as possible,
- (c) the skeleton overlaid on the obstacles with its **junctions** marked as blue nodes: this graph of junctions and skeleton branches *is* the roadmap,
- (d) the **distance transform**: for each free cell, the distance to the nearest obstacle, bright far from walls and dark near them, which is how the skeleton is computed (the ridges of this image).

> ⚠️ Slide 74 contains only the four pictures and no text. The description above is the standard textbook (Corke, Chapter 5) explanation of exactly these figures. Check with your lecturer how much of the skeleton/Voronoi detail is expected.

### Sampling-based solutions (slide 75)

Building a complete roadmap or searching every grid cell becomes impossible in high dimensions (a 6-joint arm has a 6-D configuration space). **Sampling-based** methods throw random points into the configuration space and connect the ones that work. The slide's three bullets:

- **Smaller search space**: you only look at a few hundred random configurations, not millions of cells.
- **No guarantee to find the goal**: if the random points miss a narrow corridor, no path is found even though one exists. (The technical term is *probabilistically complete*: the chance of finding a path goes to 1 as the number of samples grows, but for any finite number there is no guarantee.)
- **"Advanced guessing"**: that is exactly what it is, guessing cleverly and checking.

The two representatives are **PRM** (a graph) and **RRT** (a tree).

---

## 10. Probabilistic Roadmap Method (PRM) (slides 76–78)

PRM builds a roadmap from random samples. It has two phases.

**Phase 1: learning / planning phase (build the roadmap, done once).**

1. Pick a random point in the map. If it falls in an obstacle, throw it away; otherwise it becomes a node.
2. Repeat until you have $N$ nodes (`npoints`).
3. For each node, try to connect it to its neighbours within some distance. Draw the straight line; if the line crosses no occupied cell, add it as an edge.

The result is the tangle of black lines in slide 76. Panels (a) and (b) are the *same map with different random seeds*: the roadmaps look completely different, and that is normal for PRM.

**Phase 2: query phase (per start/goal pair, cheap).**

4. Connect the start and the goal to the nearest reachable node(s) of the roadmap.
5. Run a graph search (A* or Dijkstra) through the roadmap.
6. The path is a chain of straight segments between roadmap nodes (slide 77, the green dotted route with yellow nodes).

In MATLAB (from the `.m` file):

```matlab
load house
prm = PRM(house);                   % PRM planner on the house map
prm.plan('npoints', 200);           % phase 1: sample 200 free nodes and connect them
prm                                 % display: number of nodes, edges, components
prm.plot()                          % show the roadmap
prm.query(place.br3, place.kitchen);% phase 2: connect start/goal and search
prm.plot()                          % show the path
```

### How many nodes? (slide 78)

Slide 78 compares a 100-node and a 150-node roadmap on the house. With 100 nodes the roadmap splits into several disconnected **components** (coloured differently): some rooms are connected to each other but not to the rest. A query between two different components fails even though a door exists. With 150 nodes the components merge and the query br3 → kitchen succeeds (bottom picture, black path). More nodes means more connections and a higher chance of success, at the price of a longer learning phase. The *door* between two rooms is a narrow passage, and narrow passages are exactly what random sampling tends to miss.

> **Common mistake.** PRM paths are not optimal and not smooth: they zigzag between random nodes. In practice you post-process them (shortcutting or smoothing with the trajectory tools from Lecture 3).

---

## 11. Rapidly-Exploring Random Trees (RRT) (slides 79–88)

PRM builds a whole graph before answering anything. **RRT** grows a *tree* directly from the start, which makes it the natural choice when you have one start and want to explore outwards.

### The algorithm (slide 79)

1. **Create a root node at the robot position.**
2. **Guess a random point** in the map space (call it $q_{rand}$).
3. **Take a "step"** from the *closest node already in the tree* ($q_{near}$) towards the random point. The step has a fixed length, so you get a new candidate $q_{new}$ a short distance from $q_{near}$ in the direction of $q_{rand}$.
4. **Collision checks:** is the step-point occupied? Does the straight line from $q_{near}$ to $q_{new}$ pass through any occupied cells?
5. **If the checks pass, add $q_{new}$ to the tree** as a child of $q_{near}$.
6. **Repeat until the goal is found** (a new node lands close enough to the goal, or the goal itself can be connected).

Slide 80 draws this: the red tree grows from the robot at A; blue dots are random samples; the dashed red segments that hit obstacles are rejected; B is the goal region.

### Why "rapidly exploring"?

Because the random point is more likely to fall in a *large empty region*, and the tree always extends its *nearest* node towards it, the tree is automatically pulled into unexplored space. It does not clump near the start. Slides 81–83 show this on the small maze and on the house: with more iterations the blue tree fills the free space ever more densely (**"more iterations = tree expansion covers the search space better"**), and the red path from start to goal is found by tracing back from the node that reached the goal.

### Properties

- No grid, no pre-built roadmap: only the random samples and the tree.
- Like PRM, **no guarantee** for a finite number of samples, but the probability of success grows with the number of iterations.
- The path is jagged (it follows random tree edges) and usually needs smoothing.
- The basic version ignores kinematics; the kinematic version below fixes that.

### ERRT: combined exploration and planning (slides 84–86)

When the map is *unknown* (a mine, a collapsed building), there is no goal to plan to; the task is to *explore*. The LTU team's **ERRT** ("Exploration RRT") reuses the RRT idea:

1. **Grow a tree in the local safe space** (the part of the map the sensors have already confirmed free), slide 85 picture 1.
2. **Extract candidate branches**: each leaf of the tree is a *candidate goal* $g^c$, each root-to-leaf path a *candidate trajectory* $\chi_j$ (picture 2).
3. **Evaluate and select**: score every branch by how much new, unknown area the robot would see by flying it (the "predicted exploration", red dots in picture 3), and follow the *next-best trajectory* $\chi^*$.
4. Fly it, update the map, repeat.

Slide 84's sketch shows a tunnel junction with three unknown arms and candidate goals scattered towards each. Slide 86 is a video of this running in a simulated DARPA Subterranean Challenge world. The take-away: *RRT is not only a path planner, it is also a cheap way to generate many diverse options to choose between.*

### RRT with kinematic constraints (slides 87–88)

The Robotics Toolbox implementation of RRT plans for a **bicycle-model car**, so it works in the configuration space $q \in \mathbb R^2 \times S^1$, $q \sim (x, y, \theta)$ (position plus heading, the circle $S^1$ meaning that $\theta$ wraps around). From slide 87:

- The kinematics of the robot are taken into consideration.
- No grid: random nodes, as in PRM.
- The configuration is $(x, y, \theta)$.
- Constraints on the model are considered (max steering angle, fixed speed).
- A* or another graph method extracts the final path from the tree.

**Steps on planning (slide 87):**

1. Each node is a configuration $(x, y, \theta)$.
2. **The root node is the goal node.** (The toolbox grows the tree backwards from the goal, so that any start can later be connected; this is the same trick D* uses.)
3. A random configuration $q_{rand}$ is selected. The closest node in the tree, $q_{near}$, is found using a distance that mixes *position and orientation* (hence "optimisation f(distance and orientation)"). From $q_{near}$ the car model is driven forward for a fixed time with a few random steering inputs, and the trajectory ending closest to $q_{rand}$ is kept.
4. Its endpoint $q_{new}$ is reached and added to the graph, connected to $q_{near}$ by the drivable arc.

The right-hand picture on slide 87 shows these arcs fanning out from a node in $(x, y, \theta)$ space: red arcs for forward motion, blue for reverse. Every edge in the tree is a motion the car can actually perform, so the final path is drivable without any post-processing.

Slide 88 shows a full RRT for the bicycle model with velocity ±1 m/s, steering limits ±0.5 rad, an integration period of 1 s and initial configuration (0, 0, 0); each green dot is a node in the 3-D pose space $(x, y, \theta)$.

In MATLAB (from the `.m` file):

```matlab
car = Bicycle('steermax', 0.5);              % bicycle model, max steering 0.5 rad
rrt = RRT(car, 'npoints', 1000);             % RRT planner, 1000 nodes, obstacle-free
rrt.plan();                                  % grow the tree
rrt.plot();                                  % 3-D plot of the tree in (x, y, theta)

load road
rrt = RRT(car, road, 'npoints', 1000, 'root', [50 22 0], 'simtime', 4);
                                             % on the 'road' map, tree rooted at (50,22,0), 4 s per arc
rrt.plan();
p = rrt.query([40 45 0], [50 22 0]);         % path from (40,45,0) to the root/goal (50,22,0)
rrt.plot(p)
plot_vehicle(p, 'box', 'size', [10 15], 'fill', 'r', 'alpha', 0.1);   % animate the car along it
```

> **Common mistake.** Students often say "RRT finds the shortest path". It does not. Plain RRT finds *a feasible* path; the variant RRT* (not in these slides) rewires the tree to approach the shortest path as the number of samples grows.

### PRM vs RRT at a glance

| | **PRM** | **RRT** |
|---|---|---|
| Structure | Graph (roadmap) | Tree |
| Built | Once, then many queries | Per query (grows from one root) |
| Good for | Many start/goal pairs in a static map | Single query, exploration, kinematic constraints |
| Randomness | Random nodes, connect neighbours | Random targets, extend nearest node |
| Guarantee | None for finite samples (probabilistically complete) | Same |

---

## 12. The two-layer architecture in robotic motion (slide 89)

The final slide ties Part I and Part II together:

```
   map  ──────────►  PLANNING (low frequency)
                           │  sub-goal
   sensor data ──►  COLLISION AVOIDANCE (high frequency)
                           │  motion command
                         ROBOT
```

- The **planning layer** takes the map (and, occasionally, sensor data to update it) and produces a route, which it hands down as a **sub-goal** (the next waypoint). It runs at **low frequency**, say once a second or less, because A*, D*, PRM or RRT are expensive. This is Part II.
- The **collision-avoidance layer** takes the current sub-goal and the **live sensor data** and produces the actual **motion command** (speed and steering). It runs at **high frequency**, tens of times per second, so it can react to a person stepping in front of the robot. This is Part I: potential fields, Bug-style wall following, the Lindqvist APF.

Real robots are built exactly like this because it lets you use a slow, optimal, map-based brain and a fast, dumb, reactive reflex at the same time. If a chair that is not in the map appears, the reflex dodges it; if the chair blocks the whole corridor, the planner (D*!) eventually notices and re-routes.

---

## In a nutshell

Map-based planning starts by turning the world into an occupancy grid of free (0) and occupied (1) cells, which makes collision checking trivial and turns path planning into graph search. Dijkstra expands nodes by cost-so-far $g$ alone, while A* adds a heuristic estimate $h$ of the remaining cost and so reaches the goal with far fewer expansions; it is optimal only if $h$ never over-estimates and you stop when the goal is *popped* from the open list, not when it is first seen. D* plans backwards from a fixed goal to build a cost map and can repair that map cheaply when the robot discovers surprises, but the goal cannot change. Lattice planners replace the 8-neighbour hop with drivable arcs in $(x, y, \theta)$ so the result respects car kinematics. When grids get too big, sampling methods guess: PRM throws random nodes and connects them into a reusable roadmap, RRT grows a tree from a root towards random targets, and both trade guarantees for speed. Finally, a real robot stacks a slow map-based planner on top of a fast reactive collision-avoidance layer.

## Key terms

- **Occupancy grid**: a map made of equal cells, each 0 (free) or 1 (occupied); 3-D cells are voxels.
- **Resolution**: the size of one grid cell; finer is more accurate but slower.
- **Graph search**: finding a path through nodes connected by weighted edges.
- **8-connectivity**: a cell's neighbours are its 4 side cells and 4 diagonal cells.
- **Uninformed search**: expands in all directions with no knowledge of where the goal is (Dijkstra).
- **Informed search**: uses a heuristic to prefer promising nodes (A*, D*).
- **$g(x)$**: exact cost from the start to node $x$.
- **$h(x)$**: heuristic, estimated cost from $x$ to the goal.
- **$f(x) = g(x) + h(x)$**: A* priority; Dijkstra uses $f = g$.
- **Admissible heuristic**: one that never over-estimates the true remaining cost; needed for A* optimality.
- **Open list**: discovered but not yet expanded nodes (a priority queue on $f$).
- **Closed list**: nodes already expanded, never revisited.
- **Back-pointer / parent pointer**: the node from which a node was reached; followed backwards to read off the path.
- **Pruning**: discarding open nodes whose $f$ is already no better than the best complete path found.
- **Re-planning (online planning)**: plan, move, detect discrepancy, update map, plan again.
- **D\* (Dynamic A\*)**: plans from a fixed goal, stores a cost-to-goal map, repairs it incrementally on map changes.
- **Cost map**: for every cell, the cost of the cheapest path to the goal.
- **Lattice planner**: grid-based planner in $(x, y, \theta)$ using drivable motion primitives (straight, left arc, right arc) searched with A*.
- **Motion primitive**: a short, pre-computed, kinematically feasible movement.
- **Roadmap**: a graph of safe routes through free space, built once and queried many times.
- **Skeleton / distance transform**: the mid-way curves between obstacles / the per-cell distance to the nearest obstacle.
- **Sampling-based planning**: planning on random samples of the configuration space; "advanced guessing".
- **PRM (Probabilistic Roadmap)**: random free nodes connected by collision-free straight edges; learning phase then query phase.
- **RRT (Rapidly-exploring Random Tree)**: a tree grown from a root by stepping the nearest node towards random targets.
- **$q_{rand}, q_{near}, q_{new}$**: the random target, the nearest tree node, and the new node produced by stepping from $q_{near}$ towards $q_{rand}$.
- **ERRT**: an RRT variant that proposes many candidate branches and picks the one that explores the most unknown space.
- **Two-layer architecture**: low-frequency planning producing sub-goals, high-frequency collision avoidance producing motion commands.

## Check yourself

1. On an 8-connected grid with straight cost 10 and diagonal cost 14, a cell is reached from the start by two diagonal moves and one straight move, and it lies 4 columns and 2 rows away from the goal. Compute $g$, the Manhattan heuristic $h$ (scaled by 10), and $f$.
2. In the graph example, the first path to GOAL found had cost 5 via Start → A → E. Why did A* keep going, and which nodes could be pruned at that moment?
3. What does D* store that A* does not, and what is the one thing you are not allowed to change after planning with D*?
4. Give two reasons to prefer a lattice planner over plain A* on an occupancy grid for a car-like robot.
5. Describe one iteration of RRT in your own words, naming $q_{rand}$, $q_{near}$ and $q_{new}$, and state what "no guarantee to find the goal" means for PRM and RRT.

<details>
<summary>Answers</summary>

1. $g = 14 + 14 + 10 = 38$. Manhattan distance $= 4 + 2 = 6$, so $h = 60$. $f = 38 + 60 = 98$.

2. Because the goal was only *added* to the open list; it was not yet the node with the smallest $f$. Node C had $f = 4 < 5$, so a cheaper path through C might still exist (and it did: Start → C → K → GOAL, cost 4). With non-negative edge costs, any node whose $f$ is already $\ge 5$ cannot lead to a path cheaper than 5, so D(5), I(5), F(7), G(7) could be pruned, and later L(5) and J(5) as well.

3. D* stores a **cost map**: the cost of the cheapest path to the goal for *every* cell, computed by planning backwards from the goal. When the map changes (e.g. via `modify_cost`) it repairs only the affected entries instead of re-planning from scratch. The thing you cannot change is the **goal**; a new goal requires a completely new plan.

4. (a) Plain 8-connected A* produces paths with 45° and 90° corners that a car cannot follow because it cannot turn on the spot or move sideways; a lattice is built from arcs the car can actually drive, so the output is feasible by construction. (b) A lattice plans in $(x, y, \theta)$ and lets you put different costs on straight, left-turn and right-turn primitives (e.g. `[1 10 10]`), so you can discourage unnecessary wiggling, which a grid planner cannot express.

5. Pick a random configuration $q_{rand}$ anywhere in the map. Find the node already in the tree that is closest to it, $q_{near}$. Move a fixed step from $q_{near}$ towards $q_{rand}$ to get a candidate $q_{new}$. Check that $q_{new}$ is in free space and that the segment $q_{near} \to q_{new}$ crosses no occupied cells; if both hold, add $q_{new}$ to the tree as a child of $q_{near}$. Repeat until a node reaches the goal. "No guarantee" means that for any fixed, finite number of samples the random points may simply miss a necessary narrow passage, so the planner can report failure even though a path exists; the probability of success only tends to 1 as the number of samples grows (probabilistic completeness).

</details>
