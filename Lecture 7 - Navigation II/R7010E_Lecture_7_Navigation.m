%% clear the workspace variables and close all figures


clear all
clc
close all
%% 
sl_braitenberg
%% Reactive field
xc=50;, yc=50;
[X,Y]=meshgrid(1:1:100,1:1:100);
sens=200./((X-xc).^2 + (Y-yc).^2 +200);
surf(X,Y,sens)

%% Bug
load map1
bug = Bug2(map);
bug.plot( )
start = [20,10]; 
goal = [50,35];
bug.query(start, goal, 'animate');   % animate path
bug.display
%% MAP of the book for bug
load house
place
bug = Bug2(house);
bug.plot();
bug.query(place.br3, place.kitchen, 'animate');
% Output argument
p = bug.query(place.br3, place.kitchen)

%% D*
%LAB
load map1
ds=Dstar(map);
ds.plot
c = ds.costmap();
ds.plot
start = [20,10]; 
goal = [50,35];
ds.plan(goal) 
p=ds.query(start, 'animate');
ds.plot(p)

%% D*
%Book
load house
ds=Dstar(house);
ds.plot
c = ds.costmap();
ds.plot
ds.plan(place.kitchen, 'animate');
p=ds.query(place.br3);
ds.plot(p)
ds.niter
%% D* Increase cost
ds.modify_cost([300,325;115,125],50)
ds.plan
p=ds.query(place.br3);
ds.plot(p)
ds.niter

%% PRM 
load house
prm = PRM(house)
prm.plan('npoints', 200)
prm
prm.plot()
prm.query(place.br3, place.kitchen)
prm.plot()

%% Lattice Planner
lp = Lattice();
lp.plan('iterations', 2)
lp.plot()
%% Lattice Planner
lp.plan('iterations', 8)
lp.plot()
lp.query( [1 2 pi/2], [2 -2 0] );
lp.plot

%% Lattice Planner Cost
lp = Lattice();
lp.plan('cost', [1 10 10],'iterations', 8)  %straight ahead, turn left and turn right
lp.query([1 2 pi/2], [2 -2 0]);
lp.plot()

%% Load road
load road
lp = Lattice(road, 'grid', 5, 'root', [50 50 0])
lp.plan();
lp.query([30 45 0], [50 20 0])
lp.plot()

%% RRT
car = Bicycle('steermax', 0.5);
rrt = RRT(car, 'npoints', 1000)
rrt.plan();
rrt.plot();

%% RRT
% rrt = RRT(car,  'npoints', 1000, 'root', [50 22 0], 'simtime', 4)
% rrt.plan();
% p = rrt.query([40 45 0], [50 22 0]);
% rrt.plot(p)
% plot_vehicle(p, 'box', 'size', [20 30], 'fill', 'r', 'alpha', 0.1)
load map1
rrt = RRT(car, road, 'npoints', 1000, 'root', [50 22 0],'simtime', 4)
rrt.plan();
p = rrt.query([40 45 0], [50 22 0]);
rrt.plot(p)
plot_vehicle(p, 'box', 'size', [10 15], 'fill', 'r', 'alpha', 0.1)


