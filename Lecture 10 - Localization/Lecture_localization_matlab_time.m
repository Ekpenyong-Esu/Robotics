clear all
clc
close all
%% 6.1.1
%%
V = diag([0.02, 0.5*pi/180].^2);
%%
veh = Bicycle('covar', V)
%%
randinit
%%
odo = veh.step(1, 0.3)
%%
veh.x'
%%
veh.f([0 0 0], odo)
%%
veh.add_driver( RandomPath(10) )
%%
veh.run()
%% 6.1.2
%%
veh.Fx( [0,0,0], [0.5, 0.1] )
%%
P0 = diag([0.005, 0.005, 0.001].^2);
%%
ekf = EKF(veh, V, P0);
%%
randinit
%%
ekf.run(1000);
%%
clf
veh.plot_xy()
%%
hold on
ekf.plot_xy('r')
%%
P700 = ekf.history(700).P
%%
sqrt(P700(1,1))
%%
ekf.plot_ellipse('g')
%%
ekf.plot_P();
%% 6.2

randinit
%%
map = LandmarkMap(20, 10);
%%
map.plot()
%%
W = diag([0.1, 1*pi/180].^2);
%%
sensor = RangeBearingSensor(veh, map, 'covar', W)
%%
[z,i] = sensor.reading()
%%
map.landmark(17)
%%
randinit
map = LandmarkMap(20);
veh = Bicycle('covar', V);
veh.add_driver( RandomPath(map.dim) );
sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', ...
[-pi/2 pi/2], 'range', 4, 'animate');
ekf = EKF(veh, V, P0, sensor, W, map);
%%
ekf.run(1000);
%%
map.plot()
%%
veh.plot_xy();
%%
ekf.plot_xy('r');
%%
ekf.plot_ellipse('k')
%%
clf
ekf.plot_P()
xlabel('Time step'); ylabel('(det P)^{0.5}'); grid
%%
clf
ekf.plot_error('confidence', 0.95, 'nplots', 4)
plot_poly([1:1000; sensor.landmarklog], 'fill', 'b')
ylabel('land marks')
grid


%% 6.3

randinit
map = LandmarkMap(20);
veh = Bicycle(); % error free vehicle
veh.add_driver( RandomPath(map.dim) );
W = diag([0.1, 1*pi/180].^2);
sensor = RangeBearingSensor(veh, map, 'covar', W, 'animate');
ekf = EKF(veh, [], [], sensor, W, []);
%%
ekf.run(10);
%%
map.plot();
%%
ekf.plot_map('g');
%%
veh.plot_xy('b');
%%
ekf.landmarks(:,6)
%%
ekf.x_est(19:20)'
%%
ekf.P_est(19:20,19:20)
%% 6.4

randinit
P0 = diag([.01, .01, 0.005].^2);
map = LandmarkMap(20);
veh = Bicycle('covar', V);
veh.add_driver( RandomPath(map.dim) );
sensor = RangeBearingSensor(veh, map, 'covar', W, 'animate');
ekf = EKF(veh, V, P0, sensor, W, []);
%%
ekf.run(10);
%%
map.plot();
%%
ekf.plot_map('g');
%%
ekf.plot_xy('r');
%%
veh.plot_xy('b');


%% 6.5
syms x_i y_i theta_i x_j y_j theta_j x_m y_m theta_m assume real
xi_e = inv( SE2(x_m, y_m, theta_m) ) * inv( SE2(x_i, y_i, theta_i) ) * SE2(x_j, y_j, theta_j);
fk = simplify(xi_e.xyt);
%%
jacobian ( fk, [x_i y_i theta_i] );
Ai = simplify (ans)
%%
pg = PoseGraph('pg1.g2o')
%%
clf
pg.plot()
%%
pg.optimize('animate')
%%

pg = PoseGraph('killian-small.toro');
%%
pg.plot()
%%
%%
clf
%%
pg.optimize('animate')
%%
pg.plot()
%% 6.8

pg = PoseGraph('killian.g2o', 'laser');
%%
[r, theta] = pg.scan(2580);
about r theta
%%
clf
polar(theta, r)
%%
[x,y] = pol2cart (theta, r);
plot (x, y, '.')
%%
p2580 = pg.scanxy(2580);
p2581 = pg.scanxy(2581);
about p2580
%%
T = icp( p2581, p2580, 'verbose' , 'T0', transl2(0.5, 0), 'distthresh', 3)
%%
pg.time(2581)-pg.time(2580)
%%
map = pg.scanmap();
%%
pg.plot_occgrid(map)
