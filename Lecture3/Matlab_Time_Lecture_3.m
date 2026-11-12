%% matlab time
clear all
clc
%% p65 (3.1.2)
TB = SE3(1, 2, 0) * SE3.Rz(pi/2)           %%%define fixed body frame {B} Rotate pi/2 align z axis with 1 2 0 translation
%%
vb = [0.2 0.3 0 0 0 0.5]'                  %%%define spatial velocity in {B} frame
%%
va = TB.velxform * vb                      %%%transform spatial velocity in the world frame
%% 
TBC = SE3(0, 0.4, 0)                       %%%define fixed body frame {C} tranlation of 0.4 align y compare to B frame
                                              %%% additional frame {C} relative to frame {B}
%%
TCB = inv(TBC)                             %%%compute inverse transform of {C}
%%
vc = TCB.Ad * vb                           %%%transform spatial velocity in the {C} frame
%% p66 (3.1.3)
rotx(0.001)                                 %%%infinitesimally small rotation can be approximated by the sum of
                                            %%%a skew-symmetric matrix and identity matrix
%% p68 (3.2.1)
%%
J = [2 -1 0;-1 4 0;0 0 3];                  %%%define inertia matrix of tunbling object
%%
attitude = UnitQuaternion();                %%%initial condition for orientation
%%
w = 0.2*[1 2 2]';                           %%%initial condition for angular velocity
%%
dt = 0.05;
%%
h = attitude.plot();
%%
for t=0:dt:10                                                              %%%compute angular acceleration through 
   wd =  -inv(J) * (cross(w, J*w));                                        %%%rectangular integration              
   w = w + wd*dt;  
   attitude = attitude .* UnitQuaternion.omega(wd*dt);
   attitude.plot('handle', h); pause(dt)
end

%% p68 (3.2.2)
%%
WB = [3 4 0 0 0 0]'                         %%%define wrench in frame {B} with force 3 and 4 Nm
%%
WC = TBC.Ad' * WB;                          %%%transform wrench in frame {C}
%% p71-73 (3.3.1)
clear all
clc
%% 
tpoly(0, 1, 50);                            %%%generate a quintic polynomial trajectory
%% 
[s,sd,sdd] = tpoly(0, 1, 50)               %%%trajectory-s, velocity-sd and acceleration-sdd
%%
tpoly(0, 1, 50, 0.5, 0);                    %%%define initial velocity (0.5) and final velocity  (0)
%%
mean(sd) / max(sd)                          %%%mean velocity
%%
lspb(0, 1, 50);                             %%%linear segment (constant velocity) with parabolic blends
                                            %%%%%%% constant velocity segment with polynomial segments for acceleration and deceleration
                                            %%%% The trapezoidal trajectory is smooth in velocity, but not in acceleration
                                            %%%%% using a constant velocity segment and parabolic blends (a trapezoidal path)
%%
[s,sd,sdd] = lspb(0, 1, 50);                %%%trajectory-s, velocity-sd and acceleration-sdd
%%
max(sd)                                     %%%lspb defined velocity
%%
s = lspb(0, 1, 50, 0.025);
%%
s = lspb(0, 1, 50, 0.035);

%%%%% We see that as the velocity of the linear segment increases its duration decreases and ultimately its duration would be zero. In fact the velocity cannot be chosen arbitrarily, too high or
%%%%% too low a value for the maximum velocity will result in an infeasible trajectory and the function returns an error
%% p73 (3.3.2)
clf
%%
q = mtraj(@lspb, [0 2], [1 -1], 50);        %%%generates a multi-segment multi-axis trajectory
                                            %%%based on a matrix of via points
%%
subplot(1,2,1),plot(q)
subplot(1,2,2),plot(q(:,1),q(:,2))
%%
% OR
path=[ 20 30 ;...
       20 60 ;...
       50 60 ;...
       50 90 ;...
       90 90 ;...
       90 10 ;...
       20 10 ;...
    ];
% Genrate trajectory
QDMAX=[1,1]; %axis speed limits
Q0= path(1,:); %initial axis coordinates
DT=0.2;
traj = mstraj(path, QDMAX, [], Q0, DT, 0);
subplot(1,2,1),plot(traj)
subplot(1,2,2),plot(traj(:,1),traj(:,2))

%%%% The motion comprises linear motion segments with polynomial blends, like lspb, but here we choose quintic polynomials because they are able to match boundary conditions on position, velocity 
%%%% and acceleration at their start and end points

%%
% q = [T1.t' T1.torpy]                      %%%create a trajectory for 3-dimensional pose, converting pose T 
                                            %%%to a 6-vector
%% p74-75 (3.3.3)
clf
%%%%
%On a particular motion segment each axis will have a different distance to travel and
% traveling at its maximum speed there will be a minimum time before it can reach its
% goal. The first step in planning a segment is to determine which axis will be the slowest to complete the segment, based on the distance that each axis needs to travel for
% the segment and its maximum achievable velocity. From this the duration of the segment can be computed and then the required velocity of each axis. This ensures that
% all axes reach the next target qk at the same time
%%
via = SO2(30, 'deg') * [-1 1; 1 1; 1 -1; -1 -1]';               %%%generate via points
%%
q0 = mstraj(via(:,[2 3 4 1])', [2,1], [], via(:,1)', 0.2, 0);   %%%1st arg - via points
                                                                %%%2nd arg - vector of maximum speeds per axis
                                                                %%%3rd arg - vector of duration for each segment
                                                                %%%4rth arg - initial configuration
                                                                %%%5th arg- sample time step
                                                                %%%6th arg - acceleration time
%%
plot(q0(:,1), q0(:,2))
%% 
q2 = mstraj(via(:,[2 3 4 1])', [2,1], [], via(:,1)', 0.2, 2);   %%%increase acceleration time. Trajerctory becomes rounded
hold on
size(q2)
plot(q2(:,1), q2(:,2))
%% Keep in mind that this function simply interpolates pose represented as a vector.
%%
[numrows(q0) numrows(q2)]
%% p76 (3.3.4)
R0 = SO3.Rz(-1) * SO3.Ry(-1)                                   %%%define rotation R0
%%  
R1 = SO3.Rz(1) * SO3.Ry(1)                                    %%%define rotation R1
%%
rpy0 = R0.torpy(); rpy1 = R1.torpy();                           %%%calculate rpy angles for R0 and R1
%%
rpy = mtraj(@tpoly, rpy0, rpy1, 50);                            %%%create 50 steps trajectory between rpy0 and rpy1
%%
SO3.rpy( rpy ).animate;                                         %%%visualize animation
%%
q0 = R0. UnitQuaternion;                                        %%%define quaternion q0
%%
q1 = R1.UnitQuaternion;                                         %%%define quaternion q1
%%
q = interp(q0, q1, 50);                                         %%%interpolate q0 and q1
%%
about(q)
%%
clf
%%
q.animate()                                                     %%%animate
%% p76 (3.3.4.1)
q0 = UnitQuaternion.Rz(-2);                                     %%%create rotation around z axis of -2 rads
%%
q1 = UnitQuaternion.Rz(2);                                      %%%create rotation around z axis of +2 rads
%%
q = interp(q0, q1, 50);                                         %%%interpolate
%%
q.animate()
%%
q = interp(q0, q1, 50, 'shortest');                             %%%select shortest path
%%
q.animate()
%% p77 (3.3.6)
clear all
clc
%%
T0 = SE3([0.4, 0.2, 0]) * SE3.rpy(0, 0, 3);                     %%%define intial pose T0
%%
T1 = SE3([-0.4, -0.2, 0.3]) * SE3.rpy(-pi/4, pi/4, -pi/2);      %%%define final pose T1
%%
interp(T0, T1, 0.5)                                             %%%interpolate
%%
Ts = interp(T0, T1, 50);
%%
about(Ts)
%%
Ts(1)
%%
Ts.animate
%%
P = Ts.transl;
%%
about(P)
%%
plot(P)
%%
rpy = Ts. torpy;                                                %%%extract rpy angles of trajectory
%%
plot(rpy);
%%
Ts = T0. interp(T1, lspb(0,1, 50) );                            %%%the coordinate frame now accelerates to a
                                                                %%%constant speed along the path and then decelerates
                                                                %%%and this is refl ected in smoother curve for the trajectory
%%
Ts = ctraj(T0, T1, 50);                                         %%%compact form of the above