%% Matlab code examples from Chapter 2 of the "Robotics, Vision and Control" book
%%
clear all
clc
%% p24-25 (2.1.1.1)
R = rot2(0.2)                           %%%rotation matrix R with the angle specified in radians
%%
det(R)                                  %%% property of R is the det equals to 1 
%%
det(R*R)                                %%% the product of rotation matrix multiplication is also a rotation matrix
%%
syms theta                              %%% the toolbox also supports symbolic mathematics
%%
R = rot2(theta)                         %%% symbolic mathematics example (instead of numerical value in rad)
%%
R*R
simplify(R*R)                           %%% simplify the presented results
%% 
det(R)
simplify(det(R))                        %%% simplify the presented results
%% p24-25 (2.1.1.2)
%%
R = rot2(0.3)                           %%% rotation matrix for 0.3 rads
%%
S = logm(R)                             %%% compute the logarithm of this matrix using the matlab function logm. The result is a simple matrix with two elements 
                                        %%% having a magnitude of 0.3,
                                        %%% which intriguingly is the original rotation angle
%%
vex(S)                                  %%% This matrix has a zero diagonal and is an example of a 2 × 2 skew-symmetric matrix
                                        %%% The matrix has only one unique element and we can unpack it using the Toolbox function vex
%%
expm(S)                                 %%% The inverse of a logarithm is exponentiation and using the builtin MATLAB matrix exponential function expm
%%
R = rot2(0.3)                           %%% original rotation matrix
%%
R = expm(skew(0.3))                 %%%  original rotation matrix is equivalent to  this command. Check the rotation matrix definition with the exponential

%% p27-29 (2.1.2.1) &  (2.1.2.2)
clear all
clc
%%
T1 = transl2(1, 2) * trot2(30, 'deg')   %%% we create a homogeneous transformation which represents a translation of (1, 2) followed by a rotation of 30°.
                                        %%% The function transl2 creates a relative pose with a finite translation but zero rotation, while trot2 creates
                                        %%% a relative pose with a finite rotation but zero translation.
%%
plotvol([0 5 0 5]);                     %%% plot relative to the world coordinate frame
%%
%%
trplot2(T1, 'frame', '1', 'color', 'b')
%%
T2 = transl2(2, 1)                       %%% we create another relative pose which is represents a displacement of (2, 1) and zero rotation
%%
trplot2(T2, 'frame', '2', 'color', 'r');
%%
T3 = T1*T2                               %%% we can compose the two relative poses
%%
trplot2(T3, 'frame', '3', 'color', 'g'); %%% We see that the displacement of (2, 1) has been applied with respect to frame {1}. It is
                                         %%% important to note that our final displacement is not (3, 3) because the displacement
                                         %%% is with respect to the rotated coordinate frame.
%%
T4 = T2*T1;                              %%% noncommutativity of composition is demonstrated
%%
trplot2(T4, 'frame', '4', 'color', 'c'); %%% and we see that frame {4} is different to frame {3}.
%%
P = [3 ; 2 ];                            %%% Now we define a point (3, 2) relative to the world frame which is a column vector
%%
plot_point(P, 'label', 'P', 'solid', 'ko');
%%
P1 = inv(T1) * [P; 1]                    %%% determine the coordinate of the point with respect to frame {1}
                                         %%% where we first converted the Euclidean point coordinates to homogeneous form by appending
                                         %%% a one on vector. The result is also in homogeneous form and has a negative y-coordinate in frame {1}
%%
h2e( inv(T1) * e2h(P) )                  %%% Using the Toolbox we could also have expressed this as
%%
clf                                      %%% First we create and plot a reference coordinate frame {0} and a target frame {X}
%%
plotvol([-5 4 -1 4.5]);
%%
T0 = eye(3,3)                           %%% define frame 0
%%
trplot2(T0, 'frame', '0');              %%% plot frame 0
%%
X = transl2(2, 3)                       %%% define frame X
%%
trplot2(X, 'frame', 'X');               %%% plot frame X
%%
R = trot2(2);                           %%% rotation of 2 radians (approximately 115°)
%%
trplot2(R*X, 'framelabel', 'RX', 'color', 'r');  %%% plot the effect of the two possible orders of composition (1)
%%
trplot2(X*R, 'framelabel', 'XR', 'color', 'r');  %%% plot the effect of the two possible orders of composition (2)
                                                 %%% We see that the frame {RX} has been rotated about the origin, while frame {XR} has been rotated about the origin of {X}
%%
C = [1 2]';                                      %%% rotate a coordinate frame about an arbitrary point. Establish a new point C
%%
plot_point(C, 'label', ' C', 'textcolor', 'k', 'solid', 'ko') %%% display the point
%%
RC = transl2(C) * R * transl2(-C)                %%% then compute a transform to rotate about point C
%%
trplot2(RC*X, 'framelabel', 'XC', 'color', 'r'); %%% display the result
%% p34-37 (2.2.1.1) 
%%
clear all
clc
%%
R = rotx(pi/2)                                   %%%rotation matrix around x axis for 45 degrees
%%
clf
%%
trplot(R)                                        %%% coordinate frame can be displayed graphically
%%
tranimate(R)                                     %%% animate a rotation using the Toolbox function tranimate
%%
R = rotx(pi/2) * roty(pi/2)                      %%% rotate previous frame around y axis
%%
roty(pi/2)*rotx(pi/2)                            %%% The noncommutativity of rotation can be shown by reversing the order of the rotations above
%%
R = rotz(0.1) * roty(0.2) * rotz(0.3)            %%% ZYZ sequence for Euler rotation 
%%
R = eul2r(0.1, 0.2, 0.3)                         %%% same as previous command but more compact form
%%
gamma = tr2eul(R)                                %%% The inverse problem is finding the Euler angles that correspond to a given rotation matrix
%%
R = eul2r(0.1 , -0.2, 0.3)                       %%% if theta negative
%%
tr2eul(R)                                        %%% the inverse function returns a positive value for theta and different values for phi and psi
%%
eul2r(ans)                                       %%% however the rotation matrix is the same. The two different sets of Euler angles correspond to the one rotation matrix.
                                                 %%% The mapping from a rotation matrix to Euler angles is not unique and the Toolbox always returns a positive value for theta
%%
R = eul2r(0.1, 0, 0.3)                           %%% for theta=0
%%
tr2eul(R)                                        %%% the inverse function returns different angles. 
                                                 %%% The rotation matrix is a function of the sum of phi and psi angles. Therefore the inverse operation can do no more
                                                 %%% than determine this sum, and by convention we choose phi = 0. The case theta = 0 is a singularity
%% p38 
clear all
clc
%%
R = rpy2r(0.1, 0.2, 0.3)                        %%% When describing the attitude of a robot gripper, the convention
                                                %%% is that the z-axis points forward and the x-axis is either up or down. This leads to the XYZ angle sequence
                                                %%% ZYX sequence is the default for the toolbox
%%
gamma = tr2rpy(R)                               %%% inverse calculation (from rotation matrix to rpy angles). The roll-pitch-yaw sequence allows all angles to have
                                                %%% arbitrary sign and it has a singularity when theta=±pi/2 which is fortunately outside the range of feasible 
                                                %%% attitudes for most vehicles.
%%
tripleangle_App                                 %%% interactive graphical tool
%% p45 (2.2.1.7)
clear all
clc
%%
q = UnitQuaternion( rpy2tr(0.1, 0.2, 0.3)  )    %%% In the Toolbox quaternions are implemented by the UnitQuaternion class and the
                                                %%% constructor converts a passed argument such as a rotation matrix to a unit quaternion
%%
q = q * q                                       %%% Quaternion multiplication is invoked through the overloaded multiplication operator
%%
inv(q)                                          %%% inverse opearation. The conjugate of a unit quaternion
%%
q*inv(q)                                        %%% Multiplying a quaternion by its inverse yields the identity quaternion
%%
q/q                                             %%% which represents a null rotation
%%
q.R                                             %%% The quaternion can be converted to an orthonormal rotation matrix
%%
q.plot()                                        %%% and we can also plot the orientation represented by a quaternion
%%
q*[1 0 0]'                                      %%% A vector is rotated by a quaternion using the overloaded multiplication operator
%% 47 (2.2.2.1)
clear all
clc
%%
T = transl(1, 0, 0) * trotx(pi/2) * transl(0, 1, 0)         %%% The Toolbox has many functions to create homogeneous transformations
                                                            %%% The function transl creates a relative pose with a finite translation but no rotation,
                                                            %%% while trotx creates a relative pose corresponding to a rotation of pi/2 about the x-axis with zero translation.
                                                            %%% The result, as shown in the last column of the resulting matrix is a translation of 1 unit along the original x-axis and
                                                            %%% 1 unit along the original z-axis. The orientation of the fi nal pose shows the effect of the rotation about the x-axis.
%%
trplot(T)                                                   %%% plot
%%
t2r(T)                                                      %%% rotation matrix component of T
%%
transl(T)                                                   %%% translation column vector component of T