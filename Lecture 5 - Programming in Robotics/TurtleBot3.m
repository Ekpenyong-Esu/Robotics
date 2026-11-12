clear all
clc

%% set ROS Domain ID
setenv("ROS_DOMAIN_ID","39")

%% check ros topics
ros2 topic list
%% Creat Node
disp(" ")
testNode = ros2node("/test_node");
ros2 node list 
%% Subscribe to odmo
odomSub = ros2subscriber(testNode,"/odom")
scanSub = ros2subscriber(testNode,"/scan")

%% Read odom
odomData = receive(odomSub);
odomData.pose.pose.position
%odomData.pose.pose.orientation

%% Creat ros2 velocity publisher
velocityPublisher = ros2publisher(testNode, '/cmd_vel')

%% Publish velocity for 1 sec
velmsg = ros2message(velocityPublisher);
velmsg.linear.x = 0.1; % meters per second

send(velocityPublisher,velmsg);
pause(1)

velmsg.linear.x = 0.0;
send(velocityPublisher,velmsg);


%% plot LIDAR information for 10 seconds
tic
figure(1)
while toc<10
    scan = receive(scanSub);
    points = rosReadCartesian(scan);
    plot(points(:,1),points(:,2),'rO')
    axis image
end

%% Moving forward and plot LIDAR for 30 second
velmsg = ros2message(velocityPublisher);
velmsg.linear.x = 0.1; % meters per second

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