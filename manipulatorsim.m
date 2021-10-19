clc
clear
close all
%% DH- parameter of the manipulator is 

dh_parameter = [0.3 0 0 0; 0.31 0 0 0];

%% Planar arm under study skeleton is 
planar_arm = rigidBodyTree('DataFormat','row');

%% link1 of the arm is
link_1 = rigidBody('base1');
rb_joint = rigidBodyJoint('joint1','revolute');
setFixedTransform(rb_joint,dh_parameter(1,:),"dh");
link_1.Joint = rb_joint;
addBody(planar_arm,link_1,'base');

%% link 2 of the arm is
link_2 = rigidBody('link1');
r_joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(r_joint,dh_parameter(2,:),"dh");
link_2.Joint = r_joint;
addBody(planar_arm,link_2,'base1');
link_3 = rigidBody('link2');
addBody(planar_arm,link_3,'link1');
source = 'link1';
ee = 'link2';

jointAnglePosition = [pi;pi];
jointAngleRetract = [pi;pi];

%% path and trajectory definition
%% waypoints for the manipulator. In this the straight line is consider as the path therefore the waypoints are
interact = interactiveRigidBodyTree(planar_arm);
ax = gca;
plane = collisionBox(1,0,1);
plane.Pose = trvec2tform([0 -0.025 0 ]);
show(plane,'Parent', ax);

%% for configuration of the manipulator arm in the space
interact.MarkerBodyName = "link2";
%% but in this case the arm is placed in the homeconfiguration in initial condition


randConfig = planar_arm.randomConfiguration;
% tranfromation of end effector w.r.t base
tform = getTransform(planar_arm,randConfig,'link2','base');


home = planar_arm.homeConfiguration;
%homef = planar_arm.homeConfiguration;
n = numel(home);
%% Linear path is selected to complete the application therefore the waypoint of the path are
% position of the end effector
waypointsmax =20;
waypoints = [0 0.5 0;... % start point 
0.05 0.5 0;
0.10 0.5 0;
0.15 0.5 0;
0.20 0.5 0;
0.25 0.5 0;
0.30 0.5 0;
0.35 0.5 0;
0.35 0.35 0;
0.25 0.35 0]';

numWaypoints = size(waypoints);
 %orientation(these values are selected randomly)   
orientation = [0 0 0;...
    pi/2 pi/2 0;
    pi pi/2 0]';

% time for each waypoint
t_f = 18000;
waytime = 0:2000:t_f;

%% trajectory parameters

t = 0.2;
trajectoryT = 0:t:waytime(end);
tn = numel(trajectoryT);
%boundary conditions
waypointVels = 0.002 *[ 0  0.05  0;0.05 0 0;
                     0.05  0  0;
                     0.05  0  0;
                     0.05  0  0;
                      0.05 0.05 0 ;
                     0.05 0.05 0;
                      0.05 0.05 0;
                      0.05 0.05 0;
                      0.05 0.05 0]';
           

 weights = [0.5 0.5 0.5 1 1 1];

out =  sim('test_link_with_main.slx');

figure;
plot(out.position.signals.values(:,1),out.position.signals.values(:,2),'r');
