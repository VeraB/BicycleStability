function test_suite = test_quaternion3d
% MUST BE IN THE DIRECTORY WHERE THE TEST RUNS.
initTestSuite;



function test_stabilitymargin
%theta 0  and frame length 5.5.
quat = [1 0 0 0];
tm = ThreeD(quat);
frameLength = 5.5;
btm = BicycleStability(tm,frameLength)
[H_f_to_h] = btm.getH_f_to_h ();
expected = [ 1 0 0 0; 0 1 0 0;0 0 1 5.5;0 0 0 1];
assertElementsAlmostEqual(expected,H_f_to_h);
q = [0 0 0 1]';
assertEqual([0 0 5.5 1]',H_f_to_h*q);

%Rotate on X
theta = pi/4;
H = rotx(theta);
com = ThreeD(H);
frameLength = 5.5;
com = com
btm = BicycleStability(com,frameLength)
[H_f_to_h] = btm.getH_f_to_h ();
q = [0 0 0 1]';
assertElementsAlmostEqual([0 5.5*sin(theta) 5.5*cos(theta) 1]',H_f_to_h*q);

%Rotate on X
theta = -pi/4;
H = rotx(theta);
com = ThreeD(H);
frameLength = 5.5;
com = com
btm = BicycleStability(com,frameLength)
[H_f_to_h] = btm.getH_f_to_h ();
q = [0 0 0 1]';
assertElementsAlmostEqual([0 -5.5*sin(-theta) 5.5*cos(theta) 1]',H_f_to_h*q);
q = [-1.7 0 0 1]';
assertElementsAlmostEqual([ -1.7000 -5.5*sin(-theta) 5.5*cos(theta) 1]',H_f_to_h*q);

%Rotate on Y
theta = -pi/4;
H = roty(theta);
com = ThreeD(H);
frameLength = 5.5;
btm = BicycleStability(com,frameLength)
[H_f_to_h] = btm.getH_f_to_h ();
expected = [ 1 0 0 0; 0 1 0 0;0 0 1 5.5;0 0 0 1];
assertElementsAlmostEqual(expected,H_f_to_h);
q = [0 0 0 1]';
assertEqual([0 0 5.5 1]',H_f_to_h*q);

%Rotate on X an Y
theta = -pi/4;
H = rotx(theta)*roty(pi/6.2);
com = ThreeD(H);
frameLength = 5.5;
com = com
btm = BicycleStability(com,frameLength)
[H_f_to_h] = btm.getH_f_to_h ();
q = [0 0 0 1]';
assertElementsAlmostEqual([0 -5.5*sin(-theta) 5.5*cos(theta) 1]',H_f_to_h*q);
q = [-1.7 0 0 1]';
assertElementsAlmostEqual([ -1.7000 -5.5*sin(-theta) 5.5*cos(theta) 1]',H_f_to_h*q);

%Rotate on X an Z
theta = -pi/4;
H = rotx(theta)*rotz(pi/6.4);
com = ThreeD(H);
frameLength = 5.5;
com = com
btm = BicycleStability(com,frameLength)
[H_f_to_h] = btm.getH_f_to_h ();
q = [0 0 0 1]';
assertElementsAlmostEqual([0 -5.5*sin(-theta) 5.5*cos(theta) 1]',H_f_to_h*q);
q = [-1.7 0 0 1]';
assertElementsAlmostEqual([ -1.7000 -5.5*sin(-theta) 5.5*cos(theta) 1]',H_f_to_h*q);