% this is the code to let the robot learn the usage of tool in a
% exploration way. 
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% exploration action in order to estimate the normal direction
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;

%load robot model
kuka_robot = loadrobot('kukalwr');
%visualization of kuka_lwr at the initialized pose
Q = rand(1,7);
%the robot end-effector frame at the initialized status
T_robot_end_eff_init = kuka_robot.fkine(Q);
link_value = rand(3,1);
rot_value = [0.2,0.5,0.3];
tool_rotate = trotz(rot_value(3))*troty(rot_value(2))*trotx(rot_value(1));
tool_translate = transl(link_value);
tool_transform = tool_rotate * tool_translate ;
%the tactool end-effector frame after the transformation(translationa nd rotation
% from the robot end-effector frame)at the initialized status
T_tool_end_eff_init = T_robot_end_eff_init*tool_transform;
T_tool_end_eff_init(1:3,3)
% kuka_robot.plot(Q,'workspace',[-3, 3 -3, 3 -3, 3]);
hold on;
trplot(T_tool_end_eff_init, 'frame', 'R','length',0.02,'width',0.01);
hold on;
%analog virtual tangent surface, assume that only the normal direction is
%estimated and given.
virtual_angle = 6;
rot_tm = rpy2tr(0,0,virtual_angle,'deg');
T_tool_end_eff_init_virtual = T_tool_end_eff_init*rot_tm;
%analog the noised tactool frame
virtual_x = 26;
virtual_y = 26;
virtual_z = 26;
rot_tm = rpy2tr(virtual_x,virtual_y,virtual_z,'deg');
T_tool_end_eff_init_noise = T_tool_end_eff_init*rot_tm;
% trplot(T_tool_end_eff_init_noise, 'frame', 'N');
T_tool_end_eff_init_noise(1:3,3)

%draw contact ball
cx = -0.04 + 0.08*rand; %initialized contact point in the local x
cy = -0.04 + 0.08*rand; %initialized contact point in the local y
c = T_tool_end_eff_init*[cx,cy,0,1]';
sphere_r = 0.002;
drawsphere(c(1:3),sphere_r);

%estimate normal direction using the initialized nv guess from the
%approaching trajectory
n_hat = est_nv_tac(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise)

% estimate rotate angle from the virtual tool frame to real tool frame
rotate_angle = est_rotate(virtual_angle);

%estimate translation from robot end-effector to tool end-effector
est_trans = est_translation(kuka_robot,Q,tool_transform,tool_rotate,link_value);


