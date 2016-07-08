% this is the code to let the robot learn the usage of tool in a
% exploration way. 
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;

%load robot model and calculate the robot model kinematics
kuka_robot = loadrobot('kukalwr');
%visualization of kuka_lwr at the initialized pose
Q = rand(1,7);
%the robot end-effector frame at the initialized status
T_robot_end_eff_init = kuka_robot.fkine(Q);
link_value = rand(3,1)
link_value(1) = 0.3;
link_value(2) = 0.5;
link_value(3) = 0.9;
rot_value = [0.2,0.5,0.3];
tool_rotate = trotz(rot_value(3))*troty(rot_value(2))*trotx(rot_value(1));
tool_translate = transl(link_value);
%must firstly translation then rotation, we can get the estimation of
%translation is same with link_value
tool_transform = tool_translate * tool_rotate;
%the tactool end-effector frame after the transformation(translationa nd rotation
% from the robot end-effector frame)at the initialized status
T_tool_end_eff_init = T_robot_end_eff_init*tool_transform;
disp('real n_hat');
T_tool_end_eff_init(1:3,3)
% kuka_robot.plot(Q,'workspace',[-3, 3 -3, 3 -3, 3]);
hold on;
% trplot(T_tool_end_eff_init, 'frame', 'R','length',0.02,'width',0.01);
hold on;


%%%%%%%%%%%%%%%%%%%%%%Obsolete function%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %analog virtual tangent surface, assume that only the normal direction is
% %estimated and given.
% virtual_angle = 6;
% rot_tm = rpy2tr(0,0,virtual_angle,'deg');
% T_tool_end_eff_init_virtual = T_tool_end_eff_init*rot_tm;


%analog the noised tactool frame, all euler angles of rotation matrix can be
%selected from (-90deg - 90deg)
v_low = -40;
v_high = 40;
virtual_x = v_low+(v_high-v_low)*rand;
virtual_y = v_low+(v_high-v_low)*rand;
virtual_z = v_low+(v_high-v_low)*rand;
rot_tm = rpy2tr(virtual_x,virtual_y,virtual_z,'deg');
T_tool_end_eff_init_noise = T_tool_end_eff_init*rot_tm;
% trplot(T_tool_end_eff_init_noise, 'frame', 'N');
% disp('noised n_hat');
% T_tool_end_eff_init_noise(1:3,3)

%draw contact ball
% cx = -0.04 + 0.08*rand; %initialized contact point in the local x
% cy = -0.04 + 0.08*rand; %initialized contact point in the local y
cx = 0; %initialized contact point in the local x
cy = 0; %initialized contact point in the local y
%compute the real contact position in 3D.
tactile_ct = T_tool_end_eff_init*[cx,cy,0,1]';
sphere_r = 0.005;
% drawsphere(tactile_ct(1:3),sphere_r);

%this is a flag to improve the visualization quality 0 is only geometry, 1
%with robot
Flag_userobot = 0;
len = 80;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%Two steps method to estimate normal direction and rotation
%%%%%%%%%%%%%%%angle between virtual frame and real tactile sensor frame.
% this method has been obsolete%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% estimate normal direction using the initialized nv guess from the
% approaching trajectory
% [n_hat, dis_set, dis_set2,tool_1st_end_eff_frame,n_hat_set,nv_set,tan1,tan2] = est_nv_tac(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise,Flag_userobot);
% disp('updated n_hat');
% n_hat
% % estimate rotate angle from the virtual tool frame to real tool frame
% rotate_angle = est_rotate_tac(n_hat,tool_1st_end_eff_frame,tactile_ct);
% disp('rotation angle along z is');
% rotate_angle

% %%%%%%%%%%%%%%%%%%%%%%%%%%If we consider about the relation between the%%%
% %%%%%%%%%%%%%robot eef and tool eef as the homogeneous matrix%%%%%%%%%%%%%
% %%%following , we firstly estimate the rotation matrix, then estimate
% %%%translation.%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%%%%%%%%%%% one step to estimate the normal direction and rotation angle.
% [n_hat n_hat_set]= est_rotate_tac_onestep(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise,Flag_userobot,tactile_ct);

%estimate translation from robot end-effector to tool end-effector
% est_trans = est_translation_tac(kuka_robot,Q,tool_transform,tool_rotate,link_value);
[est_trans, est, omiga_vec_real, omiga_vec_est, vel_real_est, vel_est]= est_translation_tac_analysis(kuka_robot,Q,tool_transform,tool_rotate,link_value,len);
% est_trans_alg();
vis_learn_process(est, omiga_vec_est, vel_real_est, vel_est,link_value,len);



