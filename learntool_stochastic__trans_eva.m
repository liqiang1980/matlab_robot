% this is the code to let the robot learn the usage of tool in a
% exploration way. 
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
stochastic_num = 20;
est_set = zeros(3,80,stochastic_num);
mean_dev_val = zeros(3,stochastic_num);
std_val = zeros(3,stochastic_num);

%load robot model and calculate the robot model kinematics
kuka_robot = loadrobot('kukalwr');
%visualization of kuka_lwr at the initialized pose
Q = rand(1,7);
%the robot end-effector frame at the initialized status
T_robot_end_eff_init = kuka_robot.fkine(Q);
for i = 1:stochastic_num
    disp('times is: ');
    i
link_value = rand(3,1);
rot_value = [0.2,0.5,0.3];
tool_rotate = trotz(rot_value(3))*troty(rot_value(2))*trotx(rot_value(1));
tool_translate = transl(link_value);
%must firstly translation then rotation, we can get the estimation of
%translation is same with link_value
tool_transform = tool_translate * tool_rotate;



[est_trans, est, omiga_vec_real, omiga_vec_est, vel_real_est, vel_est]= est_translation_tac_analysis(kuka_robot,Q,tool_transform,tool_rotate,link_value,len);
est_set(:,:,i) = est';
mean_dev_val(1,i) = link_value(1)-mean(est(50:80,1));
mean_dev_val(2,i) = link_value(2)-mean(est(50:80,2));
mean_dev_val(3,i) = link_value(3)-mean(est(50:80,3));
std_val(1,i) = std(est(50:80,1));
std_val(2,i) = std(est(50:80,2));
std_val(3,i) = std(est(50:80,3));
%estimate translation from robot end-effector to tool end-effector
% est_trans = est_translation_tac(kuka_robot,Q,tool_transform,tool_rotate,link_value);
% [est_trans, omiga_vec_est, vel_real_est, vel_est]= est_translation_tac_analysis(kuka_robot,Q,tool_transform,tool_rotate,link_value);
% vis_learn_process(est_trans, omiga_vec_est, vel_real_est, vel_est,link_value);

end