% Estimate the translation from robot end-effector to tool end-effector
%
% est_trans = est_translation(kuka_robot,Q,tool_transform,tool_rotate) is function to update
% the translation from the robot end-effector frame to the tool end-effector frame
%
% kuka_robot: robot kinematics model
% Q: the joint angle value
% tool_transform: the real tool transform homogeneous matrix
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rotation action in order to estimate the translation.
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function est_trans = est_translation(kuka_robot,Q,tool_transform,tool_rotate,link_value)
T_robot_eff_end_last = eye(4);
T_tool_eff_end_last = eye(4);
Gama_r = 30*eye(3);
L_r = zeros(3);
L_r_dot = zeros(3);
c_r = zeros(3,1);
c_r_dot = zeros(3,1);
beta_r = 0.99;
est_trans = zeros(3,1);
est_trans_dot = zeros(3,1);

sample_num = 300;
    for j =1:1:sample_num
        %get the robot current state
        T_robot_end_eff_init = kuka_robot.fkine(Q);
        %kuka_lwr generate the exploration action using its end-effector
        %rotation
        rotate_actx = r2t(rotz(0.1*rand));
        rotate_acty = r2t(roty(0.1*rand));
        rotate_actz = r2t(rotx(0.1*rand));
        T_robot_end_eff_cur = T_robot_end_eff_init*rotate_actz*rotate_acty*rotate_actx;
        T_tool_end_eff_cur = T_robot_end_eff_cur*tool_transform;
%         trplot(T_robot_end_eff_cur, 'frame', 'C');
%         trplot(T_tool_end_eff_cur, 'frame', 'D');
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%estimation alg part%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%mls94 p.72 electronic version,eq. 2.53;
        %estimate the rotation velocity
        omiga_skmatrix = (t2r(T_robot_end_eff_cur) - t2r(T_robot_eff_end_last)) *(t2r(T_robot_end_eff_cur))';
%         vel = (-1)*(t2r(T_eff_end_cur) - t2r(T_eff_end_last))*(t2r(T_eff_end_cur))'*...
%             T_eff_end_cur(1:3,4) +(T_eff_end_cur(1:3,4)-T_eff_end_last(1:3,4))
        omiga_vec = [omiga_skmatrix(3,2);omiga_skmatrix(1,3);omiga_skmatrix(2,1)];
        omiga_vec_est(j,:) = omiga_vec;
        rotation_rm = T_robot_end_eff_init*rotate_actx;
%         disp('rotation axis');
%         rotation_axis = rotation_rm(1:3,3)
%         disp('rotation from the estimation')
%         omiga_vec/norm(omiga_vec)
        %compute the linear velocity from the differentiate position of the end-effector of
        %the tool
        vel = (-1)*(T_tool_end_eff_cur(1:3,4)-T_tool_eff_end_last(1:3,4)+0.01*rand(3,1));

        L_r_dot = (-1)*beta_r*L_r-omiga_skmatrix*omiga_skmatrix;
        c_r_dot = (-1)*beta_r*c_r+omiga_skmatrix*vel;
        est_trans_dot = (-1)*Gama_r*(L_r*est_trans-c_r);
        L_r = L_r + L_r_dot;
        c_r = c_r + c_r_dot;
        est_trans = est_trans + est_trans_dot;
        est_trans2=(T_robot_end_eff_cur * tool_rotate)'*[est_trans;0];
        est(j,1:3) = est_trans2(1:3);
        est(j,4) = norm(cross(omiga_vec,est_trans2(1:3))-vel);
        T_robot_eff_end_last = T_robot_end_eff_cur;
        T_tool_eff_end_last = T_tool_end_eff_cur;
    end
figure(2)
subplot(4,1,1);
plot(200:sample_num,est(200:sample_num,1));
subplot(4,1,2);
plot(200:sample_num,est(200:sample_num,2));
subplot(4,1,3);
plot(200:sample_num,est(200:sample_num,3));
subplot(4,1,4);
plot(200:sample_num,est(200:sample_num,4));
disp('estimated link parameters')
[mean(est(200:sample_num,1)),mean(est(200:sample_num,2)),mean(est(200:sample_num,3))]'

figure(3);
subplot(3,1,1);
plot(1:sample_num,omiga_vec_est(1:sample_num,1));
subplot(3,1,2);
plot(1:sample_num,omiga_vec_est(1:sample_num,2));
subplot(3,1,3);
plot(1:sample_num,omiga_vec_est(1:sample_num,3));

disp('real link parameters')
link_value




