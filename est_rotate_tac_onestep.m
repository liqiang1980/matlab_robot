% Estimate the contact frame orientation in one step
%
% hm = est_rotate_tac_onestep(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise,Flag_userobot,tactile_ct) 
%
%
% kuka_robot: robot kinematics model
% Q: the joint angle value, array 7*1
% tool_transform: the real tool transform homogeneous matrix
% T_tool_end_eff_init_noise: assumption noised homogeneous matrix of tool frame.
% Flag_userobot: flag whether use robot or only geometry compuation.
% tactile_ct: contact position in the world frame
% See also 
%
%[n_hat, dis_set, dis_set2,tool_1st_end_eff_frame,n_hat_set,nv_set,tan1,tan2] = est_nv_tac(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise,Flag_userobot);
%rotate_angle = est_rotate_tac(n_hat,tool_1st_end_eff_frame,tactile_ct);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [n_hat n_hat_set]= est_rotate_tac_onestep(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise,Flag_userobot,tactile_ct)
%initialize the parameters
P_bar = zeros(3);
L_n = zeros(3);
L_n_dot = zeros(3);
beta_n = 0.99;
gamma_n = 0.5;

n_hat = T_tool_end_eff_init_noise(1:3,3);
n_hat_dot = zeros(3,1);
sample_num = 80;
%define index for the stored the normal direction
set_index = 0;
rotation_est_ind = 1;
rotation_est_num = 4;
ct_R = zeros(rotation_est_num,2);
ct_V = zeros(rotation_est_num,2);
noised_tool_lv_dot_local_sum = zeros(3,1);

n_hat_set = zeros(3,sample_num);

%incrementally learn orientation matrix of tool
for j =1:1:sample_num    
    %compute the current robot and tool configure
    T_robot_end_eff_cur = kuka_robot.fkine(Q);
    if((Flag_userobot == 1)||(j==1))
        T_tool_end_eff_cur = T_robot_end_eff_cur*tool_transform;
    else
        T_tool_end_eff_cur = new_tool_end_eff_frame;
    end
    %draw tool: bar+square
%     myrmexsize = 0.08;
%     color = [0.3,0.6,0.8];
%     drawtactool(T_robot_end_eff_cur,T_tool_end_eff_cur,myrmexsize,color) ;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %update tool end-effector frame every control step using different
    %exploring mode(trajectory designing)-em.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Todo: online update action strategy in order to learn faster.
    em = 5;%circle motion in the tangent surface;
    if(j==1)
        T_tool_end_eff_cur_noise = T_tool_end_eff_init_noise;
    else
        T_tool_end_eff_cur_noise = new_T_tool_end_eff_noise;
    end
    [new_tool_end_eff_frame noised_tool_lv_dot_local] = update_ct_surf(T_tool_end_eff_cur,T_tool_end_eff_cur_noise,em,j);
    %estimate the linear velocity of tool, which is also the linear
    %velocity of robot end-effector, and also contact point linear velocity
    tool_lv_dot_global = new_tool_end_eff_frame(1:3,4) - T_tool_end_eff_cur(1:3,4);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%instant inverse kinematics%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %from the desired linear velocity computing the joint angle rate
    Jac = kuka_robot.jacob0(Q);
    %using the psudo inverse is not good idea here because the jitter
    %movement to compute the joint rate from cartesian velocity
%     q_dot = pinv(Jac)*[p_e_dot;0;0;0];
    %using the damped least square
    lamda = 0.9;
    q_dot = Jac'*inv(Jac*Jac'+lamda^2*eye(6))*[tool_lv_dot_global;0;0;0];
    %rendering the robot model
%     kuka_robot.plot(Q,'workspace',[-3, 3 -3, 3 -3, 3]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%algorithm to estimate the normal direction%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tool_lv_dot_global = 2000*tool_lv_dot_global;
    P_bar = eye(3)-n_hat*n_hat';
    n_hat_dot = -1*gamma_n*P_bar*L_n*n_hat;
    L_n_dot = -beta_n*L_n+(1/(1+norm(tool_lv_dot_global)^2))*tool_lv_dot_global*tool_lv_dot_global';
    n_hat = n_hat+n_hat_dot;
    n_hat = n_hat/norm(n_hat);
    L_n = L_n + L_n_dot;
    n_hat_set(:,j) = n_hat;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%from the current estimated normal direction to update the estimated%
    %%sensor frame%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    new_T_tool_end_eff_noise = rotate_generate(T_tool_end_eff_cur_noise,n_hat);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % in oder to estimate the rotation matrix from virtual frame to the
    % real frame, more data are needed instead of only start point and end
    % point which is used for the normal direction estimation
    % first step: collect data, est rotation matrix per $est_rotation_num data 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if(rotation_est_ind <= rotation_est_num)
        %get the contact position in tactile sensor on the tool end-effector
        [cx,cy] = get_tac_position(T_tool_end_eff_cur,tactile_ct);
        %add gaussian noise and collect 2d contact position
        ct_R(rotation_est_ind,1) = cx+0.0005*randn;
        ct_R(rotation_est_ind,2) = cy+0.0005*randn;
        ct_V(rotation_est_ind,1) = (-1)*noised_tool_lv_dot_local_sum(1);
        ct_V(rotation_est_ind,2) = (-1)*noised_tool_lv_dot_local_sum(2);
        rotation_est_ind = rotation_est_ind + 1;
        noised_tool_lv_dot_local_sum = noised_tool_lv_dot_local+noised_tool_lv_dot_local_sum;
    else
        %calculate once while data are collected
        rotation_matrix_2d = est_rotatematrix(ct_V,ct_R);
%         n_hat
%         inv(T_tool_end_eff_cur)*new_T_tool_end_eff_noise
        rotation_est_ind = 1;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %visualization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(mod(j,1) == 0)
        est_nv_start = T_tool_end_eff_cur(1:3,4)';
        est_nv_end = est_nv_start+n_hat';
        pts = [est_nv_start; est_nv_end];
%         plot3(pts(:,1), pts(:,2), pts(:,3),'g-');
        hold on;
        %real normal direction
        nv_start = T_tool_end_eff_cur(1:3,4)';
        nv_end = nv_start+T_tool_end_eff_cur(1:3,3)';
        pts = [nv_start; nv_end];
%         plot3(pts(:,1), pts(:,2), pts(:,3),'m-');
        hold on;  
        %draw tool: bar+square
        myrmexsize = 0.08;
        color = [0.3,0.6,0.8];
%         drawsquare(T_tool_end_eff_cur,myrmexsize,color);
    end
    %update the robot joint angle
    Q= Q+q_dot';
    pause(0.2);
end