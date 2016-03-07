% Estimate the normal direction of tool
%
% n_hat = est_nv_tac(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise,Flag_userobot) is function to update
% the normal direction of tool given a initialized guess from approaching
% trajectory. The contacted part is one point and tactile tool is a square
% surface--myrmex
%
% kuka_robot: robot kinematics model
% Q: the joint angle value, array 7*1
% tool_transform: the real tool transform homogeneous matrix
% T_tool_end_eff_init_noise: assumption noised homogeneous matrix of tool frame.
% Flag_userobot: flag whether use robot or only geometry compuation.
% n_hat[right side]: intialized normal direction, vector 3*1
% dis_set is the distance set to store neighbour frame update
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [n_hat, dis_set, new_tool_end_eff_frame]= est_nv_tac(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise,Flag_userobot)
%initialize the parameters
P_bar = zeros(3);
L_n = zeros(3);
L_n_dot = zeros(3);
beta_n = 0.99;
gamma_n = 0.5;
n_hat = T_tool_end_eff_init_noise(1:3,3);
T_tool_end_eff_old = eye(4);
n_hat_dot = zeros(3,1);
sample_num = 100;
%define index for the stored the normal direction
set_index = 0;

for j =1:1:sample_num
    
    %compute the current robot and tool configure
    T_robot_end_eff_cur = kuka_robot.fkine(Q);
    if((Flag_userobot == 1)||(j==1))
        T_tool_end_eff_cur = T_robot_end_eff_cur*tool_transform;
    else
        T_tool_end_eff_cur = new_tool_end_eff_frame;
    end
    %draw tool: bar+square
    myrmexsize = 0.08;
    color = [0.3,0.6,0.8];
    drawtactool(T_robot_end_eff_cur,T_tool_end_eff_cur,myrmexsize,color) ;
    %update tool end-effector frame every control step
    em = 3%random exploring in the tangent surface;
    new_tool_end_eff_frame = update_ct_surf(T_tool_end_eff_cur,T_tool_end_eff_init_noise,em);
    %because the numerical error of inverse kinematics methd[new_tool_end_eff_frame is not
    %equal to the T_tool_end_eff_cur(next control step)], the distance
    %of neighbour update is computed and stored. I can be sure that
    %update_ct_surface is good because if we replace T_tool_end_eff_old
    %with new_tool_end_eff_frame, then dis = 0.(dis is the distance along 
    %the real normal direction)
    dis = inv(T_tool_end_eff_cur)*(T_tool_end_eff_old(:,4) - T_tool_end_eff_cur(:,4));
    dis_set(j) = dis(3);
    %estimate the linear velocity of tool, which is also the linear
    %velocity of robot end-effector, and also contact point linear velocity
    tool_lv_dot_global = new_tool_end_eff_frame(1:3,4) - T_tool_end_eff_cur(1:3,4);
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
    
    %algorithm to estimate the normal direction
    tool_lv_dot_global = 200*tool_lv_dot_global;
    P_bar = eye(3)-n_hat*n_hat';
    n_hat_dot = -1*gamma_n*P_bar*L_n*n_hat;
    L_n_dot = -beta_n*L_n+(1/(1+norm(tool_lv_dot_global)^2))*tool_lv_dot_global*tool_lv_dot_global';
    n_hat = n_hat+n_hat_dot;
    n_hat = n_hat/norm(n_hat);
    L_n = L_n + L_n_dot;
    %visualization of the normal direction approaching from the initial
    %guess to the real direction
    if(mod(j,10) == 0)
        set_index = set_index + 1;
        n_hat_set(:,set_index) = n_hat;
        n_hat_distance(set_index) = dot(n_hat,T_tool_end_eff_cur(1:3,3)');
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %visualization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        est_nv_start = T_tool_end_eff_cur(1:3,4)';
        est_nv_end = est_nv_start+n_hat';
        pts = [est_nv_start; est_nv_end];
%         plot3(pts(:,1), pts(:,2), pts(:,3),'g-');
%         hold on;
        %real normal direction
        nv_start = T_tool_end_eff_cur(1:3,4)';
        nv_end = nv_start+T_tool_end_eff_cur(1:3,3)';
        pts = [nv_start; nv_end];
%         plot3(pts(:,1), pts(:,2), pts(:,3),'m-');
%         hold on;  
    end
    %update the robot joint angle
    T_tool_end_eff_old = T_tool_end_eff_cur;
    Q= Q+q_dot';
    pause(0.1);
end