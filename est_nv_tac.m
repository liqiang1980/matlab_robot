% Estimate the normal direction of tool
%
% n_hat = est_nv_tac(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise) is function to update
% the normal direction of tool given a initialized guess from approaching
% trajectory. The contacted part is one point and tactile tool is a square
% surface--myrmex
%
% kuka_robot: robot kinematics model
% Q: the joint angle value, array 7*1
% tool_transform: the real tool transform homogeneous matrix
% n_hat[right side]: intialized normal direction, array 3*1
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% exploration action in order to estimate the normal direction
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function n_hat = est_nv_tac(kuka_robot,Q,tool_transform,T_tool_end_eff_init_noise)
%initialize the parameters
P_bar = zeros(3);
L_n = zeros(3);
L_n_dot = zeros(3);
beta_n = 0.99;
gamma_n = 0.5;
n_hat = T_tool_end_eff_init_noise(1:3,3);
n_hat_dot = zeros(3,1);
sample_num = 100;
%define index for the stored the normal direction
set_index = 0;

for j =1:1:sample_num
    %compute the current robot and tool configure
    T_robot_end_eff_cur = kuka_robot.fkine(Q);
    T_tool_end_eff_cur = T_robot_end_eff_cur*tool_transform;
    %update tool end-effector frame every control step
    new_tool_end_eff_frame = update_ct_surf(T_tool_end_eff_cur,T_tool_end_eff_init_noise);
    %estimate the linear velocity of tool, which is also the linear
    %velocity of robot end-effector
    tool_lv_dot_global = new_tool_end_eff_frame(1:3,4) - T_tool_end_eff_cur(1:3,4);
    %from the desired linear velocity computing the joint angle rate
    Jac = kuka_robot.jacob0(Q);
    %using the psudo inverse is not good idea here because the jitter
    %movement to compute the joint rate from cartesian velocity
%     q_dot = pinv(Jac)*[p_e_dot;0;0;0];
    %using the damped least square
    lamda = 0.5;
    q_dot = Jac'*inv(Jac*Jac'+lamda^2*eye(6))*[tool_lv_dot_global;0;0;0];
    %rendering the robot model
%     kuka_robot.plot(Q,'workspace',[-3, 3 -3, 3 -3, 3]);
    
    %algorithm to estimate the normal direction
    tool_lv_dot_global = 1000*tool_lv_dot_global;
    P_bar = eye(3)-n_hat*n_hat';
    n_hat_dot = -1*gamma_n*P_bar*L_n*n_hat;
    L_n_dot = -beta_n*L_n+(1/(1+norm(tool_lv_dot_global)^2))*tool_lv_dot_global*tool_lv_dot_global';
    n_hat = n_hat+n_hat_dot;
    n_hat = n_hat/norm(n_hat);
    L_n = L_n + L_n_dot;
    %visualization of the normal direction approaching from the initial
    %guess to the real direction
    if(mod(sample_num,10) == 0)
        set_index = set_index + 1;
        n_hat_set(:,set_index) = n_hat;
        n_hat_distance(set_index) = dot(n_hat,T_tool_end_eff_cur(1:3,3)');
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %visualization%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        est_nv_start = T_tool_end_eff_cur(1:3,4)';
        est_nv_end = est_nv_start+n_hat';
        pts = [est_nv_start; est_nv_end];
        plot3(pts(:,1), pts(:,2), pts(:,3),'g-');
        hold on;
        %real normal direction
        nv_start = T_tool_end_eff_cur(1:3,4)';
        nv_end = nv_start+T_tool_end_eff_cur(1:3,3)';
        pts = [nv_start; nv_end];
        plot3(pts(:,1), pts(:,2), pts(:,3),'m-');
        hold on;  
        %draw tool: bar+square
        myrmexsize = 0.08;
        color = [0.3,0.6,0.8];
        drawtactool(T_robot_end_eff_cur,T_tool_end_eff_cur,myrmexsize,color) ;
        %draw contact ball
        cx = -0.04 + 0.08*rand; %initialized contact point in the local x
        cy = -0.04 + 0.08*rand; %initialized contact point in the local y
        c = T_tool_end_eff_cur*[cx,cy,0,1]';
        sphere_r = 0.002;
        drawsphere(c(1:3),sphere_r);
        % We consider about tool as a line segment and draw tool with line between two 3d points
        P1 = T_robot_end_eff_cur(1:3,4)';
        P2 = T_tool_end_eff_cur(1:3,4)';
        pts = [P1; P2];
        plot3(pts(:,1), pts(:,2), pts(:,3),'LineWidth',5);
    end
    %update the robot joint angle
    Q= Q+q_dot';
    pause(0.1);
end