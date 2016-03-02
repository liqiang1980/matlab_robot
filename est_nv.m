% Estimate the normal direction of tool
%
% n_hat = est_nv(kuka_robot,Q,tool_transform,n_hat) is function to update
% the normal direction of tool given a initialized guess from approaching
% trajectory
%
% kuka_robot: robot kinematics model
% Q: the joint angle value
% tool_transform: the real tool transform homogeneous matrix
% n_hat[right side]: intialized normal direction
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% exploration action in order to estimate the normal direction
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function n_hat = est_nv(kuka_robot,Q,tool_transform,n_hat)
%initialize the parameters
P_bar = zeros(3);
L_n = zeros(3);
L_n_dot = zeros(3);
beta_n = 0.99;
gamma_n = 0.5;
n_hat_dot = zeros(3,1);
sample_num = 100;
%define index for the stored the normal direction
set_index = 0;

for j =1:1:sample_num
    %compute the current robot and tool configure
    T_robot_end_eff_cur = kuka_robot.fkine(Q);
    T_tool_end_eff_cur = T_robot_end_eff_cur*tool_transform;
    trplot(T_robot_end_eff_cur, 'frame', 'A','color','c');
    trplot(T_tool_end_eff_cur, 'frame', 'B','color','r');
    %desired velocity in the local tool end-effector frame
    tool_lv_dot_local = 0.01*rand(3,1);
    tool_lv_dot_local(3) = 0;
    tool_lv_dot_global = T_tool_end_eff_cur(1:3,1:3) * tool_lv_dot_local;
    %from the desired linear velocity computing the joint angle rate
    Jac = kuka_robot.jacob0(Q);
    %using the psudo inverse is not good idea here because the jitter
    %movement to compute the joint rate from cartesian velocity
%     q_dot = pinv(Jac)*[p_e_dot;0;0;0];
    %using the damped least square
    lamda = 0.5;
    q_dot = Jac'*inv(Jac*Jac'+lamda^2*eye(6))*[tool_lv_dot_global;0;0;0];
    % We consider about tool as a line segment and draw tool with line between two 3d points
    P1 = T_robot_end_eff_cur(1:3,4)';
    P2 = T_tool_end_eff_cur(1:3,4)';
    pts = [P1; P2];
    plot3(pts(:,1), pts(:,2), pts(:,3),'LineWidth',5);
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
        %estimated normal direction
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
    end
    %update the robot joint angle
    Q= Q+q_dot';
    pause(0.1);
end