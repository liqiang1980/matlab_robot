%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
cd ..
%load robot model
kuka_robot = loadrobot('kukalwr');
% %     visualization of kuka_lwr at the initialized pose
Q = rand(1,7);
kuka_robot.plot(Q,'workspace', [-3 3 -3 3 -3 3]);
%model the tactool frame
T_init = kuka_robot.fkine(Q);
% kuka_robot.jacob0(Q);
link_value = rand(3,1);
rot_value = [0.2,0.5,0.3];
tool_rotate = trotz(rot_value(3))*troty(rot_value(2))*trotx(rot_value(1));
tool_translate = transl(link_value);
tool_transform = tool_rotate * tool_translate ;
T_eff_start = T_init*tool_transform;


%start moving computed by inverse jacobian matrix
for i =1:1:20
    %compute the current robot and tool configure
    T_eff_cur = kuka_robot.fkine(Q);
    T_tool_cur = T_eff_cur*tool_transform;
%     trplot(T_eff_cur, 'frame', 'A');
%     trplot(T_tool_cur, 'frame', 'B','color','r');
    p_t_dot_local = 0.005*rand(3,1);
    p_t_dot_local(3) = 0;
    p_e_dot_local = T_eff_cur(1:3,1:3)'*T_tool_cur(1:3,1:3)*p_t_dot_local;
    p_e_dot = T_eff_cur(1:3,1:3)*p_e_dot_local;
    Jac = kuka_robot.jacobn(Q);
    q_dot = pinv(Jac)*[p_e_dot;0;0;0];
    
    % two points
    P1 = T_eff_cur(1:3,4)';
    P2 = T_tool_cur(1:3,4)';
    
    if(i~=1)
        deltap = P2 - P2_last;
        T_tool_cur(1:3,1:3)'*deltap'
    end
    
    % Their vertial concatenation is what you want
    pts = [P1; P2];
    % Alternatively, you could use plot3:
    plot3(pts(:,1), pts(:,2), pts(:,3),'LineWidth',5);
    kuka_robot.plot(Q,'workspace', [-3 3 -3 3 -3 3]);
    hold on;
    %update joint angle
    Q= Q+q_dot';
    
    %only for testing whether moving is one the surface
    P2_last = P2;
    pause(1);
end

