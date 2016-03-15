% Update contact surface
%
% new_tool_end_eff_frame = update_ct_surf(T_tool_end_eff_cur,T_tool_end_eff_cur_noise) is function to 
% update the tactile tool frame because the noised initialized frame while
% the sliding exploration happens.
%
% T_tool_end_eff_cur: current tool end-effector homogeneous matrix
% new_tool_end_eff_frame: updated tool frame after one step sliding
% exploration
% em: exploring mode. 1, sliding along x axis; 2, sliding along y axis; 3.
% random exploring
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonomous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [new_tool_end_eff_frame noised_tool_lv_dot_local] = update_ct_surf(T_tool_end_eff_cur,T_tool_end_eff_cur_noise,em,j)
% analog the robot interactive action while there is a normal direciton
% estimation error. The whole analog process is compose three part
% (1) real tool frame randomly moving(exploring) along the estimated tool frame
% tangent surface.
% (2) compute the contact distance along the estimated tool frame normal
% direction
% (3) new tool frame center is the 1st step result translated by 2nd step 
% desired velocity in the local noised tool end-effector frame
switch em
    case 1
        noised_tool_lv_dot_local = zeros(3,1);
        noised_tool_lv_dot_local(1) = 0.001;
        noised_tool_lv_dot_local(2) = 0;
        noised_tool_lv_dot_local(3) = 0;
    case 2
        noised_tool_lv_dot_local = zeros(3,1);
        noised_tool_lv_dot_local(1) = 0;
        noised_tool_lv_dot_local(2) = 0.001;
        noised_tool_lv_dot_local(3) = 0;
    case 3
        noised_tool_lv_dot_local = 0.002*randn(3,1);
        noised_tool_lv_dot_local(3) = 0;
    case 4
        noised_tool_lv_dot_local = zeros(3,1);
        noised_tool_lv_dot_local(1) = 0.0002*sin(0.02*j);
        noised_tool_lv_dot_local(2) = 0.0002*cos(0.02*j);
        noised_tool_lv_dot_local(3) = 0;
    case 5
        noised_tool_lv_dot_local = generate_square_shape(j,0.01);
    otherwise
        disp('exploring mode 1:along x; 2: along y; 3:random; 4:circle, 5 squareshape are you planning add a new mode');
end

noised_tool_lv_dot_global = T_tool_end_eff_cur_noise(1:3,1:3) * noised_tool_lv_dot_local;
% after the 1st step, the tool frame origin is
tool_1st_end_eff = eye(4);
tool_1st_end_eff(1:3,1:3) = T_tool_end_eff_cur(1:3,1:3);
tool_1st_end_eff(1:3,4) = T_tool_end_eff_cur(1:3,4) + noised_tool_lv_dot_global;

%2nd step compute the contact point translation distance, this step is used
%for analoging the admittance controller. because tool frame is along the
%estimated tool frame tangent surface sliding, the normal direction will be
%shifted reactively along the tangent surface moving result.
%using the point vector mode to represent line equation, then all points on
%the line is represented by r_vec = r0_vec + t*v_vec;
% if v_vec is the estimated normal direction, r0_vec is the input tool
% frame origin, the line along the estimated normal direction will be
% intersected with 1st tool tangent surface (X-Y surface). At the
% intersection point, the z value represented in the local estimated tool
% frame is zero. With this relation, the "t" can be computed.
tool_inv = inv(tool_1st_end_eff);
t = -1 * tool_inv(3,1:4)* T_tool_end_eff_cur(:,4)/...
    (tool_inv(3,1:4)*[T_tool_end_eff_cur_noise(1:3,3);0]);

%the result from 1st step should be translated by t in the estimated tool
%frame, the final tool frame is computed
t_global = T_tool_end_eff_cur_noise(1:3,1:3)*[0;0;-t];

new_tool_end_eff_frame = tool_1st_end_eff;
new_tool_end_eff_frame(1:3,4) =  tool_1st_end_eff(1:3,4) + t_global;

