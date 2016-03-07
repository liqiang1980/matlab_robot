% Estimate the rotation angle of tool frame tagent surface using the
% updated normal direction estimation from the last step
%
% rotate_angle = est_rotate_tac(n_hat,tool_1st_end_eff_frame) is function to update
% the tangent surface of tool given a initialized arbitary guess generated 
% from updated normal direction
% n_hat: updated normal direction from the 1st step, 3*1 vector
% tool_1st_end_eff_frame: real tool end effector frame after the
% exploration action of 1st step, 4*4 homogeneous matrix
% See also 
%
%if the tactool slides along x axis(generated tool frame)--positive, 
%we get the taxel trajectory. In simulation, the negtive x axis
%is known which is Rv_r*[-1,0,0]; where Rv_r is transformatrix from virtual
%frame to the real tactile sensor frame.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rotate_angle = est_rotate_tac(n_hat,tool_1st_end_eff_frame)
%compute the updated noised othogonal matrix, because the update process in
%the last step, now there is very small error in the n_hat with the real
%normal direction, this step is using sliding movement in order to estimate
%the real tangent surface frame.
T_tool_end_eff_1stupdate = genOthoBasis(n_hat);
for j =1:1:sample_num
    if(j==1)
        T_tool_end_eff_cur = tool_1st_end_eff_frame;
    else
        T_tool_end_eff_cur = new_tool_end_eff_frame;
    end
    %update tool end-effector frame every control step
    em = 1;%exploring along x axis
    new_tool_end_eff_frame = update_ct_surf(T_tool_end_eff_cur,T_tool_end_eff_1stupdate,em);
    %get the contact position in tactile sensor on the tool end-effector
    [cx,cy] = get_tac_position(T_tool_end_eff_cur,tactile_ct);
    %add gaussian noise and collect 2d contact position
    ct_set(j,1) = cx+0.001*randn;
    ct_set(j,2) = cy+0.001*randn;;
end
%major direction from collected position
% for (std::vector<Eigen::Vector2d>::iterator it = ps.begin(); it != ps.end(); ++it){
%     sum_x += (*it)(0); sum_y += (*it)(1); sum_xy += (*it)(0) * (*it)(1); sum_x2 += (*it)(0) * (*it)(0);
%     }
%     // means
%     double mean_x = sum_x / ps.size();
%     double mean_y = sum_y / ps.size();
%     
%     double varx = sum_x2 - sum_x * mean_x;
%     double cov = sum_xy - sum_x * mean_y;
%     
%     rgp.k = cov / varx;
%     rgp.b = mean_y - rgp.k * mean_x;
%     
%     double a;
%     rgp.deltay = (*ps.end())(1) - (*ps.begin())(1);
%     rgp.deltax = (*ps.end())(0) - (*ps.begin())(0);
%     a = atan2(rgp.deltay,rgp.deltax);
%     std::cout<<"atan2 "<<a*180.0/M_PI<<std::endl;
%     rgp.sign_k = sign(a);
vec = rot_tm(1:3,1:3) * [-1;0;0];
angle = atan2(vec(2),vec(1));

if(sign(angle) == 1 )
    if((vec(1)>0)&&(vec(2)>0))
        gama = atan(vec(2)/vec(1))+pi;
    else
        gama = atan(vec(2)/vec(1));
    end
else
    if((vec(1)>0)&&(vec(2)<0))
        gama = atan(vec(2)/vec(1))+pi;
    else
        gama = atan(vec(2)/vec(1));
    end
end
rotate_angle = gama;
