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
function rotate_angle = est_rotate_tac(n_hat,tool_1st_end_eff_frame,tactile_ct)
%compute the updated noised othogonal matrix, because the update process in
%the last step, now there is very small error in the n_hat with the real
%normal direction, this step is using sliding movement in order to estimate
%the real tangent surface frame.
T_tool_end_eff_1stupdate = genOthoBasis(n_hat);
virtual_visualization = eye(4);
transf = eye(4);
transf(1:3,1:3) = tool_1st_end_eff_frame(1:3,1:3)'*T_tool_end_eff_1stupdate;
disp('RPY angle is ');
tr2rpy(transf)
sample_num = 50;
for j =1:1:sample_num
    if(j==1)
        T_tool_end_eff_cur = tool_1st_end_eff_frame;
    else
        T_tool_end_eff_cur = new_tool_end_eff_frame;
    end
    %update tool end-effector frame every control step
    em = 1;%exploring along x axis
    new_tool_end_eff_frame = update_ct_surf(T_tool_end_eff_cur,T_tool_end_eff_1stupdate,em,j);
    %draw tool: bar+square
    myrmexsize = 0.08;
    color = [0.3,0.6,0.8];
    drawsquare(new_tool_end_eff_frame,myrmexsize,color);
    %get the contact position in tactile sensor on the tool end-effector
    [cx,cy] = get_tac_position(T_tool_end_eff_cur,tactile_ct);
    %add gaussian noise and collect 2d contact position
    ct_set(j,1) = cx+0.001*randn;
    ct_set(j,2) = cy+0.001*randn;
    
    %visualization
    virtual_visualization(1:3,1:3) = T_tool_end_eff_1stupdate;
    virtual_visualization(1:3,4) = T_tool_end_eff_cur(1:3,4);
    trplot(virtual_visualization, 'frame', 'V','color','r','length',0.1);
    trplot(T_tool_end_eff_cur, 'frame', 'R','length',0.1);
end

%using the regression method to estimate the slop and intercept in the 2d
%case
[k,b,deltay,deltax,atan2_k,k2] = regression_2d(ct_set);

if(sign(atan2_k) == 1 )
    if((deltay>0)&&(deltax>0))
        gama = k2+pi;
    else
        gama = k2;
    end
else
    if((deltay>0)&&(deltax<0))
        gama = k2+pi;
    else
        gama = k2;
    end
end
rotate_angle = gama;
