% Estimate the contact frame orientation in one step
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