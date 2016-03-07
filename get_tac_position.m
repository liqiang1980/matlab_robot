% tactile sensor model simulate the contact position in the real tool 
% end-effector frame.
%
% [cx,cy] = est_rotate_tac(n_hat,tool_1st_end_eff_frame) 
% tactile_ct: contact position in 3D
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [cx,cy] = get_tac_position(T_tool_end_eff_cur,tactile_ct)
ct = inv(T_tool_end_eff_cur)*[tactile_ct;1];
cx = ct(1);
cy = ct(2);