% Estimate rotation angle rate
%
% [omega_s,omega_b] = est_rotation_rate(cur,last)
% 
%
% omega_s: rotation rate in the reference frame
% omega_b: rotation rate in the body frame
% cur, current rotation matrix
% last, rotation matrix last step
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [omega_s,omega_b] = est_rotation_rate(cur,last)
skew_matrix_b = cur'*(cur-last);
skew_matrix_s = (cur-last) * cur';
omega_s = vex(skew_matrix_s);
omega_b = vex(skew_matrix_b);



