% Generate the open loop rotation exploring action, given the initialized robot end effector
% pose.
%
% [T_robot_end_eff_cur] = rotation_explore_integration(T_robot_end_eff_init,ind)
% 
%
% kuka_robot: robot kinematics model
% Q: the joint angle value
% tool_transform: the real tool transform homogeneous matrix
% tool_rotate: the real tool rotation matrix
% link_value: real link translation parameters
% See also 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function T_robot_end_eff_cur = rotation_explore_integration(T_robot_end_eff_init,ind)
scale = 0.02*rand;
if((mod(ind,200)<50)&&(mod(ind,200)>=0))
rotate_actx = r2t(rotx(mod(ind,200)*scale));
rotate_acty = eye(4);
rotate_actz = eye(4);
end
if((mod(ind,200)<100)&&(mod(ind,200)>=50))
rotate_actx = r2t(rotx((-1)*(mod(ind,200)-100)*scale));
rotate_acty = eye(4);
rotate_actz = eye(4);
end
if((mod(ind,200)<150)&&(mod(ind,200)>=100))
rotate_actx = eye(4);
rotate_acty = r2t(roty((mod(ind,200)-100)*scale));
rotate_actz = eye(4);
end
if((mod(ind,200)<=200)&&(mod(ind,200)>=150))
rotate_actx = eye(4);
rotate_acty = r2t(roty((-1)*(mod(ind,200)-200)*scale));
rotate_actz = eye(4);
end
T_robot_end_eff_cur = T_robot_end_eff_init*rotate_actz*rotate_acty*rotate_actx;
