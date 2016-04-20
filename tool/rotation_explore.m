% Generate the open loop rotation exploring action, given the initialized robot end effector
% pose.
%
% [T_robot_end_eff_cur] = rotation_explore(T_robot_end_eff_init)
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
function T_robot_end_eff_cur = rotation_explore(T_robot_end_eff_init)
rotate_actx = r2t(rotz(0.1*rand));
rotate_acty = r2t(roty(0.1*rand));
rotate_actz = r2t(rotx(0.1*rand));
T_robot_end_eff_cur = T_robot_end_eff_init*rotate_actz*rotate_acty*rotate_actx;
