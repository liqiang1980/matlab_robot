% Generate the open loop rotation exploring action(sin function), given the initialized robot end effector
% pose.
%
% [T_robot_end_eff_cur] = rotation_explore_sin_constv(T_robot_end_eff_init,ind)
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
function T_robot_end_eff_cur = rotation_explore_constv(T_robot_end_eff_init,ind)
scale = 0.00005;
if((mod(ind,300)<50)&&(mod(ind,300)>=0))
rotate_actx = r2t(rotx(mod(ind,100)*scale));
rotate_acty = eye(4);
rotate_actz = eye(4);
end
if((mod(ind,300)<100)&&(mod(ind,300)>=50))
rotate_actx = r2t(rotx((100-mod(ind,100))*scale));
rotate_acty = eye(4);
rotate_actz = eye(4);
end
if((mod(ind,300)<150)&&(mod(ind,300)>=100))
rotate_actx = eye(4);
rotate_acty = r2t(roty(mod(ind,100)*scale));
rotate_actz = eye(4);
end
if((mod(ind,300)<=200)&&(mod(ind,300)>=150))
rotate_actx = eye(4);
rotate_acty = r2t(roty((100-mod(ind,100))*scale));
rotate_actz = eye(4);
end
if((mod(ind,300)<250)&&(mod(ind,300)>=200))
rotate_actx = eye(4);
rotate_acty = eye(4);
rotate_actz = r2t(rotz(mod(ind,100)*scale));
end
if((mod(ind,300)<=300)&&(mod(ind,300)>=250))
rotate_actx = eye(4);
rotate_acty = eye(4);
rotate_actz = r2t(rotz((100-mod(ind,100))*scale));
end
T_robot_end_eff_cur = T_robot_end_eff_init*rotate_actz*rotate_acty*rotate_actx;
