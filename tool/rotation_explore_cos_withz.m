% Generate the open loop rotation exploring action(cos function), given the initialized robot end effector
% pose.
%
% [T_robot_end_eff_cur] = rotation_explore_cos_withz(T_robot_end_eff_init,ind)
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
function T_robot_end_eff_cur = rotation_explore_cos_withz(T_robot_end_eff_init,ind)
scale = 1;
HALFPI = 157;

if((mod(ind,6*HALFPI)<2*HALFPI)&&(mod(ind,6*HALFPI)>=0))
rotate_actx = r2t(rotx(cos(pi/2+1/100*mod(ind,6*HALFPI))*scale));
rotate_acty = eye(4);
rotate_actz = eye(4);
end
% if((mod(ind,6*HALFPI)<2*HALFPI)&&(mod(ind,6*HALFPI)>=HALFPI))
% rotate_actx = r2t(rotx(cos(pi/2+1/100*(mod(ind,2*HALFPI)))*scale));
% rotate_acty = eye(4);
% rotate_actz = eye(4);
% rotate_actx
% ind
% end

if((mod(ind,6*HALFPI)<4*HALFPI)&&(mod(ind,6*HALFPI)>=2*HALFPI))
rotate_actx = eye(4);
rotate_acty = r2t(roty(cos(pi/2+1/100*(mod(ind,6*HALFPI)-2*HALFPI))*scale));
rotate_actz = eye(4);
end
% if((mod(ind,6*HALFPI)<=4*HALFPI)&&(mod(ind,6*HALFPI)>=3*HALFPI))
% rotate_actx = eye(4);
% rotate_acty = r2t(roty(cos(pi/2+1/100*(mod(ind,2*HALFPI)-2*HALFPI))*scale));
% rotate_actz = eye(4);
% rotate_acty
% ind
% end
if((mod(ind,6*HALFPI)<6*HALFPI)&&(mod(ind,6*HALFPI)>=4*HALFPI))
rotate_actx = eye(4);
rotate_acty = eye(4);
rotate_actz = r2t(rotz(cos(pi/2+1/100*(mod(ind,6*HALFPI)-4*HALFPI))*scale));
end
% if((mod(ind,6*HALFPI)<=6*HALFPI)&&(mod(ind,6*HALFPI)>=5*HALFPI))
% rotate_actx = eye(4);
% rotate_acty = eye(4);
% rotate_actz = r2t(rotz(cos(pi/2+1/100*(mod(ind,6*HALFPI)-4*HALFPI))*scale));
% end
T_robot_end_eff_cur = T_robot_end_eff_init*rotate_actz*rotate_acty*rotate_actx;
