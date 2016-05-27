% periodically rotation test 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_robot_end_eff_init = eye(4);
for i = 1:200
    T = rotation_explore_integration(T_robot_end_eff_init,i);
    trplot(T, 'frame', 'N');
    pause(0.2);
end