% periodically rotation test 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sponsered by DFG spp-1527: autonmous learning
% author: Qiang Li, Bielefeld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_robot_end_eff_init = eye(4);
T_old = T_robot_end_eff_init;
for i = 1:600
    T_cur = rotation_explore_sin(T_robot_end_eff_init,i);
    [omega_s,omega_b] = est_rotation_rate(T_cur,T_old);
    omega_s
    i
    T_old = T_cur;
    trplot(T_cur, 'frame', 'N');
    pause(0.2);
end